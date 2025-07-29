# Antenna Tracker (Calibration-aware, GPS-driven)
# Adds az/el zero offset + inversion + windowed mapping to servo range

import threading
import time
import math
import azi_elev_5 as tracker
import RPi.GPIO as GPIO
from pymavlink import mavutil
import csv
from datetime import datetime

# === Configuration ===
DEBUG = True
VERBOSE_GPS = False
VERBOSE_MOVEMENT = False
USE_SMOOTH_MOVEMENT = False    # False: Snap to target
LOG_TO_CSV = True

CONNECTION_TIMEOUT = 10
GPS_TIMEOUT = 5
TRACKING_UPDATE_RATE = 0.5     # seconds between target recompute

# --- Mechanics ---
GEAR_RATIO_AZ = 2.0            # physical_out : servo (pan)
GEAR_RATIO_EL = 2.0            # physical_out : servo (tilt)  <-- change if different
SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 120.0          # typical hobby servo usable range
PHYS_AZ_MIN = SERVO_MIN_DEG * GEAR_RATIO_AZ
PHYS_AZ_MAX = SERVO_MAX_DEG * GEAR_RATIO_AZ   # 0..240° physical pan window
PHYS_EL_MIN = SERVO_MIN_DEG * GEAR_RATIO_EL
PHYS_EL_MAX = SERVO_MAX_DEG * GEAR_RATIO_EL   # 0..240° physical tilt window (we only use 0..90 world)

# --- Calibration (world -> mechanism) ---
# World az 0° = True North. If your mechanism's "physical 0°" points East, set +90 here, etc.
AZIMUTH_ZERO_OFFSET_DEG = 0.0      # add to WORLD az before mapping into physical window
ELEVATION_ZERO_OFFSET_DEG = 0.0    # add to WORLD el before mapping (usually 0)

# If gear/mount reverses sense: set to True
AZIMUTH_INVERT = True
ELEVATION_INVERT = False

# Motion / filtering
STEP_SIZE = 1.0
MIN_DEGREE_DELTA_SERVO = 5.0  # minimum change (servo-side degrees) to send a new PWM

# === Logging Setup ===
if LOG_TO_CSV:
    log_file = open("antenna_tracking_log.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "Time", "Base Lat", "Base Lon", "Base Alt",
        "Drone Lat", "Drone Lon", "Drone Alt",
        "Elevation_cmd_world", "Azimuth_cmd_world",
        "Az_phys", "El_phys", "Az_servo", "El_servo",
        "Distance", "Slant Distance"
    ])

# === Servo Logical Angles (world frame, for smooth mode bookkeeping) ===
servo_logical_azimuth_angle = 90.0   # 0–360° (world)
servo_elevation_angle = 45.0         # 0–90°  (world)

# === Previous servo-side angles for delta filter ===
prev_servo_az = None
prev_servo_el = None

# === GPS State ===
drone_gps = {"lat": None, "lon": None, "alt": None}
base_gps = {
    "lat": 13.0276844,
    "lon": 77.5631084,
    "alt": 931.13
}
drone_gps_lock = threading.Lock()

# === Debugging helpers ===
def debug(msg, level="INFO"):
    if DEBUG:
        print(f"[{time.strftime('%H:%M:%S')}] [{level}] {msg}")

def gps_print(msg):
    if VERBOSE_GPS:
        debug(msg, "GPS")

def move_print(msg):
    if VERBOSE_MOVEMENT:
        debug(msg, "MOVE")

# === GPIO Setup ===
try:
    GPIO.setmode(GPIO.BCM)
    SERVO_AZI_PIN = 18
    SERVO_ELE_PIN = 13
    GPIO.setup(SERVO_AZI_PIN, GPIO.OUT)
    GPIO.setup(SERVO_ELE_PIN, GPIO.OUT)
    pwm_azi = GPIO.PWM(SERVO_AZI_PIN, 50)  # 50 Hz
    pwm_ele = GPIO.PWM(SERVO_ELE_PIN, 50)
    pwm_azi.start(0)
    pwm_ele.start(0)
    debug("GPIO setup complete")
except Exception as e:
    debug(f"GPIO setup error: {e}", "ERROR")
    raise

# === MAVLink Setup ===
def connect_mavlink():
    try:
        mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
        print("Waiting for heartbeat...")
        mav.wait_heartbeat()
        print(f"Connected to system (system ID: {mav.target_system}, component ID: {mav.target_component})")

        # Request GLOBAL_POSITION_INT at 2 Hz (500000 μs)
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            500000, 0, 0, 0, 0, 0
        )
        return mav
    except Exception as e:
        debug(f"MAVLink connection failed: {e}", "ERROR")
        return None

mav_drone = connect_mavlink()

# === Angle utilities ===
def norm360(x):
    return (x + 360.0) % 360.0

def apply_calibration_world_to_physical(az_world, el_world):
    """
    Convert world angles to *physical output* angles (post-gear), applying:
      - zero offsets
      - inversion
      - az windowing to [PHYS_AZ_MIN, PHYS_AZ_MAX]
    Returns: (az_phys, el_phys)
    """

    # 1) World -> calibrated world
    az_w = norm360(az_world + AZIMUTH_ZERO_OFFSET_DEG)
    el_w = el_world + ELEVATION_ZERO_OFFSET_DEG

    if AZIMUTH_INVERT:
        az_w = norm360(360.0 - az_w)
    if ELEVATION_INVERT:
        el_w = -el_w

    # 2) World (calibrated) -> physical output angles (deg)
    # Pan: map 0..360 world to the *mechanical window* 0..PHYS_AZ_MAX
    # NOTE: This mount only has ~240° pan. Ensure your region of interest lies within that window
    az_phys = az_w
    # Try to fold into [0, PHYS_AZ_MAX] by adding/subtracting 360 once if helpful
    if az_phys > PHYS_AZ_MAX:
        cand = az_phys - 360.0
        if PHYS_AZ_MIN <= cand <= PHYS_AZ_MAX:
            az_phys = cand

    # Final clamp to window (warn if out-of-range)
    if az_phys < PHYS_AZ_MIN or az_phys > PHYS_AZ_MAX:
        debug(f"Target AZ {az_world:.2f}° (cal {az_w:.2f}°) is outside physical pan window "
              f"[{PHYS_AZ_MIN:.1f}, {PHYS_AZ_MAX:.1f}]° after calibration; clamping.", "WARN")
        az_phys = max(PHYS_AZ_MIN, min(PHYS_AZ_MAX, az_phys))

    # Tilt: world el is already clamped (0..90) by tracker, apply offset/invert then clamp to physical
    el_phys = max(PHYS_EL_MIN, min(PHYS_EL_MAX, el_w))

    return az_phys, el_phys

def phys_to_servo(az_phys, el_phys):
    """Convert physical output angles to servo-side angles."""
    az_servo = az_phys / GEAR_RATIO_AZ
    el_servo = el_phys / GEAR_RATIO_EL
    # Clamp to servo limits
    az_servo = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, az_servo))
    el_servo = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, el_servo))
    return az_servo, el_servo

def servo_to_pwm_duty(servo_deg):
    # Map 0..SERVO_MAX_DEG => 5%..10%
    return 5.0 + (servo_deg * 5.0 / SERVO_MAX_DEG)

# === Servo Control (world angles in, PWM out) ===
def set_angle(world_az, world_el):
    global prev_servo_az, prev_servo_el

    try:
        # Map world -> physical -> servo
        az_phys, el_phys = apply_calibration_world_to_physical(world_az, world_el)
        servo_az, servo_el = phys_to_servo(az_phys, el_phys)

        # Minimum change filter (servo-side degrees)
        if (prev_servo_az is not None and abs(servo_az - prev_servo_az) < MIN_DEGREE_DELTA_SERVO and
            prev_servo_el is not None and abs(servo_el - prev_servo_el) < MIN_DEGREE_DELTA_SERVO):
            debug("Angle change below threshold — skipping servo update")
            return

        duty_az = servo_to_pwm_duty(servo_az)
        duty_el = servo_to_pwm_duty(servo_el)

        # ----- Existing debug lines (gated by DEBUG) -----
        debug(f"WORLD cmd  → Az {world_az:.2f}°, El {world_el:.2f}°")
        debug(f"PHYS  cmd  → Az {az_phys:.2f}°, El {el_phys:.2f}°  | window [{PHYS_AZ_MIN:.0f},{PHYS_AZ_MAX:.0f}]")
        debug(f"SERVO cmd → Az {servo_az:.2f}°, El {servo_el:.2f}°  | duty {duty_az:.2f}%, {duty_el:.2f}%")

        # ===== NEW: Always print what is effectively passed to the "set_servo" and the physical target =====
        print(f"[SET_SERVO] AZ: servo_input={servo_az:.2f}°, phys_target={az_phys:.2f}°, duty={duty_az:.2f}%")
        print(f"[SET_SERVO] EL: servo_input={servo_el:.2f}°, phys_target={el_phys:.2f}°, duty={duty_el:.2f}%")
        # ================================================================================================

        pwm_azi.ChangeDutyCycle(duty_az)
        pwm_ele.ChangeDutyCycle(duty_el)
        time.sleep(0.5)
        pwm_azi.ChangeDutyCycle(0)
        pwm_ele.ChangeDutyCycle(0)

        prev_servo_az = servo_az
        prev_servo_el = servo_el

        # CSV: emit both world and servo/phys for debugging alignment
        if LOG_TO_CSV:
            with drone_gps_lock:
                log_writer.writerow([
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    base_gps["lat"], base_gps["lon"], base_gps["alt"],
                    drone_gps["lat"], drone_gps["lon"], drone_gps["alt"],
                    world_el, world_az, az_phys, el_phys, servo_az, servo_el, "", ""
                ])
                log_file.flush()

    except Exception as e:
        debug(f"Servo error: {e}", "ERROR")

# === GPS Thread ===
def update_gps(mav, gps_dict, lock):
    debug("Drone GPS thread started")
    errors = 0
    while errors < 10:
        try:
            msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=GPS_TIMEOUT)
            if msg:
                with lock:
                    gps_dict["lat"] = msg.lat / 1e7
                    gps_dict["lon"] = msg.lon / 1e7
                    gps_dict["alt"] = msg.alt / 1000.0
                gps_print(f"[Drone] Lat: {gps_dict['lat']}, Lon: {gps_dict['lon']}, Alt: {gps_dict['alt']} m")
                errors = 0
            else:
                debug("Drone GPS timeout", "WARN")
                errors += 1
        except Exception as e:
            debug(f"Drone GPS error: {e}", "ERROR")
            errors += 1
            time.sleep(1)

# === Movement Logic ===
def move_to(az_target_world, el_target_world, step=STEP_SIZE, delay=0.05):
    global servo_logical_azimuth_angle, servo_elevation_angle

    if not USE_SMOOTH_MOVEMENT:
        servo_logical_azimuth_angle = az_target_world
        servo_elevation_angle = el_target_world
        # keep tracker’s elevation clamp but don’t ceil
        adj_az, adj_el = tracker.adjust_angles_for_servo_limits(az_target_world, el_target_world)
        set_angle(adj_az, adj_el)
        move_print(f"Snapped → Az: {adj_az:.2f}°, El: {adj_el:.2f}°")
        return

    # Smooth mode (world frame bookkeeping)
    max_steps = 500
    for _ in range(max_steps):
        delta_az = (az_target_world - servo_logical_azimuth_angle + 540) % 360 - 180
        delta_el = el_target_world - servo_elevation_angle

        if abs(delta_az) <= 1 and abs(delta_el) <= 1:
            debug("Target reached")
            break

        step_az = step if delta_az > 0 else -step if abs(delta_az) > step else delta_az
        step_el = step if delta_el > 0 else -step if abs(delta_el) > step else delta_el

        servo_logical_azimuth_angle = (servo_logical_azimuth_angle + step_az) % 360
        servo_elevation_angle = max(0, min(180, servo_elevation_angle + step_el))

        adj_az, adj_el = tracker.adjust_angles_for_servo_limits(
            servo_logical_azimuth_angle, servo_elevation_angle)

        set_angle(adj_az, adj_el)
        move_print(f"Moved → Az: {adj_az:.2f}°, El: {adj_el:.2f}°")
        time.sleep(delay)

# === Tracking and Logging ===
def calculate_tracking_angles():
    with drone_gps_lock:
        if not all([drone_gps["lat"], drone_gps["lon"], drone_gps["alt"]]):
            debug("Drone GPS missing", "WARN")
            return None
        try:
            info = tracker.get_tracking_info(
                base_gps["lat"], base_gps["lon"], base_gps["alt"],
                drone_gps["lat"], drone_gps["lon"], drone_gps["alt"]
            )
            if info:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] Azimuth: {info['azimuth']}°, Elevation: {info['elevation']}°")
                return info
            else:
                return None
        except Exception as e:
            debug(f"Angle calculation error: {e}", "ERROR")
            return None

def log_to_csv_tracking(info):
    if not LOG_TO_CSV:
        return
    try:
        with drone_gps_lock:
            log_writer.writerow([
                datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                base_gps["lat"], base_gps["lon"], base_gps["alt"],
                drone_gps["lat"], drone_gps["lon"], drone_gps["alt"],
                info["elevation"], info["azimuth"], "", "", "", "",
                info["horizontal_distance"], info["slant_range"]
            ])
            log_file.flush()
    except Exception as e:
        debug(f"Log error: {e}", "ERROR")

def tracking_loop():
    debug("Tracking loop started")
    while True:
        try:
            info = calculate_tracking_angles()
            if info:
                # Log raw tracking numbers (world frame + ranges)
                log_to_csv_tracking(info)
                # Move using world angles; set_angle handles calibration window
                move_to(info["adjusted_azimuth"], info["adjusted_elevation"])
            time.sleep(TRACKING_UPDATE_RATE)
        except KeyboardInterrupt:
            debug("Stopped by user")
            break
        except Exception as e:
            debug(f"Loop error: {e}", "ERROR")
            time.sleep(5)

# === Cleanup ===
def cleanup():
    debug("Cleaning up GPIO and logging")
    try:
        pwm_azi.stop()
        pwm_ele.stop()
        GPIO.cleanup()
        if LOG_TO_CSV:
            log_file.close()
    except Exception as e:
        debug(f"Cleanup error: {e}", "ERROR")

# === Entry Point ===
def main():
    debug("Starting Antenna Tracker")
    # Move to a known staring pose (uses calibration)
    set_angle(servo_logical_azimuth_angle, servo_elevation_angle)
    if mav_drone:
        threading.Thread(target=update_gps, args=(mav_drone, drone_gps, drone_gps_lock), daemon=True).start()
    time.sleep(5)
    tracking_loop()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        debug("Shutdown by user")
    finally:
        cleanup()
