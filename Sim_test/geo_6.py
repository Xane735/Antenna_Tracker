#!/usr/bin/env python3
# geo_4_pigpio_debug.py
# Antenna Tracker (GPS-driven) — pigpio edition with rich debugging & logging
#
# - PAN: full 360° physical with 2:1 gearing (servo 0..180°)
# - TILT: 2:1 gearing (world 0..90° -> servo 0..45°); safe clamp to 0..180° phys
# - Exact 1000–2000 µs pulses via pigpio
# - Independent per-axis updates with a shared settle time
# - MAVLink GPS input (GLOBAL_POSITION_INT)
# - CSV logging with detailed mapping & pulses to diagnose "why not 360° in mission"

import threading
import time
import math
import csv
from datetime import datetime

import pigpio
from pymavlink import mavutil

# Angle math (your tested module)
import azi_elev_5 as tracker

# ===================== Configuration =====================
DEBUG = True
VERBOSE_GPS = False
VERBOSE_MOVEMENT = False
LOG_TO_CSV = True

GPS_TIMEOUT = 5
TRACKING_UPDATE_RATE = 0.5     # seconds between target recompute
SETTLE_TIME_SEC = 0.5          # shared settle time (seconds)

# Az & El gearing
GEAR_RATIO_AZ = 2.0            # physical_out : servo
GEAR_RATIO_EL = 2.0
last_pulse_az_us = None
last_pulse_el_us = None

# Servo ranges
SERVO_MAX_DEG_AZ = 180.0
SERVO_MAX_DEG_EL = 180.0

# Physical limits (info / clamps for tilt)
PHYS_AZ_MIN = 0.0
PHYS_AZ_MAX = 360.0
PHYS_EL_MIN = 0.0
PHYS_EL_MAX = 180.0

# PWM endpoints (µs) — matches your servo tester
PULSE_MIN_US = 1000.0
PULSE_MAX_US = 2000.0

# Per-axis minimum-change filter (in SERVO-side degrees)
MIN_DEGREE_DELTA_SERVO = 4.5

# Calibration (world -> mechanism)
AZIMUTH_ZERO_OFFSET_DEG   = 0.0
ELEVATION_ZERO_OFFSET_DEG = 0.0
AZIMUTH_INVERT   = True
ELEVATION_INVERT = False

# GPIO pins (BCM)
SERVO_AZI_PIN = 18
SERVO_ELE_PIN = 13

# Startup pose (world)
INITIAL_WORLD_AZ = 0.0
INITIAL_WORLD_EL = 20.0

# Base station (ASL)
base_gps = {
    "lat": 13.0272176,
    "lon": 77.5630984,
    "alt": 931.13
}

# Debug helpers / options
HOLD_PULSES = True           # if True, keep pulses ON after update (no stop)
RUN_STARTUP_PAN_TEST = False  # if True, do a quick 0/90/180/270/359 test at boot

# ===================== State & Logging =====================
prev_servo_az = None
prev_servo_el = None
prev_world_az = None
prev_servo_az_for_delta = None
prev_step_time = None

drone_gps = {"lat": None, "lon": None, "alt": None}
drone_gps_lock = threading.Lock()

log_writer = None
log_file = None
if LOG_TO_CSV:
    log_file = open(f"antenna_tracking_log_debug-{datetime.now().strftime('%Y%m%d-%H%M%S')}.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        # Time & positions
        "Time", "BaseLat", "BaseLon", "BaseAlt",
        "DroneLat", "DroneLon", "DroneAlt",
        # World angles (raw from tracker, and adjusted)
        "WorldAzRaw", "WorldElRaw", "WorldAzAdj", "WorldElAdj",
        # Calibrated world after offset/invert (before gearing)
        "AzWCal", "ElWCal",
        # Physical and servo targets (computed this cycle)
        "AzPhys", "ElPhys", "ServoAzDeg", "ServoElDeg",
        # Commanded pulses this cycle (0 if axis not updated due to threshold)
        "PulseAz_us_cmd", "PulseEl_us_cmd",
        # Which axes we *decided* to update this cycle
        "UpdAz", "UpdEl",
        # Effective pulses actually on pins (read-back from pigpio)
        "PulseAz_us_eff", "PulseEl_us_eff",
        # Effective servo angles from eff pulses (deg)
        "ServoAzDeg_eff", "ServoElDeg_eff",
        # Deltas & wrap detection (world / physical)
        "dWorldAz_deg", "dServoAz_phys_deg", "WrapEvent",
        # Timing
        "tSinceLast_s"
    ])

# ===================== Utilities =====================
def debug(msg, level="INFO"):
    if DEBUG:
        print(f"[{time.strftime('%H:%M:%S')}] [{level}] {msg}")

def gps_print(msg):
    if VERBOSE_GPS:
        debug(msg, "GPS")

def move_print(msg):
    if VERBOSE_MOVEMENT:
        debug(msg, "MOVE")

def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

def ang_diff_deg(a_new: float, a_old: float) -> float:
    """Shortest signed difference a_new - a_old (deg) in (-180, +180]."""
    return ((a_new - a_old + 180.0) % 360.0) - 180.0

def print_config():
    debug("=== CONFIG DUMP ===")
    debug(f"AZ gear={GEAR_RATIO_AZ}:1, EL gear={GEAR_RATIO_EL}:1")
    debug(f"Servo max: AZ={SERVO_MAX_DEG_AZ}°, EL={SERVO_MAX_DEG_EL}°")
    debug(f"Physical pan range: {PHYS_AZ_MIN}..{PHYS_AZ_MAX}°")
    debug(f"PWM µs: min={PULSE_MIN_US}, max={PULSE_MAX_US}")
    debug(f"Offsets: AZ={AZIMUTH_ZERO_OFFSET_DEG}°, EL={ELEVATION_ZERO_OFFSET_DEG}°")
    debug(f"Invert: AZ={AZIMUTH_INVERT}, EL={ELEVATION_INVERT}")
    debug(f"Threshold (servo-side): {MIN_DEGREE_DELTA_SERVO}°")
    debug(f"HOLD_PULSES={HOLD_PULSES}, STARTUP_TEST={RUN_STARTUP_PAN_TEST}")
    debug("====================")

# ===================== Mapping =====================
def calibrate_world(az_world: float, el_world: float):
    """Apply zero offsets and inversion; return (az_w, el_w) in world frame."""
    az_w = norm360(az_world + AZIMUTH_ZERO_OFFSET_DEG)
    el_w = el_world + ELEVATION_ZERO_OFFSET_DEG
    if AZIMUTH_INVERT:
        az_w = norm360(360.0 - az_w)
    if ELEVATION_INVERT:
        el_w = -el_w
    return az_w, el_w

def apply_calibration_world_to_physical(az_world: float, el_world: float):
    """
    World -> calibrated world (offset/invert) -> PHYSICAL (post-gear windowless).
    Returns (az_phys, el_phys, az_w, el_w).
    """
    az_w, el_w = calibrate_world(az_world, el_world)
    az_phys = az_w                         # full 0..360 valid
    el_phys = max(PHYS_EL_MIN, min(PHYS_EL_MAX, el_w))  # clamp for safety
    return az_phys, el_phys, az_w, el_w

def phys_to_servo(az_phys: float, el_phys: float):
    """PHYSICAL -> SERVO (deg). With 2:1 gearing: servo = physical / 2."""
    az_servo = az_phys / GEAR_RATIO_AZ
    el_servo = el_phys / GEAR_RATIO_EL
    az_servo = max(0.0, min(SERVO_MAX_DEG_AZ, az_servo))
    el_servo = max(0.0, min(SERVO_MAX_DEG_EL, el_servo))
    return az_servo, el_servo

def servo_deg_to_us(servo_deg: float, servo_max_deg: float) -> float:
    """Linear: 0..servo_max_deg -> 1000..2000 µs."""
    servo_deg = max(0.0, min(servo_max_deg, float(servo_deg)))
    return PULSE_MIN_US + (servo_deg / servo_max_deg) * (PULSE_MAX_US - PULSE_MIN_US)

# ===================== pigpio Setup =====================
def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running. Start it with: sudo pigpio")
    pi.set_mode(SERVO_AZI_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_ELE_PIN, pigpio.OUTPUT)
    debug("GPIO (pigpio) setup complete")
    return pi

# ===================== Servo Control =====================
def set_angle(world_az: float, world_el: float, pi: pigpio.pi):
    """
    Compute servo targets from world angles and update axes independently.
    Re-apply pulses every cycle so the servo is continuously driven.
    Log commanded vs. effective pulses and angles.
    """
    global prev_servo_az, prev_servo_el, prev_world_az, prev_servo_az_for_delta, prev_step_time
    global last_pulse_az_us, last_pulse_el_us

    try:
        t_now = time.time()

        # --- World -> physical -> servo ---
        az_phys, el_phys, az_w, el_w = apply_calibration_world_to_physical(world_az, world_el)
        servo_az, servo_el = phys_to_servo(az_phys, el_phys)

        # --- Threshold (servo-side) ---
        update_az = (prev_servo_az is None) or (abs(servo_az - prev_servo_az) >= MIN_DEGREE_DELTA_SERVO)
        update_el = (prev_servo_el is None) or (abs(servo_el - prev_servo_el) >= MIN_DEGREE_DELTA_SERVO)

        # --- Compute commanded pulses (only change if updating) ---
        pulse_az_cmd = 0.0
        pulse_el_cmd = 0.0

        if last_pulse_az_us is None:
            last_pulse_az_us = servo_deg_to_us(servo_az, SERVO_MAX_DEG_AZ)
        if last_pulse_el_us is None:
            last_pulse_el_us = servo_deg_to_us(servo_el, SERVO_MAX_DEG_EL)

        if update_az:
            last_pulse_az_us = servo_deg_to_us(servo_az, SERVO_MAX_DEG_AZ)
            pulse_az_cmd = last_pulse_az_us  # for logging
        if update_el:
            last_pulse_el_us = servo_deg_to_us(servo_el, SERVO_MAX_DEG_EL)
            pulse_el_cmd = last_pulse_el_us  # for logging

        # --- Always (re)apply pulses every loop (continuous hold) ---
        pi.set_servo_pulsewidth(SERVO_AZI_PIN, last_pulse_az_us)
        pi.set_servo_pulsewidth(SERVO_ELE_PIN, last_pulse_el_us)

        # --- Deltas & wrap detection ---
        wrap_event = 0
        d_world_az = 0.0
        if prev_world_az is not None:
            d_world_az = ((world_az - prev_world_az + 180.0) % 360.0) - 180.0
            if abs(d_world_az) > 180 - 1e-6:
                wrap_event = 1

        d_servo_az_phys = 0.0
        if prev_servo_az_for_delta is not None:
            # report as PHYSICAL delta
            d_servo_az_phys = (((servo_az - prev_servo_az_for_delta + 180.0) % 360.0) - 180.0) * GEAR_RATIO_AZ

        # --- Read back effective pulses actually output by pigpio ---
        pulse_az_eff = float(pi.get_servo_pulsewidth(SERVO_AZI_PIN))
        pulse_el_eff = float(pi.get_servo_pulsewidth(SERVO_ELE_PIN))

        # Convert effective pulses back to servo degrees for sanity check
        def us_to_servo_deg(us: float, servo_max_deg: float) -> float:
            us = max(PULSE_MIN_US, min(PULSE_MAX_US, us))
            return (us - PULSE_MIN_US) / (PULSE_MAX_US - PULSE_MIN_US) * servo_max_deg

        servo_az_eff = us_to_servo_deg(pulse_az_eff, SERVO_MAX_DEG_AZ)
        servo_el_eff = us_to_servo_deg(pulse_el_eff, SERVO_MAX_DEG_EL)

        # --- Diagnostics ---
        debug(f"WORLD raw  → Az {world_az:.2f}°, El {world_el:.2f}°")
        debug(f"WORLD cal  → Az {az_w:.2f}° , El {el_w:.2f}°")
        debug(f"PHYS tgt   → Az {az_phys:.2f}°, El {el_phys:.2f}°")
        debug(f"SERVO tgt  → Az {servo_az:.2f}°, El {servo_el:.2f}°")
        if update_az:
            print(f"[AZ CMD] servo={servo_az:.2f}° -> {last_pulse_az_us:.0f}us | phys={az_phys:.2f}°")
        else:
            debug("AZ below threshold — no new command")
        if update_el:
            print(f"[EL CMD] servo={servo_el:.2f}° -> {last_pulse_el_us:.0f}us | phys={el_phys:.2f}°")
        else:
            debug("EL below threshold — no new command")

        print(f"[EFF] AZ pulse={pulse_az_eff:.0f}us (~{servo_az_eff:.1f}° servo)"
              f" | EL pulse={pulse_el_eff:.0f}us (~{servo_el_eff:.1f}° servo)")

        # --- Shared settle time (kept small; pulses are held continuously anyway) ---
        time.sleep(SETTLE_TIME_SEC)

        # --- Bookkeeping ---
        if update_az: prev_servo_az = servo_az
        if update_el: prev_servo_el = servo_el
        prev_world_az = world_az
        prev_servo_az_for_delta = servo_az

        t_since_last = (t_now - prev_step_time) if prev_step_time else 0.0
        prev_step_time = t_now

        # --- CSV log (always) ---
        if LOG_TO_CSV:
            with drone_gps_lock:
                log_writer.writerow([
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    base_gps["lat"], base_gps["lon"], base_gps["alt"],
                    drone_gps["lat"], drone_gps["lon"], drone_gps["alt"],
                    # World raw & adjusted (we pass adjusted at the call)
                    world_az, world_el,
                    world_az, world_el,
                    # Calibrated world
                    az_w, el_w,
                    # Physical & servo (targets)
                    az_phys, el_phys, servo_az, servo_el,
                    # Commanded pulses this step (0 if not updated)
                    round(pulse_az_cmd, 1), round(pulse_el_cmd, 1),
                    int(update_az), int(update_el),
                    # Effective pulses and effective servo angles
                    round(pulse_az_eff, 1), round(pulse_el_eff, 1),
                    round(servo_az_eff, 2), round(servo_el_eff, 2),
                    # Deltas & wrap
                    round(d_world_az, 2), round(d_servo_az_phys, 2), wrap_event,
                    round(t_since_last, 3)
                ])
                log_file.flush()

    except Exception as e:
        debug(f"Servo error: {e}", "ERROR")

# ===================== MAVLink & GPS =====================
def connect_mavlink():
    try:
        mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
        print("Waiting for heartbeat...")
        mav.wait_heartbeat()
        print(f"Connected to system (system ID: {mav.target_system}, component ID: {mav.target_component})")
        # Request GLOBAL_POSITION_INT at 2 Hz
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

# ===================== Tracking =====================
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
                # info contains: azimuth, elevation, adjusted_azimuth, adjusted_elevation, distances...
                print(f"[{datetime.now().strftime('%H:%M:%S')}] Azimuth: {info['azimuth']}°, Elevation: {info['elevation']}°")
                return info
            else:
                return None
        except Exception as e:
            debug(f"Angle calculation error: {e}", "ERROR")
            return None

def tracking_loop(pi: pigpio.pi):
    debug("Tracking loop started")
    while True:
        try:
            info = calculate_tracking_angles()
            if info:
                # Use world "adjusted" angles from your math module (0..360 az, 0..90 el)
                world_az = info.get("adjusted_azimuth", info["azimuth"])
                world_el = info.get("adjusted_elevation", info["elevation"])
                set_angle(world_az, world_el, pi)
            time.sleep(TRACKING_UPDATE_RATE)
        except KeyboardInterrupt:
            debug("Stopped by user")
            break
        except Exception as e:
            debug(f"Loop error: {e}", "ERROR")
            time.sleep(5)

# ===================== Cleanup =====================
def cleanup(pi: pigpio.pi):
    debug("Cleaning up pigpio and logging")
    try:
        pi.set_servo_pulsewidth(SERVO_AZI_PIN, 0)
        pi.set_servo_pulsewidth(SERVO_ELE_PIN, 0)
        time.sleep(0.2)
        pi.stop()
        if LOG_TO_CSV and log_file:
            log_file.close()
    except Exception as e:
        debug(f"Cleanup error: {e}", "ERROR")

# ===================== Startup Test (optional) =====================
def startup_pan_test(pi: pigpio.pi):
    pts = [0, 90, 180, 270, 359]
    debug("Startup PAN test (0,90,180,270,359)")
    for a in pts:
        set_angle(a, 0.0, pi)
        time.sleep(0.6)

# ===================== Main =====================
def main():
    debug("Starting Antenna Tracker (pigpio)")
    print_config()
    pi = setup_pigpio()

    # Known starting pose
    set_angle(INITIAL_WORLD_AZ, INITIAL_WORLD_EL, pi)

    # Optional sanity test
    if RUN_STARTUP_PAN_TEST:
        startup_pan_test(pi)

    # MAVLink
    mav_drone = connect_mavlink()
    if mav_drone:
        threading.Thread(target=update_gps, args=(mav_drone, drone_gps, drone_gps_lock), daemon=True).start()

    # Tracking
    try:
        time.sleep(2)
        tracking_loop(pi)
    finally:
        cleanup(pi)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        debug("Shutdown by user")
