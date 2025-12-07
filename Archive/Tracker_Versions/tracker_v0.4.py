# Antenna Tracker (Simulation-Ready Version)
# WORRKSSSS!!!!

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
USE_SMOOTH_MOVEMENT = False  # False: Snap to target
LOG_TO_CSV = True           # False: Disable CSV logging

CONNECTION_TIMEOUT = 10
GPS_TIMEOUT = 5
TRACKING_UPDATE_RATE = 0.5
GEAR_RATIO = 2.0
STEP_SIZE = 1.0
MIN_DEGREE_DELTA = 1.0  # Minimum change in angle to trigger servo movement

# === Logging Setup ===
if LOG_TO_CSV:
    log_file = open("antenna_tracking_log.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "Time", "Base Lat", "Base Lon", "Base Alt",
        "Drone Lat", "Drone Lon", "Drone Alt",
        "Elevation", "Azimuth", "Distance", "Slant Distance"
    ])

# === Servo Logical Angles ===
servo_logical_azimuth_angle = 90.0  # 0–360° azimuth
servo_elevation_angle = 45.0        # 0–90° elevation

# === Previous Physical Angles for Filtering ===
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

# === Debugging ===
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
    pwm_azi = GPIO.PWM(SERVO_AZI_PIN, 50)
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

        # Request GLOBAL_POSITION_INT at 2Hz (500000 microseconds)
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

# === Servo Control ===
def set_angle(logical_az, elevation):
    global prev_servo_az, prev_servo_el

    try:
        # Calculate physical servo angle based on gear ratio
        servo_az = logical_az / GEAR_RATIO
        servo_el = elevation / GEAR_RATIO

        # Clamp to 0–120° range
        servo_az = max(0, min(120, servo_az))
        servo_el = max(0, min(120, servo_el))

        # Check for significant change
        if prev_servo_az is not None and abs(servo_az - prev_servo_az) < MIN_DEGREE_DELTA and \
           prev_servo_el is not None and abs(servo_el - prev_servo_el) < MIN_DEGREE_DELTA:
            debug("Angle change below threshold — skipping servo update")
            return

        # Convert to duty cycle (5% to 10%)
        duty_az = 5 + (servo_az * 5.0 / 120.0)
        duty_el = 5 + (servo_el * 5.0 / 120.0)

        debug(f"Input angles => Logical Az: {logical_az:.2f}°, Elevation: {elevation:.2f}°")
        debug(f"Servo input angles => Az: {servo_az:.2f}°, El: {servo_el:.2f}°")
        debug(f"Duty cycles => Az: {duty_az:.2f}%, El: {duty_el:.2f}%")

        pwm_azi.ChangeDutyCycle(duty_az)
        pwm_ele.ChangeDutyCycle(duty_el)

        time.sleep(0.5)
        pwm_azi.ChangeDutyCycle(0)
        pwm_ele.ChangeDutyCycle(0)

        # Update previous values
        prev_servo_az = servo_az
        prev_servo_el = servo_el

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
def move_to(az_target, el_target, step=STEP_SIZE, delay=0.05):
    global servo_logical_azimuth_angle, servo_elevation_angle

    if not USE_SMOOTH_MOVEMENT:
        servo_logical_azimuth_angle = az_target
        servo_elevation_angle = el_target
        adj_az, adj_el = tracker.adjust_angles_for_servo_limits(az_target, el_target)
        set_angle(math.ceil(adj_az), math.ceil(adj_el))
        move_print(f"Snapped → Az: {adj_az:.2f}°, El: {adj_el:.2f}°")
        return

    max_steps = 500
    for _ in range(max_steps):
        delta_az = (az_target - servo_logical_azimuth_angle + 540) % 360 - 180
        delta_el = el_target - servo_elevation_angle

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

def log_to_csv(info):
    if not LOG_TO_CSV:
        return
    try:
        with drone_gps_lock:
            log_writer.writerow([
                datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                base_gps["lat"], base_gps["lon"], base_gps["alt"],
                drone_gps["lat"], drone_gps["lon"], drone_gps["alt"],
                info["elevation"], info["azimuth"],
                info["horizontal_distance"], info["slant_range"]
            ])
            log_file.flush()
    except Exception as e:
        debug(f"Log error: {e}", "ERROR")

# === Tracking Loop ===
def tracking_loop():
    debug("Tracking loop started")
    while True:
        try:
            info = calculate_tracking_angles()
            if info:
                log_to_csv(info)
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
