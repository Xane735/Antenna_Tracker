# Antenna Tracker (lauki.py) — FINAL VERSION

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
USE_SMOOTH_MOVEMENT = True  # False: Snap to target
LOG_TO_CSV = True           # False: Disable CSV logging

CONNECTION_TIMEOUT = 10
GPS_TIMEOUT = 5
TRACKING_UPDATE_RATE = 1
GEAR_RATIO = 2.0
STEP_SIZE = 1.0

# === Logging Setup
if LOG_TO_CSV:
    log_file = open("antenna_tracking_log.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "timestamp",
        "drone_lat", "drone_lon", "drone_alt",
        "base_lat", "base_lon", "base_alt",
        "azimuth", "elevation",
        "servo_azimuth_input", "servo_elevation_input",
        "horizontal_distance", "slant_range"
    ])

# === Servo Logical Angles
servo_logical_azimuth_angle = 90.0  # 0–360° azimuth
servo_elevation_angle = 45.0        # 0–90° elevation

# === GPS State
drone_gps = {"lat": None, "lon": None, "alt": None}
base_gps = {"lat": None, "lon": None, "alt": None}
drone_gps_lock = threading.Lock()
base_gps_lock = threading.Lock()

# === Debugging
def debug(msg, level="INFO"):
    if DEBUG:
        print(f"[{time.strftime('%H:%M:%S')}] [{level}] {msg}")

def gps_print(msg):
    if VERBOSE_GPS:
        debug(msg, "GPS")

def move_print(msg):
    if VERBOSE_MOVEMENT:
        debug(msg, "MOVE")

# === GPIO Setup
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

# === MAVLink Setup
def connect_mavlink():
    try:
        mav_drone = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
        mav_base = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
        debug("MAVLink ports connected")

        mav_drone.wait_heartbeat(timeout=CONNECTION_TIMEOUT)
        debug("Drone heartbeat received")
        mav_base.wait_heartbeat(timeout=CONNECTION_TIMEOUT)
        debug("Base heartbeat received")
        return mav_drone, mav_base
    except Exception as e:
        debug(f"MAVLink connection failed: {e}", "ERROR")
        return None, None

mav_drone, mav_base = connect_mavlink()

# === Servo Control
def set_angle(logical_az, elevation):
    try:
        physical_az = logical_az / GEAR_RATIO
        physical_el = elevation / GEAR_RATIO

        duty_az = 2.5 + (physical_az * 10.0 / 180.0)
        duty_el = 2.5 + (physical_el * 10.0 / 180.0)

        duty_az = max(2.5, min(12.5, duty_az))
        duty_el = max(2.5, min(12.5, duty_el))

        pwm_azi.ChangeDutyCycle(duty_az)
        pwm_ele.ChangeDutyCycle(duty_el)
        debug(f"Set angles, Az: {logical_az:.2f}°, El: {elevation:.2f}°")

        time.sleep(0.5)
        pwm_azi.ChangeDutyCycle(0)
        pwm_ele.ChangeDutyCycle(0)
    except Exception as e:
        debug(f"Servo error: {e}", "ERROR")

# === GPS Threads
def update_gps(mav, gps_dict, lock, label):
    debug(f"{label} GPS thread started")
    errors = 0
    while errors < 10:
        try:
            msg = mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=GPS_TIMEOUT)
            if msg:
                with lock:
                    gps_dict["lat"] = msg.lat / 1e7
                    gps_dict["lon"] = msg.lon / 1e7
                    gps_dict["alt"] = msg.alt / 1000.0
                gps_print(f"[{label}] Lat: {gps_dict['lat']}, Lon: {gps_dict['lon']}, Alt: {gps_dict['alt']} m")
                errors = 0
            else:
                debug(f"{label} GPS timeout", "WARN")
                errors += 1
        except Exception as e:
            debug(f"{label} GPS error: {e}", "ERROR")
            errors += 1
            time.sleep(1)

# === Movement Logic
def move_to(az_target, el_target, step=STEP_SIZE, delay=0.05):
    global servo_logical_azimuth_angle, servo_elevation_angle

    if not USE_SMOOTH_MOVEMENT:
        servo_logical_azimuth_angle = az_target
        servo_elevation_angle = el_target
        adj_az, adj_el = tracker.adjust_angles_for_servo_limits(az_target, el_target)
        set_angle(adj_az, adj_el)
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

# === Angle Calculation
def calculate_tracking_angles():
    with drone_gps_lock, base_gps_lock:
        if not all([drone_gps["lat"], drone_gps["lon"], drone_gps["alt"]]):
            debug("Drone GPS missing", "WARN")
            return None
        try:
            info = tracker.get_tracking_info(
                base_gps["lat"], base_gps["lon"], base_gps["alt"],
                drone_gps["lat"], drone_gps["lon"], drone_gps["alt"]
            )
            if info:
                debug(f"Tracking Az: {info['azimuth']:.2f}°, El: {info['elevation']:.2f}°")
                return info
            else:
                return None
        except Exception as e:
            debug(f"Angle calculation error: {e}", "ERROR")
            return None

# === Logging
def log_to_csv(info):
    if not LOG_TO_CSV:
        return
    try:
        with drone_gps_lock, base_gps_lock:
            log_writer.writerow([
                datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                drone_gps["lat"], drone_gps["lon"], drone_gps["alt"],
                base_gps["lat"], base_gps["lon"], base_gps["alt"],
                info["azimuth"], info["elevation"],
                round(info["adjusted_azimuth"] / GEAR_RATIO, 2),
                round(info["adjusted_elevation"] / GEAR_RATIO, 2),
                info["horizontal_distance"], info["slant_range"]
            ])
            log_file.flush()
    except Exception as e:
        debug(f"Log error: {e}", "ERROR")

# === Tracking Loop
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

# === Cleanup
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

# === Entry Point
def main():
    debug("Starting Antenna Tracker")
    set_angle(servo_logical_azimuth_angle, servo_elevation_angle)
    if mav_drone:
        threading.Thread(target=update_gps, args=(mav_drone, drone_gps, drone_gps_lock, "Drone"), daemon=True).start()
    if mav_base:
        threading.Thread(target=update_gps, args=(mav_base, base_gps, base_gps_lock, "Base"), daemon=True).start()
    time.sleep(5)
    tracking_loop()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        debug("Shutdown by user")
    finally:
        cleanup()
