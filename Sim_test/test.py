# Antenna Tracker (Simulation-Ready Version)

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
TRACKING_UPDATE_RATE = 0.5
GEAR_RATIO = 2.0
STEP_SIZE = 1.0

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

# === GPS State ===
drone_gps = {"lat": None, "lon": None, "alt": None}
base_gps = {
    "lat": None,  # TODO: Add base latitude here (e.g., 12.9716)
    "lon": None,  # TODO: Add base longitude here (e.g., 77.5946)
    "alt": None   # TODO: Add base altitude in meters (e.g., 900.0)
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
    try:
        physical_az = logical_az / GEAR_RATIO
        physical_el = elevation / GEAR_RATIO

        duty_az = 2.5 + (physical_az * 10.0 / 180.0)
        duty_el = 2.5 + (physical_el * 10.0 / 180.0)

        duty_az = max(2.5, min(12.5, duty_az))
        duty_el = max(2.5, min(12.5, duty_el))

        debug(f"Input angles => Logical Az: {logical_az:.2f}°, Elevation: {elevation:.2f}°")
        debug(f"Physical servo angles => Az: {physical_az:.2f}°, El: {physical_el:.2f}°")

        pwm_azi.ChangeDutyCycle(duty_az)
        pwm_ele.ChangeDutyCycle(duty_el)

        time.sleep(0.5)
        pwm_azi.ChangeDutyCycle(0)
        pwm_ele.ChangeDutyCycle(0)
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
# (rest of the code remains unchanged)
