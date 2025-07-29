#!/usr/bin/env python3
# geo_4_full360.py
# Antenna Tracker (GPS-driven) — PAN 360° (2:1), TILT 2:1
# - Full-circle azimuth (no windowing / no clamping)
# - Elevation 2:1 gearing (world 0..90 -> servo 0..45)
# - 1000–2000 µs PWM mapping at 50 Hz
# - Independent per-axis updates with shared settle time
# - MAVLink GPS input (GLOBAL_POSITION_INT), CSV logging

import threading
import time
import math
import csv
from datetime import datetime

import RPi.GPIO as GPIO
from pymavlink import mavutil

# Your math module (the one you already tested)
import azi_elev_5 as tracker

# ===================== Configuration =====================
DEBUG = True
VERBOSE_GPS = False
VERBOSE_MOVEMENT = False
LOG_TO_CSV = True

GPS_TIMEOUT = 5
TRACKING_UPDATE_RATE = 1.0     # seconds between target recompute

# --- Mechanics (axis-specific) ---
# Azimuth: 2:1 gear, 180° servo -> 360° physical (assumed continuous)
GEAR_RATIO_AZ = 2.0
SERVO_MAX_DEG_AZ = 180.0
PHYS_AZ_MIN = 0.0
PHYS_AZ_MAX = 360.0   # full circle

# Elevation: 2:1 gear; world elevation 0..90 -> servo 0..45
GEAR_RATIO_EL = 2.0
SERVO_MAX_DEG_EL = 180.0
PHYS_EL_MIN = 0.0
PHYS_EL_MAX = 180.0   # safe clamp

# PWM / timing
PWM_FREQ_HZ = 50
PULSE_MIN_US = 1000.0         # 1.000 ms
PULSE_MAX_US = 2000.0         # 2.000 ms
SETTLE_TIME_SEC = 1.0         # shared settle time

# Per-axis minimum-change filter (in SERVO-side degrees)
MIN_DEGREE_DELTA_SERVO = 1.0  # your preferred 3–5° range

# --- Calibration (world -> mechanism) ---
# World az 0° = True North. Adjust these if your zero points differ.
AZIMUTH_ZERO_OFFSET_DEG = 0.0      # add to WORLD az before mapping
ELEVATION_ZERO_OFFSET_DEG = 0.0    # add to WORLD el before mapping
AZIMUTH_INVERT = True              # True if pan sense is reversed
ELEVATION_INVERT = False

# --- GPIO Pins (BCM) ---
SERVO_AZI_PIN = 18
SERVO_ELE_PIN = 13

# --- Startup pose (world angles) ---
INITIAL_WORLD_AZ = 0.0
INITIAL_WORLD_EL = 20.0

# --- Base station position (ASL) ---
base_gps = {
    "lat": 13.0272176,
    "lon": 77.5630984,
    "alt": 931.13
}

# ===================== State & Logging =====================
# Previous servo-side angles for delta filter
prev_servo_az = None
prev_servo_el = None

# GPS state
drone_gps = {"lat": None, "lon": None, "alt": None}
drone_gps_lock = threading.Lock()

# CSV logging
log_writer = None
log_file = None
if LOG_TO_CSV:
    log_file = open(f"antenna_tracking_log-{datetime.now().strftime('%Y%m%d-%H%M%S')}.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "Time", "Base Lat", "Base Lon", "Base Alt",
        "Drone Lat", "Drone Lon", "Drone Alt",
        "Elevation_cmd_world", "Azimuth_cmd_world",
        "Az_phys", "El_phys", "Az_servo", "El_servo",
        "Horizontal_Distance", "Slant_Range"
    ])

# ===================== Utils =====================
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

# ===================== Mapping =====================
def apply_calibration_world_to_physical(az_world: float, el_world: float):
    """
    World -> calibrated world (offset/invert) -> PHYSICAL.
    - PAN: treat 0..360° as valid (no window/folding).
    - TILT: clamp to physical range (0..180 by default).
    """
    # Offsets
    az_w = norm360(az_world + AZIMUTH_ZERO_OFFSET_DEG)
    el_w = el_world + ELEVATION_ZERO_OFFSET_DEG

    # Inversions
    if AZIMUTH_INVERT:
        az_w = norm360(360.0 - az_w)
    if ELEVATION_INVERT:
        el_w = -el_w

    # Physical targets
    az_phys = az_w                  # 0..360
    el_phys = max(PHYS_EL_MIN, min(PHYS_EL_MAX, el_w))  # 0..180 clamp

    return az_phys, el_phys

def phys_to_servo(az_phys: float, el_phys: float):
    """
    PHYSICAL -> SERVO (deg). Axis-specific ratios and ranges.
    With 2:1 gearing: servo = physical / 2.
    So world el 0..90° -> servo el 0..45°.
    """
    az_servo = az_phys / GEAR_RATIO_AZ
    el_servo = el_phys / GEAR_RATIO_EL
    az_servo = max(0.0, min(SERVO_MAX_DEG_AZ, az_servo))
    el_servo = max(0.0, min(SERVO_MAX_DEG_EL, el_servo))
    return az_servo, el_servo

def _us_to_duty(pulse_us: float, freq_hz: float = PWM_FREQ_HZ) -> float:
    period_us = 1_000_000.0 / float(freq_hz)  # 20,000 us at 50 Hz
    return 100.0 * (pulse_us / period_us)

def _servo_deg_to_us(servo_deg: float, servo_max_deg: float) -> float:
    # Linear 0..servo_max_deg -> 1000..2000 us
    servo_deg = max(0.0, min(servo_max_deg, servo_deg))
    return PULSE_MIN_US + (servo_deg / servo_max_deg) * (PULSE_MAX_US - PULSE_MIN_US)

def servo_to_pwm_duty_axis(servo_deg: float, servo_max_deg: float) -> float:
    pulse = _servo_deg_to_us(servo_deg, servo_max_deg)
    return _us_to_duty(pulse)

# ===================== GPIO Setup =====================
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(SERVO_AZI_PIN, GPIO.OUT)
    GPIO.setup(SERVO_ELE_PIN, GPIO.OUT)
    pwm_azi = GPIO.PWM(SERVO_AZI_PIN, PWM_FREQ_HZ)
    pwm_ele = GPIO.PWM(SERVO_ELE_PIN, PWM_FREQ_HZ)
    pwm_azi.start(0)
    pwm_ele.start(0)
    debug("GPIO setup complete")
    return pwm_azi, pwm_ele

# ===================== MAVLink Setup =====================
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

# ===================== Servo Control =====================
def set_angle(world_az: float, world_el: float, pwm_azi, pwm_ele):
    """
    Compute servo targets from world angles and update axes independently.
    Uses a single shared threshold and a single shared settle time.
    """
    global prev_servo_az, prev_servo_el

    try:
        # World -> physical -> servo
        az_phys, el_phys = apply_calibration_world_to_physical(world_az, world_el)
        servo_az, servo_el = phys_to_servo(az_phys, el_phys)

        # Threshold per axis (servo-side degrees)
        update_az = (prev_servo_az is None) or (abs(servo_az - prev_servo_az) >= MIN_DEGREE_DELTA_SERVO)
        update_el = (prev_servo_el is None) or (abs(servo_el - prev_servo_el) >= MIN_DEGREE_DELTA_SERVO)

        if not update_az and not update_el:
            debug("Both axes below threshold — skipping PWM update")
            return

        # Duties only for axes that update
        duty_az = servo_to_pwm_duty_axis(servo_az, SERVO_MAX_DEG_AZ) if update_az else None
        duty_el = servo_to_pwm_duty_axis(servo_el, SERVO_MAX_DEG_EL) if update_el else None

        # Diagnostics
        debug(f"WORLD cmd  → Az {world_az:.2f}°, El {world_el:.2f}°")
        debug(f"PHYS  cmd  → Az {az_phys:.2f}°, El {el_phys:.2f}°  | pan 0..{PHYS_AZ_MAX:.0f}°")
        debug(f"SERVO tgt → Az {servo_az:.2f}° (max {SERVO_MAX_DEG_AZ:.0f}), El {servo_el:.2f}° (max {SERVO_MAX_DEG_EL:.0f})")

        if update_az:
            print(f"[SET_SERVO] AZ: servo_input={servo_az:.2f}°, phys_target={az_phys:.2f}°, duty={duty_az:.2f}%")
            pwm_azi.ChangeDutyCycle(duty_az)
        else:
            debug("AZ below threshold — no update")

        if update_el:
            print(f"[SET_SERVO] EL: servo_input={servo_el:.2f}°, phys_target={el_phys:.2f}°, duty={duty_el:.2f}%")
            pwm_ele.ChangeDutyCycle(duty_el)
        else:
            debug("EL below threshold — no update")

        # Shared settle time
        time.sleep(SETTLE_TIME_SEC)

        # Stop channels we started
        if update_az:
            pwm_azi.ChangeDutyCycle(0)
            prev_servo_az = servo_az
        if update_el:
            pwm_ele.ChangeDutyCycle(0)
            prev_servo_el = servo_el

        # Log computed targets (even if only one axis updated)
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

# ===================== GPS Thread =====================
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

# ===================== Tracking & Logging =====================
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

def tracking_loop(pwm_azi, pwm_ele):
    debug("Tracking loop started")
    while True:
        try:
            info = calculate_tracking_angles()
            if info:
                # Move using world angles (calibration & gearing handled inside set_angle)
                set_angle(info["adjusted_azimuth"], info["adjusted_elevation"], pwm_azi, pwm_ele)
            time.sleep(TRACKING_UPDATE_RATE)
        except KeyboardInterrupt:
            debug("Stopped by user")
            break
        except Exception as e:
            debug(f"Loop error: {e}", "ERROR")
            time.sleep(5)

# ===================== Cleanup =====================
def cleanup(pwm_azi, pwm_ele):
    debug("Cleaning up GPIO and logging")
    try:
        pwm_azi.ChangeDutyCycle(0); pwm_ele.ChangeDutyCycle(0)
        time.sleep(0.2)
        pwm_azi.stop()
        pwm_ele.stop()
        GPIO.cleanup()
        if LOG_TO_CSV and log_file:
            log_file.close()
    except Exception as e:
        debug(f"Cleanup error: {e}", "ERROR")

# ===================== Main =====================
def main():
    debug("Starting Antenna Tracker")
    pwm_azi, pwm_ele = setup_gpio()

    # Move to a known starting pose
    set_angle(INITIAL_WORLD_AZ, INITIAL_WORLD_EL, pwm_azi, pwm_ele)

    # MAVLink
    mav_drone = connect_mavlink()
    if mav_drone:
        threading.Thread(target=update_gps, args=(mav_drone, drone_gps, drone_gps_lock), daemon=True).start()

    # Tracking
    try:
        time.sleep(2)
        tracking_loop(pwm_azi, pwm_ele)
    finally:
        cleanup(pwm_azi, pwm_ele)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        debug("Shutdown by user")
