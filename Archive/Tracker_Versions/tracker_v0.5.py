#!/usr/bin/env python3
# geo_4_pigpio.py
# Antenna Tracker (GPS-driven) — pigpio edition
# - PAN: full 360° physical with 2:1 gearing (servo 0..180°)
# - TILT: 2:1 gearing (world 0..90° -> servo 0..45°); safe clamp to 0..180° phys
# - Exact 1000–2000 µs pulses via pigpio
# - Independent per-axis updates with a shared settle time
# - MAVLink GPS input (GLOBAL_POSITION_INT), CSV logging

import threading
import time
import math
import csv
from datetime import datetime

import pigpio
from pymavlink import mavutil

# Your angle math (already tested)
import azi_elev_5 as tracker

# ===================== Configuration =====================
DEBUG = True
VERBOSE_GPS = False
VERBOSE_MOVEMENT = False
LOG_TO_CSV = True

GPS_TIMEOUT = 5
TRACKING_UPDATE_RATE = 0.5     # seconds between target recompute
SETTLE_TIME_SEC = 0.5          # shared settle time (seconds)

# --- Mechanics (axis-specific) ---
# Azimuth: 2:1 gear, 180° servo -> 360° physical (assumed continuous)
GEAR_RATIO_AZ = 2.0
SERVO_MAX_DEG_AZ = 180.0
PHYS_AZ_MIN = 0.0
PHYS_AZ_MAX = 360.0   # full circle

# Elevation: 2:1 gear; world elevation 0..90 -> servo 0..45 (safe to clamp phys to 180)
GEAR_RATIO_EL = 2.0
SERVO_MAX_DEG_EL = 180.0
PHYS_EL_MIN = 0.0
PHYS_EL_MAX = 180.0

# PWM endpoints (µs) — matches your servo checker
PULSE_MIN_US = 1000.0
PULSE_MAX_US = 2000.0

# Per-axis minimum-change filter (in SERVO-side degrees)
MIN_DEGREE_DELTA_SERVO = 4.5

# --- Calibration (world -> mechanism) ---
# World az 0° = True North. Adjust if your zero points differ.
AZIMUTH_ZERO_OFFSET_DEG = 0.0      # adds to WORLD az before mapping
ELEVATION_ZERO_OFFSET_DEG = 0.0    # adds to WORLD el before mapping
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
prev_servo_az = None
prev_servo_el = None

drone_gps = {"lat": None, "lon": None, "alt": None}
drone_gps_lock = threading.Lock()

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
    az_phys = az_w                                   # 0..360
    el_phys = max(PHYS_EL_MIN, min(PHYS_EL_MAX, el_w))  # clamp

    return az_phys, el_phys

def phys_to_servo(az_phys: float, el_phys: float):
    """
    PHYSICAL -> SERVO (deg). With 2:1 gearing: servo = physical / 2.
    So world el 0..90° -> servo el 0..45°.
    """
    az_servo = az_phys / GEAR_RATIO_AZ
    el_servo = el_phys / GEAR_RATIO_EL
    az_servo = max(0.0, min(SERVO_MAX_DEG_AZ, az_servo))
    el_servo = max(0.0, min(SERVO_MAX_DEG_EL, el_servo))
    return az_servo, el_servo

def servo_deg_to_us(servo_deg: float, servo_max_deg: float) -> float:
    """Linear map: 0..servo_max_deg  ->  1000..2000 µs."""
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
    Uses a single shared threshold and a single shared settle time.
    Generates exact 1000–2000 µs pulses via pigpio.
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

        # Compute pulses for axes that update
        pulse_az = servo_deg_to_us(servo_az, SERVO_MAX_DEG_AZ) if update_az else None
        pulse_el = servo_deg_to_us(servo_el, SERVO_MAX_DEG_EL) if update_el else None

        # Diagnostics
        debug(f"WORLD cmd  → Az {world_az:.2f}°, El {world_el:.2f}°")
        debug(f"PHYS  cmd  → Az {az_phys:.2f}°, El {el_phys:.2f}°  | pan 0..{PHYS_AZ_MAX:.0f}°")
        debug(f"SERVO tgt → Az {servo_az:.2f}° (max {SERVO_MAX_DEG_AZ:.0f}), El {servo_el:.2f}° (max {SERVO_MAX_DEG_EL:.0f})")

        if update_az:
            print(f"[SET_SERVO] AZ: servo_input={servo_az:.2f}°, phys_target={az_phys:.2f}°, pulse={pulse_az:.0f}us")
            pi.set_servo_pulsewidth(SERVO_AZI_PIN, pulse_az)
        else:
            debug("AZ below threshold — no update")

        if update_el:
            print(f"[SET_SERVO] EL: servo_input={servo_el:.2f}°, phys_target={el_phys:.2f}°, pulse={pulse_el:.0f}us")
            pi.set_servo_pulsewidth(SERVO_ELE_PIN, pulse_el)
        else:
            debug("EL below threshold — no update")

        # Shared settle time
        time.sleep(SETTLE_TIME_SEC)

        # Stop channels we started (to reduce jitter)
        if update_az:
            pi.set_servo_pulsewidth(SERVO_AZI_PIN, 0)
            prev_servo_az = servo_az
        if update_el:
            pi.set_servo_pulsewidth(SERVO_ELE_PIN, 0)
            prev_servo_el = servo_el

        # CSV log (even if only one axis updated)
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

# ===================== MAVLink Setup & GPS =====================
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
                    gps_dict["alt"] = msg.alt / 1000.0  # ASL (m)
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
                # Move using world angles (calibration & gearing handled inside set_angle)
                set_angle(info["adjusted_azimuth"], info["adjusted_elevation"], pi)
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

# ===================== Main =====================
def main():
    debug("Starting Antenna Tracker (pigpio)")
    pi = setup_pigpio()

    # Move to a known starting pose
    set_angle(INITIAL_WORLD_AZ, INITIAL_WORLD_EL, pi)

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
