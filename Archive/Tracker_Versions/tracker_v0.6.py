#!/usr/bin/env python3
"""
Antenna-tracker control:
• Reads drone GPS via MAVLink (GLOBAL_POSITION_INT on UDP :14551)
• Calculates ideal az/el with azi_elev_5.py
• Applies calibration, clamps to physical limits, maps through 2 : 1 gearing
• Drives two hobby servos via pigpio PWM

Now configured for a **180 ° servo** (900-2100 µs) ⇒ **360 ° physical** travel.
"""

import time
from datetime import datetime
import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker

# ===================== Configuration =====================

# Base station reference (only used if you’re NOT feeding a live base-GPS)
base_gps = {
    "lat": 13.0276816,
    "lon": 77.5630373,
    "alt": 931.13,          # metres ASL
}

# Gear spokes → ratio (physical_out : servo)
SPOKES_SMALL = 12
SPOKES_BIG   = 24
GEAR_RATIO   = SPOKES_BIG / SPOKES_SMALL      # == 2.0

# Physical limits (mechanism side)
AZ_PHYS_MIN = 0.0
AZ_PHYS_MAX = 350.0        # keep a small safety margin
EL_PHYS_MIN = 0.0
EL_PHYS_MAX = 180.0

# Servo (electrical) limits & mapping ― **180 ° servo**
PULSE_MIN_US    = 900.0    # tune these two if endpoints need trim
PULSE_MAX_US    = 2100.0
SERVO_RANGE_DEG = 180.0    # 0–180 ° servo travel maps over 900–2100 µs

# Calibration (apply to world angles before gearing)
AZIMUTH_ZERO_OFFSET_DEG   = 0.0
ELEVATION_ZERO_OFFSET_DEG = 0.0
AZIMUTH_INVERT   = True   # True flips CW/CCW
ELEVATION_INVERT = False   # True flips up/down

# pigpio GPIO pins (BCM numbering)
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 13

# Update cadence
UPDATE_PERIOD_S      = 0.20        # 5 Hz
MAV_MSG_INTERVAL_US  = 200_000     # request GLOBAL_POSITION_INT at 5 Hz

# Logging
PRINT_EVERY = 1                    # console print each cycle
LOG_TO_CSV  = True                 # write a CSV log

# =========================================================
# ------------------- Helper functions --------------------
# =========================================================

def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

def apply_calibration(az_world: float, el_world: float):
    """Zero-offset & optional inversion in WORLD frame (before gearing)."""
    az = norm360(az_world + AZIMUTH_ZERO_OFFSET_DEG)
    el = el_world + ELEVATION_ZERO_OFFSET_DEG
    if AZIMUTH_INVERT:
        az = norm360(360.0 - az)
    if ELEVATION_INVERT:
        el = -el
    return az, el

def world_to_physical(az_world_cal: float, el_world_cal: float):
    """Apply hard stops of the mechanism."""
    az_phys = max(AZ_PHYS_MIN, min(AZ_PHYS_MAX, az_world_cal))
    el_phys = max(EL_PHYS_MIN, min(EL_PHYS_MAX, el_world_cal))
    return az_phys, el_phys

def physical_to_servo_deg(az_phys: float, el_phys: float):
    """
    Convert physical motion → servo degrees via the 2 : 1 gearing,
    then clamp to the servo’s **180 °** usable range.
    """
    az_raw = az_phys / GEAR_RATIO
    el_raw = el_phys / GEAR_RATIO

    az_servo = max(0.0, min(SERVO_RANGE_DEG, az_raw))
    el_servo = max(0.0, min(SERVO_RANGE_DEG, el_raw))

    # Warn if we had to clip
    if az_servo != az_raw:
        print(f"[WARN] Az demand {az_phys:.1f}° phys -> {az_raw:.1f}° servo exceeds "
              f"{SERVO_RANGE_DEG}°. Clipped to {az_servo:.1f}°.")
    if el_servo != el_raw:
        print(f"[WARN] El demand {el_phys:.1f}° phys -> {el_raw:.1f}° servo exceeds "
              f"{SERVO_RANGE_DEG}°. Clipped to {el_servo:.1f}°.")

    return az_servo, el_servo

def servo_deg_to_us(servo_deg: float) -> float:
    """Linear map 0–180 ° → 900–2100 µs."""
    servo_deg = max(0.0, min(SERVO_RANGE_DEG, servo_deg))
    return PULSE_MIN_US + (servo_deg / SERVO_RANGE_DEG) * (PULSE_MAX_US - PULSE_MIN_US)

# =========================================================
# ---------------------- pigpio init ----------------------
# =========================================================

def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running (start with sudo pigpiod)")
    pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
    print(f"[INFO] pigpio ready (AZ pin {SERVO_AZ_PIN}, EL pin {SERVO_EL_PIN})")
    return pi

# =========================================================
# ---------------------- MAVLink I/O ----------------------
# =========================================================

def connect_mavlink():
    mav = mavutil.mavlink_connection("udp:0.0.0.0:14551")
    print("Waiting for heartbeat...")
    mav.wait_heartbeat()
    print(f"Connected (sys={mav.target_system}, comp={mav.target_component})")

    # Ask sender to stream GLOBAL_POSITION_INT at ~5 Hz
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        MAV_MSG_INTERVAL_US, 0, 0, 0, 0, 0
    )
    return mav

# =========================================================
# ----------------------- CSV log -------------------------
# =========================================================

_log_writer = None
_log_file   = None

def log_open():
    global _log_writer, _log_file
    if not LOG_TO_CSV:
        return
    import csv
    from pathlib import Path
    Path("Tracker_Logs").mkdir(exist_ok=True)
    fn = Path(f"Tracker_Logs/Tracker_Log{datetime.now().strftime('%Y%m%d-%H%M%S')}.csv")
    _log_file = fn.open("w", newline="")
    _log_writer = csv.writer(_log_file)
    _log_writer.writerow([
        "Time",
        "WorldAz", "WorldEl",
        "CalAz", "CalEl",
        "PhysAz", "PhysEl",
        "ServoAz_deg", "ServoEl_deg",
        "PulseAz_us", "PulseEl_us",
    ])
    print(f"[INFO] CSV log → {fn}")

def log_row(*row):
    if LOG_TO_CSV and _log_writer:
        _log_writer.writerow(row)
        _log_file.flush()

def log_close():
    if LOG_TO_CSV and _log_file:
        _log_file.close()

# =========================================================
# --------------------- Main  loop ------------------------
# =========================================================

def main():
    print("=== Antenna Tracker (pigpio, 2:1 gear, 360° capable) ===")
    print(f"[CFG] Base reference lat={base_gps['lat']}, lon={base_gps['lon']}, alt={base_gps['alt']} m")
    print(f"[CFG] Servo range {SERVO_RANGE_DEG}°, pulses {PULSE_MIN_US:.0f}–{PULSE_MAX_US:.0f} µs")
    print(f"[CFG] Physical limits AZ 0–{AZ_PHYS_MAX}°, EL 0–{EL_PHYS_MAX}°")

    pi  = setup_pigpio()
    log_open()
    mav = connect_mavlink()

    # Initialise servos: world 0/20 °
    init_cal_az, init_cal_el = apply_calibration(0.0, 20.0)
    init_phys_az, init_phys_el = world_to_physical(init_cal_az, init_cal_el)
    init_s_az, init_s_el = physical_to_servo_deg(init_phys_az, init_phys_el)
    pi.set_servo_pulsewidth(SERVO_AZ_PIN, servo_deg_to_us(init_s_az))
    pi.set_servo_pulsewidth(SERVO_EL_PIN, servo_deg_to_us(init_s_el))
    time.sleep(0.3)

    cycle = 0
    try:
        while True:
            msg = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.5)
            if not msg:
                print("[WARN] No GPS update")
                continue

            drone_lat = msg.lat / 1e7
            drone_lon = msg.lon / 1e7
            drone_alt = msg.alt / 1000.0

            # 1. Get ideal az/el
            info = tracker.get_tracking_info(
                base_gps["lat"], base_gps["lon"], base_gps["alt"],
                drone_lat, drone_lon, drone_alt
            )
            if not info:
                continue

            world_az = info.get("adjusted_azimuth", info["azimuth"])
            world_el = info.get("adjusted_elevation", info["elevation"])

            # 2. Calibration → physical clamp → gearing
            cal_az,  cal_el  = apply_calibration(world_az, world_el)
            phys_az, phys_el = world_to_physical(cal_az, cal_el)
            s_az, s_el       = physical_to_servo_deg(phys_az, phys_el)

            # 3. Convert to pulse-width & drive servos
            pulse_az = servo_deg_to_us(s_az)
            pulse_el = servo_deg_to_us(s_el)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, pulse_az)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, pulse_el)

            # 4. Console & CSV
            if cycle % PRINT_EVERY == 0:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                      f"WORLD Az/El {world_az:6.2f}/{world_el:5.2f}°  "
                      f"CAL {cal_az:6.2f}/{cal_el:5.2f}°  "
                      f"PHYS {phys_az:6.2f}/{phys_el:5.2f}°  "
                      f"SERVO {s_az:6.2f}/{s_el:5.2f}°  "
                      f"µs {pulse_az:5.0f}/{pulse_el:5.0f}")
            log_row(
                datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                round(world_az,3), round(world_el,3),
                round(cal_az,3),   round(cal_el,3),
                round(phys_az,3),  round(phys_el,3),
                round(s_az,3),     round(s_el,3),
                round(pulse_az,1), round(pulse_el,1),
            )

            cycle += 1
            time.sleep(UPDATE_PERIOD_S)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")

    finally:
        try:
            # Park & release PWM
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, servo_deg_to_us(0.0))
            pi.set_servo_pulsewidth(SERVO_EL_PIN, servo_deg_to_us(0.0))
            time.sleep(0.2)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, 0)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, 0)
            pi.stop()
        except Exception as e:
            print(f"[WARN] pigpio cleanup: {e}")
        log_close()

if __name__ == "__main__":
    main()
