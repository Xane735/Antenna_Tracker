#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tracker_mpl_180deg.py

Mission Planner (MAVLink) → antenna tracker (pigpio) with 0–180° physical capability,
2:1 (physical:servo) gearing, and 1000..2000 µs mapping.

What it does
------------
- Subscribes to MAVLink GLOBAL_POSITION_INT (Mission Planner / ArduPilot / SITL compatible).
- Computes WORLD az/el from Base → Drone (simple, robust math).
- Applies calibration & optional axis inversion.
- Enforces 0..180° PHYSICAL window on both axes (WARN + clamp).
- Converts PHYSICAL → SERVO via 2:1 gear, maps to 1000..2000 µs, thresholds, and commands servos.

Requirements
------------
sudo apt-get install pigpio
sudo pigpiod
pip install pymavlink

Usage
-----
python3 tracker_mpl_180deg.py --conn udp:127.0.0.1:14550 --debug

Notes
-----
- Altitude source: GLOBAL_POSITION_INT.alt (mm AMSL). Treated as ASL here, per your workflow.
- Keep your mission such that the drone stays in the 0..180° physical window around the mast.
"""

import math
import time
import sys
import argparse
from typing import Tuple, Optional

import pigpio
from pymavlink import mavutil

# ========================= USER CONFIG =========================

BASE_GPS = {
    "lat": 13.0272176,
    "lon": 77.5630984,
    "alt": 931.13,  # meters ASL (treated as AMSL from GLOBAL_POSITION_INT)
}

# GPIO pins (BCM numbering)
SERVO_AZI_PIN = 18
SERVO_ELE_PIN = 13

# World → physical calibration
AZIMUTH_ZERO_OFFSET_DEG   = 0.0
ELEVATION_ZERO_OFFSET_DEG = 0.0
AZIMUTH_INVERT   = False     # You reported True gives the correct az direction
ELEVATION_INVERT = False

# Physical capability enforced by this script
PHYS_AZ_MIN = 0.0
PHYS_AZ_MAX = 180.0
PHYS_EL_MIN = 0.0
PHYS_EL_MAX = 90.0

# Gear: physical_out : servo
GEAR_RATIO_AZ = 2.0
GEAR_RATIO_EL = 2.0

# Servo characteristics per your tests
SERVO_US_MIN     = 1000.0   # µs at 0° servo
SERVO_US_MAX     = 2000.0   # µs at 180° servo
SERVO_RANGE_DEG  = 180.0    # servo degrees covered by the pulse range

# Update behavior
MIN_DEGREE_DELTA_SERVO = 3.0    # per-axis threshold in SERVO degrees (≈6° physical with 2:1)
SETTLE_TIME_S          = 0.15   # shared settle time after any axis update

# ===============================================================

def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

def calculate_bearing(lat1, lon1, lat2, lon2) -> float:
    """Initial bearing (deg, 0..360; 0=N)."""
    lat1_rad, lat2_rad = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0

def horiz_distance_m(lat1, lon1, lat2, lon2) -> float:
    """Equirectangular approximation; accurate enough for short ranges."""
    R = 6371000.0  # m
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
    y = math.radians(lat2 - lat1)
    return math.hypot(x, y) * R

def az_el_from_base_to_target(base: dict, tlat: float, tlon: float, talt_m: float) -> Tuple[float, float]:
    """WORLD az/el (deg) Base → Target."""
    blat, blon, balt = base["lat"], base["lon"], base["alt"]
    az = calculate_bearing(blat, blon, tlat, tlon)
    hd = horiz_distance_m(blat, blon, tlat, tlon)
    dz = (talt_m - balt)
    el = math.degrees(math.atan2(dz, hd)) if hd > 1e-6 else (90.0 if dz > 0 else 0.0)
    return az, el

class ServoTracker:
    """WORLD → PHYS (0..180 enforced) → SERVO deg → µs → pigpio."""

    def __init__(self, pi: pigpio.pi, pin_az: int, pin_el: int, debug: bool = True):
        self.pi = pi
        self.pin_az = pin_az
        self.pin_el = pin_el
        self.debug = debug

        self.prev_servo_az_deg: Optional[float] = None
        self.prev_servo_el_deg: Optional[float] = None

        self.pi.set_mode(self.pin_az, pigpio.OUTPUT)
        self.pi.set_mode(self.pin_el, pigpio.OUTPUT)
        print("[INFO] pigpio setup complete")

    # ----- Mapping helpers -----

    def _dbg(self, msg: str):
        if self.debug:
            print(msg)

    def servo_deg_to_us(self, servo_deg: float) -> float:
        """0..180° servo → 1000..2000 µs (linear)."""
        servo_deg = max(0.0, min(SERVO_RANGE_DEG, servo_deg))
        return SERVO_US_MIN + (servo_deg / SERVO_RANGE_DEG) * (SERVO_US_MAX - SERVO_US_MIN)

    def phys_to_servo_deg(self, az_phys: float, el_phys: float) -> Tuple[float, float]:
        """Physical → Servo deg via gear ratio (physical:servo = 2:1)."""
        az_servo = az_phys / GEAR_RATIO_AZ
        el_servo = el_phys / GEAR_RATIO_EL
        # Bound to servo’s intrinsic 0..180°
        az_servo = max(0.0, min(SERVO_RANGE_DEG, az_servo))
        el_servo = max(0.0, min(SERVO_RANGE_DEG, el_servo))
        return az_servo, el_servo

    def world_to_physical(self, az_world: float, el_world: float) -> Tuple[float, float, bool]:
        """WORLD → calibrated WORLD → PHYSICAL; enforce 0..180°; return (az_phys, el_phys, warned)."""
        az_w = norm360(az_world + AZIMUTH_ZERO_OFFSET_DEG)
        el_w = el_world + ELEVATION_ZERO_OFFSET_DEG

        if AZIMUTH_INVERT:
            az_w = norm360(360.0 - az_w)
        if ELEVATION_INVERT:
            el_w = -el_w

        az_phys = az_w
        el_phys = el_w

        warned = False
        if az_phys < PHYS_AZ_MIN or az_phys > PHYS_AZ_MAX:
            print(f"[WARN] AZ {az_phys:.2f}° outside 0..180° physical window; clamping.")
            az_phys = max(PHYS_AZ_MIN, min(PHYS_AZ_MAX, az_phys))
            warned = True
        if el_phys < PHYS_EL_MIN or el_phys > PHYS_EL_MAX:
            print(f"[WARN] EL {el_phys:.2f}° outside 0..180° physical window; clamping.")
            el_phys = max(PHYS_EL_MIN, min(PHYS_EL_MAX, el_phys))
            warned = True
        return az_phys, el_phys, warned

    # ----- Command path -----

    def set_world_angles(self, az_world: float, el_world: float):
        az_phys, el_phys, warned = self.world_to_physical(az_world, el_world)
        az_servo_deg, el_servo_deg = self.phys_to_servo_deg(az_phys, el_phys)
        self._apply_servo_commands(az_servo_deg, el_servo_deg, az_phys, el_phys, warned)

    def _apply_servo_commands(self, az_servo_deg: float, el_servo_deg: float,
                              az_phys: float, el_phys: float, warned: bool):
        az_us = self.servo_deg_to_us(az_servo_deg)
        el_us = self.servo_deg_to_us(el_servo_deg)

        upd_az = (self.prev_servo_az_deg is None) or (abs(az_servo_deg - self.prev_servo_az_deg) >= MIN_DEGREE_DELTA_SERVO)
        upd_el = (self.prev_servo_el_deg is None) or (abs(el_servo_deg - self.prev_servo_el_deg) >= MIN_DEGREE_DELTA_SERVO)

        if self.debug:
            info = (f"[WORLD→PHYS] AZ_w→p {az_phys:.2f}°, EL_w→p {el_phys:.2f}° | "
                    f"SERVO AZ {az_servo_deg:.2f}° → {az_us:.0f} µs, "
                    f"EL {el_servo_deg:.2f}° → {el_us:.0f} µs "
                    f"| upd_az={int(upd_az)} upd_el={int(upd_el)}")
            print(info)
            if warned:
                print("             ^-- clamp applied")

        moved = False
        if upd_az:
            self.pi.set_servo_pulsewidth(self.pin_az, az_us)
            self.prev_servo_az_deg = az_servo_deg
            moved = True
        if upd_el:
            self.pi.set_servo_pulsewidth(self.pin_el, el_us)
            self.prev_servo_el_deg = el_servo_deg
            moved = True

        if moved:
            time.sleep(SETTLE_TIME_S)

    def center(self):
        """Center to physical (90°,90°) == servo (45°,45°)."""
        az_servo_deg, el_servo_deg = self.phys_to_servo_deg(90.0, 90.0)
        self.pi.set_servo_pulsewidth(self.pin_az, self.servo_deg_to_us(az_servo_deg))
        self.pi.set_servo_pulsewidth(self.pin_el, self.servo_deg_to_us(el_servo_deg))
        self.prev_servo_az_deg = az_servo_deg
        self.prev_servo_el_deg = el_servo_deg

    def stop(self):
        try:
            self.pi.set_servo_pulsewidth(self.pin_az, 0)
            self.pi.set_servo_pulsewidth(self.pin_el, 0)
            self.pi.stop()
            print("[INFO] pigpio cleaned up")
        except Exception as e:
            print(f"[ERROR] stop(): {e}")


def main():
    ap = argparse.ArgumentParser(description="Mission Planner → pigpio tracker (0..180° physical, 2:1 gear).")
    ap.add_argument("--conn", default="udp:127.0.0.1:14550", help="MAVLink connection string (Mission Planner/SITL)")
    ap.add_argument("--debug", action="store_true", help="Verbose prints")
    args = ap.parse_args()

    pi = pigpio.pi()
    if not pi.connected:
        print("[ERROR] pigpio daemon not running. Start with: sudo pigpiod")
        sys.exit(1)

    tracker = ServoTracker(pi, SERVO_AZI_PIN, SERVO_ELE_PIN, debug=args.debug)

    print(f"[INFO] Connecting MAVLink: {args.conn}")
    mlink = mavutil.mavlink_connection(args.conn, autoreconnect=True)
    print("[INFO] Waiting for heartbeat…")
    mlink.wait_heartbeat()
    sysid, compid = mlink.target_system, mlink.target_component
    print(f"[INFO] Connected to system (system ID: {sysid}, component ID: {compid})")

    # Optional: center on start
    tracker.center()
    time.sleep(0.5)

    try:
        last_print = 0.0
        while True:
            # Request GLOBAL_POSITION_INT if stream not active (optional)
            # mlink.mav.request_data_stream_send(sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1)

            msg = mlink.recv_match(type=["GLOBAL_POSITION_INT"], blocking=True, timeout=2)
            if msg is None:
                continue

            # GLOBAL_POSITION_INT fields: lat/lon (1e7), alt (mm AMSL), relative_alt (mm)
            tlat = msg.lat / 1e7
            tlon = msg.lon / 1e7
            talt_m = msg.alt / 1000.0  # mm → m (AMSL per ArduPilot). Treated as ASL here.

            az_w, el_w = az_el_from_base_to_target(BASE_GPS, tlat, tlon, talt_m)
            tracker.set_world_angles(az_w, el_w)

            # Light status print every ~0.5s even if threshold blocked motion
            now = time.time()
            if args.debug and now - last_print > 0.5:
                print(f"[WORLD] az={az_w:.2f}°, el={el_w:.2f}° | base=({BASE_GPS['lat']:.7f},{BASE_GPS['lon']:.7f},{BASE_GPS['alt']:.1f}) "
                      f"target=({tlat:.7f},{tlon:.7f},{talt_m:.1f})")
                last_print = now

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
    finally:
        tracker.stop()


if __name__ == "__main__":
    main()
