import argparse
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import math
import threading
from typing import Optional, Tuple, Callable

import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker

def setup_pigpio():
    pi = pigpio.pi()
    
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running (sudo pigpiod)")
    
    pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT) #Setting Azimuthal pin to pin 18
    pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT) #Setting Azimuthal pin to pin 17
    print(f"[INFO] pigpio ready (AZ={SERVO_AZ_PIN}, EL={SERVO_EL_PIN})")
    return pi

def log_open(prefix="Tracker"):
    global _log_writer, _log_file, _raw_writer, _raw_file # Defined for two types of log file writing
    if not LOG_TO_CSV and not LOG_RAW_GPS:          #Flags to check if Lo to CSv and Raw log GPS are enabled otherwise return
        return
    import csv, pathlib
    pathlib.Path("Tracker_Logs").mkdir(exist_ok=True)  #
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    if LOG_TO_CSV:
        fn = pathlib.Path(f"Tracker_Logs/{prefix}_{ts}.csv")
        _log_file = fn.open("w", newline="")
        _log_writer = csv.writer(_log_file)
        _log_writer.writerow([
            "Time",
            "BaseMode","BaseLocked",
            "WorldAz","WorldEl","CalAz","CalEl",
            "PhysAz","PhysEl","ServoAz","ServoEl","Az(us)","El(us)",
            "DroneLat","DroneLon","DroneAlt",
            "BaseLat","BaseLon","BaseAlt",
            "BaseLatSDm","BaseLonSDm","BaseFix","BaseSats"
        ])
        print(f"[INFO] CSV log → {fn}")
    if LOG_RAW_GPS:
        fnr = pathlib.Path(f"Tracker_Logs/{prefix}_RAW_{ts}.csv")
        _raw_file = fnr.open("w", newline="")
        _raw_writer = csv.writer(_raw_file)
        _raw_writer.writerow(["Time","Stream","Lat","Lon","Alt","eph(m)","epv(m)","fix_type","sats"])
        print(f"[INFO] RAW GPS log → {fnr}")

def main():
    ap = argparse.ArgumentParser(description="Unified antenna tracker with static/dynamic base modes + smooth parking")
    ap.add_argument("--mode", choices=["sim","ground"], default=MODE_DEFAULT,
                    help="Run mode: SITL 'sim' or hardware 'ground' (default: ground)")

    ap.add_argument("--base-mode", choices=["dynamic","static"], default=BASE_MODE_DEFAULT,
                    help="Base position mode for 'ground': dynamic (filtered live) or static (auto-lock)")

    ap.add_argument("--static-window-sec", type=float, default=10.0,
                    help="Seconds of stable base needed before locking (static mode)")

    ap.add_argument("--static-sd-thresh-m", type=float, default=0.9,
                    help="Stddev threshold in meters to consider base stable (static mode)")

    ap.add_argument("--dynamic-window-sec", type=float, default=6.0,
                    help="Window for rolling median in dynamic mode")

    ap.add_argument("--alpha", type=float, default=0.2,
                    help="Exponential smoothing factor for world az/el (0..1)")

    ap.add_argument("--print-period", type=float, default=PRINT_PERIOD_S,
                    help="Seconds between console prints")

    ap.add_argument("--update-period", type=float, default=UPDATE_PERIOD_S,
                    help="Main loop period seconds (servo update rate)")

    # Parking & overrides
    ap.add_argument("--park-home-az", type=float, default=0.0,
                    help="Home azimuth (deg) for parking (default 0)")
    ap.add_argument("--park-home-el", type=float, default=90.0,
                    help="Home elevation (deg) for parking (default 90)")
    ap.add_argument("--park-face-drone-start", action="store_true",
                    help="On startup, park facing current drone azimuth (EL=home)")
    ap.add_argument("--park-face-drone-exit", action="store_true",
                    help="On shutdown, park facing current drone azimuth (EL=home)")
    ap.add_argument("--park-duration", type=float, default=1.5,
                    help="Seconds for smooth parking at init/cleanup")
    ap.add_argument("--park-rate-hz", type=float, default=60.0,
                    help="Update rate for smooth parking")
    ap.add_argument("--az-gear-ratio", dest="az_gear_ratio", type=float, default=None,
                    help="Override AZ gear ratio (default 2.0)")
    ap.add_argument("--el-gear-ratio", dest="el_gear_ratio", type=float, default=None,
                    help="Override EL gear ratio (default 2.0)")
    ap.add_argument("--servo-min-us", dest="servo_min_us", type=float, default=None,
                    help="Override servo min pulse (µs), e.g., 900")
    ap.add_argument("--servo-max-us", dest="servo_max_us", type=float, default=None,
                    help="Override servo max pulse (µs), e.g., 1200")

    args = ap.parse_args()
    
    # Apply optional overrides
    global AZ_GEAR_RATIO, EL_GEAR_RATIO, PULSE_MIN_US, PULSE_MAX_US
    if args.az_gear_ratio is not None:
        AZ_GEAR_RATIO = float(args.az_gear_ratio)
    if args.el_gear_ratio is not None:
        EL_GEAR_RATIO = float(args.el_gear_ratio)
    if args.servo_min_us  is not None:
        PULSE_MIN_US = float(args.servo_min_us)
    if args.servo_max_us  is not None:
        PULSE_MAX_US = float(args.servo_max_us)

    print("=== geo_9 (sim/ground) with base dynamic/static, zero-ref + smoothing + smooth parking ===")
    print(f"[CFG] Mode: {args.mode} | BaseMode: {args.base_mode} | "
          f"Gear AZ {AZ_GEAR_RATIO}:1, EL {EL_GEAR_RATIO}:1 | "
          f"Servo 180° @ {PULSE_MIN_US}-{PULSE_MAX_US}µs")
    print(f"[CFG] update_period={args.update_period:.2f}s, print_period={args.print_period:.1f}s, alpha={args.alpha}")
    print(f"[CFG] parking: home=({args.park_home_az:.1f}°, {args.park_home_el:.1f}°) "
          f"face_drone(start={args.park_face_drone_start}, exit={args.park_face_drone_exit}) "
          f"duration={args.park_duration:.2f}s @ {args.park_rate_hz:.0f} Hz")

# Set up pigpio
    pi = setup_pigpio()
    log_open(prefix="Tracker")

# Prepare buffers and readers
    drone_buf = GpsBuffer("drone", maxlen=300)
    base_buf  = GpsBuffer("base",  maxlen=300)