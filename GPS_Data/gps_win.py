#!/usr/bin/env python3
"""
gps_check.py — Verify GPS stream from a Pixhawk over MAVLink (Windows-friendly)

Usage examples:
  # Serial (most common)
  python gps_check.py --device COM5 --baud 57600

  # UDP (e.g., SITL or telemetry forwarded to localhost)
  python gps_check.py --device udp:127.0.0.1:14550
"""

import argparse
import sys
import time
from datetime import datetime

from pymavlink import mavutil

FIX_TYPE_MAP = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED"
}

def human_fix(fix_type: int) -> str:
    return FIX_TYPE_MAP.get(fix_type, f"UNKNOWN({fix_type})")

def deg1e7_to_deg(x: int) -> float:
    # MAVLink lat/lon are in 1e-7 degrees
    if x is None:
        return float("nan")
    return x / 1e7

def mm_to_m(x: int) -> float:
    # alt/rel alt in millimeters
    if x is None:
        return float("nan")
    return x / 1000.0

def cm_to_m(x: int) -> float:
    # some GPS fields use centimeters (eph/epv in GPS_RAW_INT)
    if x is None:
        return float("nan")
    return x / 100.0

def try_set_message_interval(m, msg_id: int, rate_hz: float):
    """
    Ask the autopilot to stream a message at a given frequency.
    Falls back silently if the FCU doesn't support SET_MESSAGE_INTERVAL.
    """
    try:
        interval_us = int(1_000_000 / rate_hz) if rate_hz > 0 else -1
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0, 0, 0, 0, 0
        )
    except Exception:
        pass  # not fatal—many stacks stream these by default

def request_legacy_streams(m):
    """
    Legacy request (older stacks). Not strictly required but can help.
    """
    try:
        # Request position and extra status at ~5 Hz
        m.mav.request_data_stream_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            5,
            1
        )
        m.mav.request_data_stream_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            5,
            1
        )
    except Exception:
        pass

def connect(device: str, baud: int) -> mavutil.mavlink_connection:
    if device.lower().startswith("udp:") or device.lower().startswith("tcp:"):
        m = mavutil.mavlink_connection(device, autoreconnect=True)
    else:
        # Serial on Windows (e.g., COM5)
        m = mavutil.mavlink_connection(device, baud=baud, autoreconnect=True)
    return m

def main():
    parser = argparse.ArgumentParser(description="Check GPS stream from Pixhawk via MAVLink.")
    parser.add_argument("--device", "-d", required=True,
                        help="Connection string (e.g., COM5 or udp:127.0.0.1:14550)")
    parser.add_argument("--baud", "-b", type=int, default=57600,
                        help="Baud rate for serial (default: 57600)")
    parser.add_argument("--timeout", "-t", type=float, default=15.0,
                        help="Seconds to wait for first GPS message before warning (default: 15)")
    parser.add_argument("--rate", "-r", type=float, default=5.0,
                        help="Desired stream rate in Hz for GPS messages (best-effort)")
    args = parser.parse_args()

    print(f"[{datetime.now().strftime('%H:%M:%S')}] Connecting to {args.device} (baud={args.baud}) ...")
    m = connect(args.device, args.baud)

    print("Waiting for HEARTBEAT...")
    hb = m.wait_heartbeat(timeout=10)
    if not hb:
        print("No HEARTBEAT received. Check device/port/baud and cable. Exiting.")
        sys.exit(2)

    print(f"HEARTBEAT OK: system={m.target_system} component={m.target_component}")

    # Best-effort: ask FCU to stream GPS-related messages
    try_set_message_interval(m, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, args.rate)
    try_set_message_interval(m, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, args.rate)
    request_legacy_streams(m)

    first_gps_time = None
    last_raw = None
    last_global = None

    start = time.time()
    print("\nListening for GPS… Press Ctrl+C to stop.\n")
    print("TIME        | FIX        | SATS | LAT        , LON         | ALT(m) | REL_ALT(m) | HDOP(m) | VDOP(m) | VEL(m/s) | HEAD(deg)")
    print("-" * 110)

    try:
        while True:
            msg = m.recv_match(blocking=True, timeout=2)
            now = datetime.now().strftime("%H:%M:%S")
            if msg is None:
                # If no messages at all after timeout, print a heartbeat of the loop
                elapsed = time.time() - start
                if first_gps_time is None and elapsed > args.timeout:
                    print(f"[{now}] WARNING: No GPS messages received yet. "
                          f"Check GPS lock, wiring, and SERIALx_BAUD/SERIALx_PROTOCOL.")
                    # extend the window once
                    args.timeout += 10
                continue

            msg_type = msg.get_type()

            if msg_type == "GPS_RAW_INT":
                last_raw = msg
                if first_gps_time is None:
                    first_gps_time = time.time()

            elif msg_type == "GLOBAL_POSITION_INT":
                last_global = msg

            # Print a merged row when we have at least one of them
            if last_raw or last_global:
                # Prefer lat/lon/alt from GLOBAL_POSITION_INT if available, else GPS_RAW_INT
                if last_global:
                    lat = deg1e7_to_deg(getattr(last_global, "lat", None))
                    lon = deg1e7_to_deg(getattr(last_global, "lon", None))
                    alt_m = mm_to_m(getattr(last_global, "alt", None))
                    rel_alt_m = mm_to_m(getattr(last_global, "relative_alt", None))
                    vel_ms = getattr(last_global, "vx", 0)
                    vy = getattr(last_global, "vy", 0)
                    vz = getattr(last_global, "vz", 0)
                    # Total ground speed magnitude if components present (cm/s)
                    try:
                        spd_ms = ( (vel_ms**2 + vy**2 + vz**2) ** 0.5 ) / 100.0
                    except Exception:
                        spd_ms = float("nan")
                    hdg_cdeg = getattr(last_global, "hdg", None)  # 0..35999 cdeg
                    hdg_deg = (hdg_cdeg / 100.0) if hdg_cdeg is not None and hdg_cdeg != 65535 else float("nan")
                else:
                    lat = deg1e7_to_deg(getattr(last_raw, "lat", None))
                    lon = deg1e7_to_deg(getattr(last_raw, "lon", None))
                    alt_m = mm_to_m(getattr(last_raw, "alt", None))
                    rel_alt_m = float("nan")
                    spd_ms = float("nan")
                    hdg_deg = float("nan")

                if last_raw:
                    fix = human_fix(getattr(last_raw, "fix_type", 0))
                    sats = getattr(last_raw, "satellites_visible", 0)
                    hdop = cm_to_m(getattr(last_raw, "eph", None))
                    vdop = cm_to_m(getattr(last_raw, "epv", None))
                else:
                    # If RAW not yet received, use placeholders
                    fix = "UNKNOWN"
                    sats = 0
                    hdop = float("nan")
                    vdop = float("nan")

                print(f"{now} | {fix:10s} | {sats:4d} | "
                      f"{lat:10.6f}, {lon:11.6f} | "
                      f"{alt_m:6.1f} | {rel_alt_m:10.1f} | "
                      f"{hdop:6.2f} | {vdop:6.2f} | "
                      f"{spd_ms:7.2f} | {hdg_deg:8.2f}")

    except KeyboardInterrupt:
        print("\nExiting on user request.")
    except Exception as e:
        print(f"\nERROR: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
