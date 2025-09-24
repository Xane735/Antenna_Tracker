#!/usr/bin/env python3
# gps_preflight.py — try both GPS ports up to 5 times, then launch geo_13.py

import time
from pymavlink import mavutil

DRONE_PORT = "/dev/ttyUSB0"
BASE_PORT  = "/dev/ttyACM0"
BAUD       = 57600

RETRIES    = 5           # total attempts per port
WAIT_S     = 2.0         # seconds to wait for a valid message each attempt
SLEEP_BETWEEN = 1.0      # seconds between attempts

def _ok_coords(lat, lon):
    return (lat is not None and lon is not None and not (lat == 0 and lon == 0))

def check_once(port: str, baud: int, name: str, wait_s: float) -> bool:
    master = None
    try:
        print(f"[CHK] {name}: opening {port} @ {baud}…", flush=True)
        master = mavutil.mavlink_connection(port, baud=baud)
        try:
            master.wait_heartbeat(timeout=1.5)
            print(f"[CHK] {name}: heartbeat OK (sys={master.target_system}, comp={master.target_component})")
        except Exception:
            print(f"[WARN] {name}: no heartbeat; continuing (may still receive GPS).")

        t0 = time.time()
        while time.time() - t0 < wait_s:
            msg = master.recv_match(type=["GPS_RAW_INT","GLOBAL_POSITION_INT"], blocking=True, timeout=1.0)
            if not msg:
                continue
            tp = msg.get_type()
            if tp == "GPS_RAW_INT":
                lat, lon, alt = msg.lat, msg.lon, msg.alt
                if _ok_coords(lat, lon):
                    print(f"[OK ] {name}: GPS_RAW_INT lat={lat/1e7:.7f}, lon={lon/1e7:.7f}, alt={alt/1000:.2f} m")
                    return True
            elif tp == "GLOBAL_POSITION_INT":
                lat, lon, alt = msg.lat, msg.lon, msg.alt
                if _ok_coords(lat, lon):
                    print(f"[OK ] {name}: GLOBAL_POSITION_INT lat={lat/1e7:.7f}, lon={lon/1e7:.7f}, alt={alt/1000:.2f} m")
                    return True
        print(f"[ERR] {name}: no valid GPS within {wait_s:.1f}s")
        return False
    except Exception as e:
        print(f"[ERR] {name}: {e}")
        return False
    finally:
        try:
            if master:
                master.close()
        except Exception:
            pass

def check_with_retries(port: str, baud: int, name: str) -> bool:
    for attempt in range(1, RETRIES+1):
        print(f"\n=== {name} Attempt {attempt}/{RETRIES} ===")
        if check_once(port, baud, name, WAIT_S):
            return True
        if attempt < RETRIES:
            print(f"[WARN] {name}: retrying in {SLEEP_BETWEEN:.1f}s…")
            time.sleep(SLEEP_BETWEEN)
    return False

def main():
    ok_base  = check_with_retries(BASE_PORT,  BAUD, "BASE")
    ok_drone = check_with_retries(DRONE_PORT, BAUD, "DRONE")

    if not (ok_base and ok_drone):
        print("[ERR] GPS preflight failed ✗")
        raise SystemExit(1)

    print("[CHK] Both GPS streams OK Launching tracker…")

if __name__ == "__main__":
    main()
