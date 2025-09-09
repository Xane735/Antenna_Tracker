import time
import subprocess
from pymavlink import mavutil
import threading

PortBase = "/dev/ttyACM0"
PortDrone = "/dev/ttyUSB0"
BAUD = 57600

RETRIES = 5
WAIT_S  = 2.0
SLEEP_BETWEEN = 1.0

def _ok_coords(lat, lon):
    return (lat is not None and lon is not None and not (lat == 0 and lon == 0))

def gps_stream(port: str, baud_rate: int, name: str, success_event: threading.Event):
    master = None
    try:
        print(f"[INFO] [{name}] Opening {port} @ {baud_rate} …")
        master = mavutil.mavlink_connection(port, baud=baud_rate)

        try:
            master.wait_heartbeat(timeout=1.5)
            print(f"[INFO] [{name}] Heartbeat OK (sys={master.target_system}, comp={master.target_component})")
        except Exception:
            print(f"[WARN] [{name}] No heartbeat; continuing")

        t0 = time.time()
        while time.time() - t0 < WAIT_S:
            msg = master.recv_match(type=["GPS_RAW_INT","GLOBAL_POSITION_INT"], blocking=True, timeout=1.0)
            if not msg:
                continue
            tp = msg.get_type()
            if tp == "GPS_RAW_INT":
                lat, lon, alt = msg.lat, msg.lon, msg.alt
                if _ok_coords(lat, lon):
                    print(f"[OK ] [{name}] GPS_RAW_INT lat={lat/1e7:.7f} lon={lon/1e7:.7f} alt={alt/1000:.2f} m")
                    success_event.set()
                    return
            elif tp == "GLOBAL_POSITION_INT":
                lat, lon, alt = msg.lat, msg.lon, msg.alt
                if _ok_coords(lat, lon):
                    print(f"[OK ] [{name}] GLOBAL_POSITION_INT lat={lat/1e7:.7f} lon={lon/1e7:.7f} alt={alt/1000:.2f} m")
                    success_event.set()
                    return

        print(f"[ERR] [{name}] No valid GPS within {WAIT_S:.1f}s")
    except Exception as e:
        print(f"[ERR] [{name}] {e}")
    finally:
        try:
            if master:
                master.close()
        except Exception:
            pass
        print(f"[INFO] [{name}] Thread done")

def gps_check_and_run():
    for attempt in range(1, RETRIES+1):
        print(f"\n=== GPS Preflight Attempt {attempt}/{RETRIES} ===")

        base_ok  = threading.Event()
        drone_ok = threading.Event()

        t1 = threading.Thread(target=gps_stream, args=(PortBase, BAUD, "BASE",  base_ok))
        t2 = threading.Thread(target=gps_stream, args=(PortDrone, BAUD, "DRONE", drone_ok))

        t1.start(); t2.start()
        t1.join();  t2.join()

        if base_ok.is_set() and drone_ok.is_set():
            print("[CHK] Both GPS streams OK ✓  Launching tracker…")
            subprocess.run(["python3", "geo_13.py", "--mode", "ground", "--base-mode", "static"], check=True)
            return
        else:
            print("[WARN] One or both GPS streams failed; retrying…")
            time.sleep(SLEEP_BETWEEN)

    print("[ERR] GPS preflight failed after retries ✗")

if __name__ == "__main__":
    gps_check_and_run()
