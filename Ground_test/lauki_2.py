# Tested, has offset error
#!/usr/bin/env python3

import time
from datetime import datetime
import threading
from typing import Optional, Dict, Tuple

import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker

# =========================================================
# -------------------- Configuration ----------------------
# =========================================================

# MAVLink endpoints
DRONE_ENDPOINT = "/dev/ttyACM0"
DRONE_BAUD     = 115200        # set 57600 if needed

BASE_ENDPOINT  = "/dev/ttyUSB0"
BASE_BAUD      = 57600

MAV_MSG_INTERVAL_US_GPS    = 200_000   # 5 Hz
MAV_MSG_INTERVAL_US_GLOBAL = 200_000

# Gear ratio 2 : 1 (physical : servo)
SPOKES_SMALL = 12
SPOKES_BIG   = 24
GEAR_RATIO   = SPOKES_BIG / SPOKES_SMALL     # 2.0

# Physical limits (mechanism)
AZ_PHYS_MIN = 0.0
AZ_PHYS_MAX = 350.0
EL_PHYS_MIN = 0.0
EL_PHYS_MAX = 180.0

# Servo electrical limits – **180° servo**
PULSE_MIN_US    = 900.0
PULSE_MAX_US    = 2100.0
SERVO_RANGE_DEG = 180.0

# Calibration (applies only when mapping world→physical)
AZIMUTH_ZERO_OFFSET_DEG   = 30.0
ELEVATION_ZERO_OFFSET_DEG = 0.0
AZIMUTH_INVERT   = True
ELEVATION_INVERT = True

# GPIO pins (BCM)
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 13

# Timings
UPDATE_PERIOD_S = 0.20      # 5 Hz servo update
PRINT_EVERY     = 1         # console print every cycle
LOG_TO_CSV      = True

# Optional world-frame startup pose (no longer used for initial park-to-zero)
STARTUP_AZ_WORLD = 0.0
STARTUP_EL_WORLD = 0.0

# =========================================================
# -------------------- Shared State -----------------------
# =========================================================

drone_gps = {"lat": None, "lon": None, "alt": None}
base_gps  = {"lat": None, "lon": None, "alt": None}
_drone_lock = threading.Lock()
_base_lock  = threading.Lock()
stop_event  = threading.Event()

# =========================================================
# ----------------- Helper functions ----------------------
# =========================================================

def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

def apply_calibration(az: float, el: float) -> Tuple[float, float]:
    az = norm360(az + AZIMUTH_ZERO_OFFSET_DEG)
    el += ELEVATION_ZERO_OFFSET_DEG
    if AZIMUTH_INVERT:
        az = norm360(360.0 - az)
    if ELEVATION_INVERT:
        el = -el
    return az, el

def world_to_physical(az: float, el: float) -> Tuple[float, float]:
    return (max(AZ_PHYS_MIN, min(AZ_PHYS_MAX, az)),
            max(EL_PHYS_MIN, min(EL_PHYS_MAX, el)))

def physical_to_servo_deg(az_phys: float, el_phys: float) -> Tuple[float, float]:
    az_raw = az_phys / GEAR_RATIO
    el_raw = el_phys / GEAR_RATIO

    az_servo = max(0.0, min(SERVO_RANGE_DEG, az_raw))
    el_servo = max(0.0, min(SERVO_RANGE_DEG, el_raw))

    if az_servo != az_raw:
        print(f"[WARN] Az demand {az_phys:.1f}° phys -> {az_raw:.1f}° servo > {SERVO_RANGE_DEG}°. Clipped.")
    if el_servo != el_raw:
        print(f"[WARN] El demand {el_phys:.1f}° phys -> {el_raw:.1f}° servo > {SERVO_RANGE_DEG}°. Clipped.")

    return az_servo, el_servo

def servo_deg_to_us(deg: float) -> float:
    deg = max(0.0, min(SERVO_RANGE_DEG, deg))
    return PULSE_MIN_US + (deg / SERVO_RANGE_DEG) * (PULSE_MAX_US - PULSE_MIN_US)

# --- NEW: explicit park to servo 0°/0° regardless of calibration/inversion ---
def park_servos_zero(pi: pigpio.pi, dwell_s: float = 0.5):
    """Immediately drive both servos to 0° (servo degrees), then dwell."""
    us_az = servo_deg_to_us(0.0)
    us_el = servo_deg_to_us(0.0)
    pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
    pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
    print(f"[INIT] Parked servos to 0°/0° (µs {us_az:.0f}/{us_el:.0f}); dwell {dwell_s:.1f}s")
    time.sleep(dwell_s)

# =========================================================
# --------------------- pigpio init -----------------------
# =========================================================

def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running (sudo pigpiod)")
    pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
    print(f"[INFO] pigpio ready (AZ={SERVO_AZ_PIN}, EL={SERVO_EL_PIN})")
    return pi

# =========================================================
# --------------------- MAVLink I/O -----------------------
# =========================================================

def connect_mav(endpoint: str, baud: int, hb_required: bool) -> mavutil.mavfile:
    mav = mavutil.mavlink_connection(endpoint, baud=baud) if baud else mavutil.mavlink_connection(endpoint)
    try:
        mav.wait_heartbeat(timeout=5)
        print(f"[MAV] Connected {endpoint}  sys={mav.target_system} comp={mav.target_component}")
    except Exception as e:
        if hb_required:
            raise
        print(f"[MAV] No heartbeat on {endpoint} – continuing: {e}")

    for msg_id, interval in ((mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,    MAV_MSG_INTERVAL_US_GPS),
                             (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAV_MSG_INTERVAL_US_GLOBAL)):
        try:
            mav.mav.command_long_send(mav.target_system or 0, mav.target_component or 0,
                                      mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                      0, msg_id, interval, 0,0,0,0,0)
        except Exception:
            pass
    return mav

def _msg_to_lat_lon_alt(msg) -> Optional[Tuple[float, float, float]]:
    t = msg.get_type()
    if t == "GPS_RAW_INT":
        return msg.lat/1e7, msg.lon/1e7, msg.alt/1000.0
    if t == "GLOBAL_POSITION_INT":
        return msg.lat/1e7, msg.lon/1e7, msg.alt/1000.0
    return None

def start_reader(mav, tgt: Dict[str, Optional[float]], lock: threading.Lock):
    def _run():
        while not stop_event.is_set():
            msg = mav.recv_match(type=["GPS_RAW_INT","GLOBAL_POSITION_INT"],
                                 blocking=True, timeout=1.5)
            if not msg:
                continue
            vals = _msg_to_lat_lon_alt(msg)
            if vals:
                with lock:
                    tgt["lat"], tgt["lon"], tgt["alt"] = vals
    th = threading.Thread(target=_run, daemon=True)
    th.start()
    return th

def snapshot() -> Optional[Tuple[Dict[str,float], Dict[str,float]]]:
    with _drone_lock: d = drone_gps.copy()
    with _base_lock:  b = base_gps.copy()
    if None in (d["lat"],d["lon"],d["alt"], b["lat"],b["lon"],b["alt"]):
        return None
    return d, b

# =========================================================
# --------------------- CSV logging -----------------------
# =========================================================

_log_writer = _log_file = None
def log_open():
    global _log_writer,_log_file
    if not LOG_TO_CSV: return
    import csv, pathlib
    pathlib.Path("Tracker_Logs").mkdir(exist_ok=True)
    fn = pathlib.Path(f"Tracker_Logs/Tracker_{datetime.now():%Y%m%d-%H%M%S}.csv")
    _log_file = fn.open("w", newline="")
    _log_writer = csv.writer(_log_file)
    _log_writer.writerow(["Time","WorldAz","WorldEl","CalAz","CalEl",
                          "PhysAz","PhysEl","ServoAz","ServoEl",
                          "µsAz","µsEl","DroneLat","DroneLon","DroneAlt",
                          "BaseLat","BaseLon","BaseAlt"])
    print(f"[INFO] CSV log → {fn}")

def log_row(*row):
    if _log_writer: _log_writer.writerow(row); _log_file.flush()
def log_close():
    if _log_file: _log_file.close()

# =========================================================
# -----------------------  Main  --------------------------
# =========================================================

def main():
    print("=== Dual-GPS Tracker | startup park to 0°/0° | 180° servo → 360° phys ===")
    pi = setup_pigpio()
    log_open()

    # --- NEW: explicitly park to 0°/0° (servo degrees) at startup ---
    park_servos_zero(pi, dwell_s=0.5)

    # Connect MAVLink and start readers
    mav_drone = connect_mav(DRONE_ENDPOINT, DRONE_BAUD,  True)
    mav_base  = connect_mav(BASE_ENDPOINT,  BASE_BAUD,   False)
    start_reader(mav_drone, drone_gps, _drone_lock)
    start_reader(mav_base,  base_gps,  _base_lock)

    cycle = 0
    try:
        while not stop_event.is_set():
            snap = snapshot()
            if not snap:
                if cycle % PRINT_EVERY == 0:
                    print("[WAIT] waiting for both GPS fixes …")
                cycle += 1; time.sleep(0.2); continue

            drone, base = snap
            info = tracker.get_tracking_info(base["lat"], base["lon"], base["alt"],
                                             drone["lat"], drone["lon"], drone["alt"])
            if not info: time.sleep(UPDATE_PERIOD_S); continue

            w_az = info.get("adjusted_azimuth",  info["azimuth"])
            w_el = info.get("adjusted_elevation",info["elevation"])

            c_az,c_el   = apply_calibration(w_az, w_el)
            p_az,p_el   = world_to_physical(c_az, c_el)
            s_az,s_el   = physical_to_servo_deg(p_az, p_el)
            us_az,us_el = servo_deg_to_us(s_az), servo_deg_to_us(s_el)

            pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)

            if cycle % PRINT_EVERY == 0:
                print(f"[{datetime.now():%H:%M:%S}] "
                      f"WORLD {w_az:6.2f}/{w_el:5.2f}°  "
                      f"PHYS {p_az:6.2f}/{p_el:5.2f}°  "
                      f"SERVO {s_az:6.2f}/{s_el:5.2f}°  "
                      f"µs {us_az:5.0f}/{us_el:5.0f}")
            log_row(datetime.now().isoformat(timespec='seconds'),
                    round(w_az,3),round(w_el,3),
                    round(c_az,3),round(c_el,3),
                    round(p_az,3),round(p_el,3),
                    round(s_az,3),round(s_el,3),
                    round(us_az,1),round(us_el,1),
                    round(drone["lat"],7),round(drone["lon"],7),round(drone["alt"],1),
                    round(base["lat"],7), round(base["lon"],7), round(base["alt"],1))
            cycle += 1
            time.sleep(UPDATE_PERIOD_S)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl-C, shutting down")

    finally:
        stop_event.set()
        try:
            # Park to 0°/0° on shutdown, then release PWM
            park_servos_zero(pi, dwell_s=0.2)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, 0)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, 0)
            pi.stop()
        finally:
            log_close()

if __name__ == "__main__":
    main()
