#!/usr/bin/env python3

# ======= CHANGE THIS FLAG =======
MODE = "ground"   # "sim" or "ground"
# ================================

import time
from datetime import datetime
import threading
from typing import Optional, Dict, Tuple

import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker

# ===================== Configuration =====================

base_static = {
    "lat": 13.0276816,
    "lon": 77.5630373,
    "alt": 931.13,          # metres ASL
}

# --- Endpoints ---
SIM_DRONE_ENDPOINT = "udp:0.0.0.0:14551"
SIM_DRONE_BAUD     = None
DRONE_ENDPOINT = "/dev/ttyACM0"
DRONE_BAUD     = 115200
BASE_ENDPOINT  = "/dev/ttyUSB0"
BASE_BAUD      = 57600

# MAVLink stream requests (5 Hz)
MAV_MSG_INTERVAL_US_GPS    = 200_000
MAV_MSG_INTERVAL_US_GLOBAL = 200_000

# Gear spokes → ratio
SPOKES_SMALL = 12
SPOKES_BIG   = 24
GEAR_RATIO   = SPOKES_BIG / SPOKES_SMALL      # 2.0

# Physical limits
AZ_PHYS_MIN = 0.0
AZ_PHYS_MAX = 350.0
EL_PHYS_MIN = 0.0
EL_PHYS_MAX = 180.0

# Servo mapping — 180° servo
PULSE_MIN_US    = 900.0
PULSE_MAX_US    = 2100.0
SERVO_RANGE_DEG = 180.0

# Calibration
AZIMUTH_ZERO_OFFSET_DEG   = 0.0
ELEVATION_ZERO_OFFSET_DEG = 0.0
AZIMUTH_INVERT   = True
ELEVATION_INVERT = False

# pigpio GPIO pins (BCM numbering)
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 17

# Loop timings
UPDATE_PERIOD_S = 0.10   # 50 Hz
PRINT_EVERY     = 1
LOG_TO_CSV      = True

# Startup pose is now forced to (0,0) servo
STARTUP_AZ_WORLD = 0.0
STARTUP_EL_WORLD = 0.0

# Shared state (GROUND mode)
drone_gps = {"lat": None, "lon": None, "alt": None}
base_gps  = {"lat": None, "lon": None, "alt": None}
_drone_lock = threading.Lock()
_base_lock  = threading.Lock()
stop_event  = threading.Event()

# ===== Helpers =====
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
        print(f"[WARN] Az phys {az_phys:.1f}° → servo {az_raw:.1f}° clipped to {SERVO_RANGE_DEG}°.")
    if el_servo != el_raw:
        print(f"[WARN] El phys {el_phys:.1f}° → servo {el_raw:.1f}° clipped to {SERVO_RANGE_DEG}°.")
    return az_servo, el_servo


def servo_deg_to_us(deg: float) -> float:
    deg = max(0.0, min(SERVO_RANGE_DEG, deg))
    return PULSE_MIN_US + (deg / SERVO_RANGE_DEG) * (PULSE_MAX_US - PULSE_MIN_US)

# ===== pigpio setup =====
def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running (sudo pigpiod)")
    pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
    print(f"[INFO] pigpio ready (AZ={SERVO_AZ_PIN}, EL={SERVO_EL_PIN})")
    return pi

# ===== MAVLink I/O =====
def connect_mav(endpoint: str, baud: Optional[int], hb_required: bool) -> mavutil.mavfile:
    mav = mavutil.mavlink_connection(endpoint, baud=baud) if baud else mavutil.mavlink_connection(endpoint)
    try:
        mav.wait_heartbeat(timeout=5)
        print(f"[MAV] Connected {endpoint} sys={mav.target_system} comp={mav.target_component}")
    except Exception as e:
        if hb_required:
            raise
        print(f"[MAV] No heartbeat on {endpoint} – continuing: {e}")
    for msg_id, interval in ((mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, MAV_MSG_INTERVAL_US_GPS),
                             (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAV_MSG_INTERVAL_US_GLOBAL)):
        try:
            mav.mav.command_long_send(
                mav.target_system or 0, mav.target_component or 0,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval, 0,0,0,0,0
            )
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


def start_reader(mav: mavutil.mavfile, target_dict: Dict[str, Optional[float]], lock: threading.Lock):
    def _run():
        while not stop_event.is_set():
            msg = mav.recv_match(type=["GPS_RAW_INT","GLOBAL_POSITION_INT"], blocking=True, timeout=1.5)
            if not msg:
                continue
            vals = _msg_to_lat_lon_alt(msg)
            if vals:
                with lock:
                    target_dict.update({"lat": vals[0], "lon": vals[1], "alt": vals[2]})
    th = threading.Thread(target=_run, daemon=True)
    th.start()
    return th

# ===== CSV logging =====
_log_writer = None
_log_file = None

def log_open():
    global _log_writer, _log_file
    if not LOG_TO_CSV:
        return
    import csv, pathlib
    pathlib.Path("Tracker_Logs").mkdir(exist_ok=True)
    fn = pathlib.Path(f"Tracker_Logs/Tracker_{datetime.now():%Y%m%d-%H%M%S}.csv")
    _log_file = fn.open("w", newline="")
    _log_writer = csv.writer(_log_file)
    _log_writer.writerow([
        "Time","WorldAz","WorldEl","CalAz","CalEl",
        "PhysAz","PhysEl","ServoAz","ServoEl","µsAz","µsEl",
        "DroneLat","DroneLon","DroneAlt","BaseLat","BaseLon","BaseAlt"
    ])
    print(f"[INFO] CSV log → {fn}")

def log_row(*row):
    if _log_writer:
        _log_writer.writerow(row)
        _log_file.flush()

def log_close():
    if _log_file:
        _log_file.close()

# ===== Snapshots =====
def snapshot_ground() -> Optional[Tuple[Dict[str,float], Dict[str,float]]]:
    with _drone_lock: d = drone_gps.copy()
    with _base_lock:  b = base_gps.copy()
    # Ensure both drone and base GPS have valid data
    if None in d.values() or None in b.values():
        return None
    return d, b

def snapshot_sim() -> Optional[Tuple[Dict[str,float], Dict[str,float]]]:
    with _drone_lock: d = drone_gps.copy()
    if None in (d.values()):
        return None
    b = {**base_static}
    return d, b

# ===== Main =====
def main():
    print(f"[CONFIG] Mode: {MODE}, Gear {GEAR_RATIO}:1, Servo 180° over {PULSE_MIN_US}-{PULSE_MAX_US}µs")

    pi = setup_pigpio()
    log_open()

    # Connections per mode
    if MODE.lower() == "sim":
        mav_drone = connect_mav(SIM_DRONE_ENDPOINT, SIM_DRONE_BAUD, True)
        start_reader(mav_drone, drone_gps, _drone_lock)
        snapshot_fn = snapshot_sim
        print(f"[CFG] SIM base @ {base_static['lat']}, {base_static['lon']}, {base_static['alt']}m")
    elif MODE.lower() == "ground":
        mav_drone = connect_mav(DRONE_ENDPOINT, DRONE_BAUD, True)
        mav_base  = connect_mav(BASE_ENDPOINT,  BASE_BAUD,  False)
        start_reader(mav_drone, drone_gps, _drone_lock)
        start_reader(mav_base,  base_gps,  _base_lock)
        snapshot_fn = snapshot_ground
    else:
        raise SystemExit("Set MODE = 'sim' or 'ground'")

    # Zero-reference setup
    zero_world_az = None
    zero_world_el = None
    print("[INFO] Point the tracker at the drone and stabilize GPS.")
    print("[INFO] Waiting 10 seconds for GPS Stabalization")
    time.sleep(10)
    snap0 = snapshot_fn()
    if snap0:
        d0, b0 = snap0
        info0 = tracker.get_tracking_info(
            b0['lat'], b0['lon'], b0['alt'],
            d0['lat'], d0['lon'], d0['alt']
        )
        if info0:
            zero_world_az = info0.get('adjusted_azimuth', info0['azimuth'])
            zero_world_el = info0.get('adjusted_elevation', info0['elevation'])
            print(f"[INFO] Zero ref set: AZ={zero_world_az:.2f}°, EL={zero_world_el:.2f}°")
        else:
            print("[WARN] tracker.get_tracking_info failed for zeroing.")
    else:
        print("[WARN] Could not obtain initial GPS snapshot for zeroing.")

    # Park at servo (0,0)
    print("Point the tracker towards the drone")
    pi.set_servo_pulsewidth(SERVO_AZ_PIN, servo_deg_to_us(0.0))
    pi.set_servo_pulsewidth(SERVO_EL_PIN, servo_deg_to_us(0.0))
    time.sleep(5)

    cycle = 0
    try:
        while not stop_event.is_set():
            snap = snapshot_fn()
            if not snap:
                if cycle % PRINT_EVERY == 0:
                    print("[WAIT] waiting for GPS…")
                cycle += 1
                time.sleep(0.2)
                continue
            drone, base = snap
            info = tracker.get_tracking_info(
                base['lat'], base['lon'], base['alt'],
                drone['lat'], drone['lon'], drone['alt']
            )
            if not info:
                time.sleep(UPDATE_PERIOD_S)
                continue
            # compute relative angles
            abs_az = info.get('adjusted_azimuth', info['azimuth'])
            abs_el = info.get('adjusted_elevation', info['elevation'])
            world_az = norm360(abs_az - (zero_world_az or 0.0))
            world_el = abs_el - (zero_world_el or 0.0)

            # apply calibration, clamp, gearing
            cal_az, cal_el = apply_calibration(world_az, world_el)
            phys_az, phys_el = world_to_physical(cal_az, cal_el)
            s_az, s_el = physical_to_servo_deg(phys_az, phys_el)

            # drive servos
            us_az = servo_deg_to_us(s_az)
            us_el = servo_deg_to_us(s_el)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)

            # console output
            if cycle % PRINT_EVERY == 0:
                print(f"[{datetime.now():%H:%M:%S}] WORLD {world_az:6.2f}/{world_el:5.2f}°  "
                      f"PHYS {phys_az:6.2f}/{phys_el:5.2f}°  "
                      f"SERVO {s_az:6.2f}/{s_el:6.2f}°  "
                      f"µs {us_az:5.0f}/{us_el:5.0f}")

            # CSV log
            log_row(
                datetime.now().isoformat(timespec='seconds'),
                round(world_az,3), round(world_el,3),
                round(cal_az,3), round(cal_el,3),
                round(phys_az,3), round(phys_el,3),
                round(s_az,3), round(s_el,3),
                round(us_az,1), round(us_el,1),
                round(drone['lat'],7), round(drone['lon'],7), round(drone['alt'],2),
                round(base['lat'],7), round(base['lon'],7), round(base['alt'],2)
            )

            cycle += 1
            time.sleep(UPDATE_PERIOD_S)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
    finally:
        stop_event.set()
        try:
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