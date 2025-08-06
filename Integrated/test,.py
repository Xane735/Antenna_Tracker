#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ====== CHANGE THIS FLAG ======
MODE = "ground"   # "ground" -> lauki_2.py behavior, "sim" -> geo_6.py behavior
# ==============================

import time
from datetime import datetime

# Common imports (both modes use these)
import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker

# -------------------------------------------------------------------
# -------------------------- GROUND MODE -----------------------------
# ------------------------ (lauki_2.py) ------------------------------
# -------------------------------------------------------------------

def run_ground():
    """
    This block reproduces lauki_2.py behavior (dual-GPS over serial, threads).
    Nothing changed except being wrapped in a function.
    """
    import threading
    from typing import Optional, Dict, Tuple

    # -------------------- Configuration ----------------------
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
    AZ_PHYS_MAX = 350.0        # full rotation (leave 10 ° safety)
    EL_PHYS_MIN = 0.0
    EL_PHYS_MAX = 180.0

    # Servo electrical limits – **180 ° servo**
    PULSE_MIN_US    = 900.0
    PULSE_MAX_US    = 2100.0
    SERVO_RANGE_DEG = 180.0

    # Calibration (exactly as in your lauki_2.py you shared)
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

    STARTUP_AZ_WORLD = 0.0
    STARTUP_EL_WORLD = 0.0

    # -------------------- Shared State -----------------------
    drone_gps = {"lat": None, "lon": None, "alt": None}
    base_gps  = {"lat": None, "lon": None, "alt": None}
    _drone_lock = threading.Lock()
    _base_lock  = threading.Lock()
    stop_event  = threading.Event()

    # ----------------- Helper functions ----------------------
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

    # --------------------- pigpio init -----------------------
    def setup_pigpio():
        pi = pigpio.pi()
        if not pi.connected:
            raise RuntimeError("pigpio daemon not running (sudo pigpiod)")
        pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
        pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
        print(f"[INFO] pigpio ready (AZ={SERVO_AZ_PIN}, EL={SERVO_EL_PIN})")
        return pi

    # --------------------- MAVLink I/O -----------------------
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

    # --------------------- CSV logging -----------------------
    _log_writer = _log_file = None
    def log_open():
        nonlocal _log_writer, _log_file
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

    # -----------------------  Main  --------------------------
    print("=== Dual-GPS Tracker | 180° servo → 360° phys (GROUND mode) ===")
    pi = setup_pigpio();   log_open()

    mav_drone = connect_mav(DRONE_ENDPOINT, DRONE_BAUD,  True)
    mav_base  = connect_mav(BASE_ENDPOINT,  BASE_BAUD,   False)
    start_reader(mav_drone, drone_gps, _drone_lock)
    start_reader(mav_base,  base_gps,  _base_lock)

    # park at startup pose
    s0_az, s0_el = physical_to_servo_deg(*world_to_physical(
        *apply_calibration(STARTUP_AZ_WORLD, STARTUP_EL_WORLD)))
    pi.set_servo_pulsewidth(SERVO_AZ_PIN, servo_deg_to_us(s0_az))
    pi.set_servo_pulsewidth(SERVO_EL_PIN, servo_deg_to_us(s0_el))
    time.sleep(0.3)

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
        print("\n[INFO] Ctrl-C – shutting down")

    finally:
        stop_event.set()
        try:
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, servo_deg_to_us(0.0))
            pi.set_servo_pulsewidth(SERVO_EL_PIN, servo_deg_to_us(0.0))
            time.sleep(0.2)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, 0)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, 0)
            pi.stop()
        finally:
            log_close()


# -------------------------------------------------------------------
# --------------------------- SIM MODE -------------------------------
# ------------------------- (geo_6.py) -------------------------------
# -------------------------------------------------------------------

def run_sim():
    """
    This block reproduces geo_6.py behavior (SITL over UDP, static base).
    Nothing changed except being wrapped in a function.
    """
    # -------------------- Configuration --------------------
    base_gps = {
        "lat": 13.0276816,
        "lon": 77.5630373,
        "alt": 931.13,          # metres ASL
    }

    SPOKES_SMALL = 12
    SPOKES_BIG   = 24
    GEAR_RATIO   = SPOKES_BIG / SPOKES_SMALL      # == 2.0

    AZ_PHYS_MIN = 0.0
    AZ_PHYS_MAX = 350.0        # keep a small safety margin
    EL_PHYS_MIN = 0.0
    EL_PHYS_MAX = 180.0

    PULSE_MIN_US    = 900.0    # tune endpoints if needed
    PULSE_MAX_US    = 2100.0
    SERVO_RANGE_DEG = 180.0

    AZIMUTH_ZERO_OFFSET_DEG   = 0.0
    ELEVATION_ZERO_OFFSET_DEG = 0.0
    AZIMUTH_INVERT   = True     # as in geo_6.py you shared
    ELEVATION_INVERT = False

    SERVO_AZ_PIN = 18
    SERVO_EL_PIN = 13

    UPDATE_PERIOD_S      = 0.20   # 5 Hz
    MAV_MSG_INTERVAL_US  = 200_000

    PRINT_EVERY = 1
    LOG_TO_CSV  = True

    # ------------------- Helper functions -----------------
    def norm360(x: float) -> float:
        return (x + 360.0) % 360.0

    def apply_calibration(az_world: float, el_world: float):
        az = norm360(az_world + AZIMUTH_ZERO_OFFSET_DEG)
        el = el_world + ELEVATION_ZERO_OFFSET_DEG
        if AZIMUTH_INVERT:
            az = norm360(360.0 - az)
        if ELEVATION_INVERT:
            el = -el
        return az, el

    def world_to_physical(az_world_cal: float, el_world_cal: float):
        az_phys = max(AZ_PHYS_MIN, min(AZ_PHYS_MAX, az_world_cal))
        el_phys = max(EL_PHYS_MIN, min(EL_PHYS_MAX, el_world_cal))
        return az_phys, el_phys

    def physical_to_servo_deg(az_phys: float, el_phys: float):
        az_raw = az_phys / GEAR_RATIO
        el_raw = el_phys / GEAR_RATIO

        az_servo = max(0.0, min(SERVO_RANGE_DEG, az_raw))
        el_servo = max(0.0, min(SERVO_RANGE_DEG, el_raw))

        if az_servo != az_raw:
            print(f"[WARN] Az demand {az_phys:.1f}° phys -> {az_raw:.1f}° servo exceeds "
                  f"{SERVO_RANGE_DEG}°. Clipped to {az_servo:.1f}°.")
        if el_servo != el_raw:
            print(f"[WARN] El demand {el_phys:.1f}° phys -> {el_raw:.1f}° servo exceeds "
                  f"{SERVO_RANGE_DEG}°. Clipped to {el_servo:.1f}°.")

        return az_servo, el_servo

    def servo_deg_to_us(servo_deg: float) -> float:
        servo_deg = max(0.0, min(SERVO_RANGE_DEG, servo_deg))
        return PULSE_MIN_US + (servo_deg / SERVO_RANGE_DEG) * (PULSE_MAX_US - PULSE_MIN_US)

    # ---------------------- pigpio init --------------------
    def setup_pigpio():
        pi = pigpio.pi()
        if not pi.connected:
            raise RuntimeError("pigpio daemon not running (start with sudo pigpiod)")
        pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
        pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
        print(f"[INFO] pigpio ready (AZ pin {SERVO_AZ_PIN}, EL pin {SERVO_EL_PIN})")
        return pi

    # ---------------------- MAVLink I/O --------------------
    def connect_mavlink():
        mav = mavutil.mavlink_connection("udp:0.0.0.0:14551")
        print("Waiting for heartbeat...")
        mav.wait_heartbeat()
        print(f"Connected (sys={mav.target_system}, comp={mav.target_component})")

        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            MAV_MSG_INTERVAL_US, 0, 0, 0, 0, 0
        )
        return mav

    # ----------------------- CSV log -----------------------
    _log_writer = None
    _log_file   = None

    def log_open():
        nonlocal _log_writer, _log_file
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

    # --------------------- Main loop -----------------------
    print("=== Antenna Tracker (pigpio, 2:1 gear, 360° capable) — SIM mode ===")
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

            info = tracker.get_tracking_info(
                base_gps["lat"], base_gps["lon"], base_gps["alt"],
                drone_lat, drone_lon, drone_alt
            )
            if not info:
                continue

            world_az = info.get("adjusted_azimuth", info["azimuth"])
            world_el = info.get("adjusted_elevation", info["elevation"])

            cal_az,  cal_el  = apply_calibration(world_az, world_el)
            phys_az, phys_el = world_to_physical(cal_az, cal_el)
            s_az, s_el       = physical_to_servo_deg(phys_az, phys_el)

            pulse_az = servo_deg_to_us(s_az)
            pulse_el = servo_deg_to_us(s_el)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, pulse_az)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, pulse_el)

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
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, servo_deg_to_us(0.0))
            pi.set_servo_pulsewidth(SERVO_EL_PIN, servo_deg_to_us(0.0))
            time.sleep(0.2)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, 0)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, 0)
            pi.stop()
        except Exception as e:
            print(f"[WARN] pigpio cleanup: {e}")
        log_close()


# ---------------------------------------------------------
# -------------------------- MAIN -------------------------
# ---------------------------------------------------------

if __name__ == "__main__":
    if MODE.lower() == "ground":
        run_ground()
    elif MODE.lower() == "sim":
        run_sim()
    else:
        raise SystemExit("Set MODE = 'ground' or 'sim' at the top of this file.")
