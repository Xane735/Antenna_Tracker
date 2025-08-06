#Version 1, not tested yet on simulation or Ground
#!/usr/bin/env python3
"""
Unified Antenna Tracker (Simulation / Ground) @ up to 20 Hz

Modes:
  sim    : Drone GPS via UDP MAVLink; Base location from static config (below).
  ground : Drone GPS via serial MAVLink (/dev/ttyACM0), Base GPS via serial MAVLink (/dev/ttyUSB0).

Features:
  • 180° servos (900-2100 µs) with 2:1 gearing → ~360° physical az range.
  • Startup park to 0°/0° (can be disabled with --no-park).
  • 20 Hz loop + 20 Hz MAVLink requests (tunable).
  • GPS staleness guard.
  • Az unwrap + slew limiting + optional backlash compensation.
  • Console shows IDEAL(World), PHYS target, PHYS commanded, SERVO, µs, GPS ages.
  • CSV logging.

Example:
  python3 sim_ground.py --mode sim
  python sim_ground.py --mode ground
"""
import argparse
import time
from datetime import datetime
import threading
from typing import Optional, Dict, Tuple

import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker  # your angle/distance calculator

# ========================= Default Config =========================

# Simulation defaults (SITL/Mission Planner)
SIM_DRONE_ENDPOINT = "192.168.1.193"

# Ground defaults (real autopilot & base GPS)
GR_DRONE_ENDPOINT  = "/dev/ttyACM0"
GR_DRONE_BAUD      = 115200
GR_BASE_ENDPOINT   = "/dev/ttyUSB0"
GR_BASE_BAUD       = 57600

# Static base position for SIM mode (ASL in meters)
BASE_STATIC = {"lat": 13.0272372, "lon": 77.5631011, "alt": 931.13}

# MAVLink stream rates (20 Hz = 50,000 µs). Use 100,000 for ~10 Hz
DEFAULT_MAV_US_GPS    = 50_000   # GPS_RAW_INT
DEFAULT_MAV_US_GLOBAL = 50_000   # GLOBAL_POSITION_INT

# Mechanical / servo
SPOKES_SMALL   = 12
SPOKES_BIG     = 24
GEAR_RATIO     = SPOKES_BIG / SPOKES_SMALL  # 2.0 (physical : servo)
AZ_PHYS_MIN    = 0.0                        # Physical minimum azimuth of the tracker
AZ_PHYS_MAX    = 350.0                      # Physical maximum azimuth of the tracker
EL_PHYS_MIN    = 0.0                        # Physical minimum elevation of the tracker
EL_PHYS_MAX    = 180.0                      # Physical maximum elevation of the tracker
PULSE_MIN_US   = 900.0                      # Minimum PWM (microseconds)
PULSE_MAX_US   = 2100.0                     # Maximum PWM (microseconds)
SERVO_RANGE_DEG= 180.0                      # Maximum Range possible by the servo

# Calibration (world frame)
AZIMUTH_ZERO_OFFSET_DEG   = 0.0
ELEVATION_ZERO_OFFSET_DEG = 0.0
AZIMUTH_INVERT   = False
ELEVATION_INVERT = False

# GPIO (BCM)
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 13

# Rates / filtering / shaping
DEFAULT_UPDATE_PERIOD_S = 0.05              # 20 Hz loop
GPS_STALE_SEC           = 0.5               
MAX_AZ_SLEW_DPS         = 180.0             # Maximum azimuth slew rate
MAX_EL_SLEW_DPS         = 120.0             # Maximum elevation slew rate
BACKLASH_DEG            = 0.5

# Logging
DEFAULT_PRINT_EVERY     = 1
DEFAULT_LOG_TO_CSV      = True

# ========================= Shared State ==========================

drone_gps = {"lat": None, "lon": None, "alt": None, "t": None}
base_gps  = {"lat": None, "lon": None, "alt": None, "t": None}

_drone_lock = threading.Lock()
_base_lock  = threading.Lock()
stop_event  = threading.Event()

# ========================= Helpers ===============================

def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

def apply_calibration(az_world: float, el_world: float) -> Tuple[float, float]:
    az = norm360(az_world + AZIMUTH_ZERO_OFFSET_DEG)
    el = el_world + ELEVATION_ZERO_OFFSET_DEG
    if AZIMUTH_INVERT:
        az = norm360(360.0 - az)
    if ELEVATION_INVERT:
        el = -el
    return az, el

def world_to_physical(az_world_cal: float, el_world_cal: float) -> Tuple[float, float]:
    az_phys = max(AZ_PHYS_MIN, min(AZ_PHYS_MAX, az_world_cal))
    el_phys = max(EL_PHYS_MIN, min(EL_PHYS_MAX, el_world_cal))
    return az_phys, el_phys

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

def servo_deg_to_us(servo_deg: float) -> float:
    servo_deg = max(0.0, min(SERVO_RANGE_DEG, servo_deg))
    return PULSE_MIN_US + (servo_deg / SERVO_RANGE_DEG) * (PULSE_MAX_US - PULSE_MIN_US)

def shortest_delta_deg(target: float, current: float) -> float:
    return (target - current + 540.0) % 360.0 - 180.0

class AxisShaper360:
    """Azimuth shaper: unwrap + slew limit + optional backlash (wraps at 0/360)."""
    def __init__(self, max_slew_dps: float, backlash_deg: float = 0.0):
        self.phys = None
        self.last_dir = 0
        self.max_slew_dps = max_slew_dps
        self.backlash_deg = backlash_deg
    def step(self, target_phys: float, dt: float) -> float:
        if self.phys is None:
            self.phys = target_phys % 360.0
            self.last_dir = 0
            return self.phys
        err = shortest_delta_deg(target_phys, self.phys)
        new_dir = 1 if err > 0 else (-1 if err < 0 else 0)
        if self.backlash_deg > 0.0 and new_dir != 0 and self.last_dir not in (0, new_dir):
            err += new_dir * self.backlash_deg
        max_step = self.max_slew_dps * dt
        step = max(-max_step, min(max_step, err))
        self.phys = (self.phys + step) % 360.0
        if abs(step) > 1e-6:
            self.last_dir = new_dir
        return self.phys

class AxisShaperLinear:
    """Elevation shaper: linear (no wrap), slew limit & clamp."""
    def __init__(self, max_slew_dps: float, min_deg: float, max_deg: float, backlash_deg: float = 0.0):
        self.phys = None
        self.last_dir = 0
        self.max_slew_dps = max_slew_dps
        self.min_deg = min_deg
        self.max_deg = max_deg
        self.backlash_deg = backlash_deg
    def step(self, target_phys: float, dt: float) -> float:
        target = max(self.min_deg, min(self.max_deg, target_phys))
        if self.phys is None:
            self.phys = target
            self.last_dir = 0
            return self.phys
        err = target - self.phys
        new_dir = 1 if err > 0 else (-1 if err < 0 else 0)
        if self.backlash_deg > 0.0 and new_dir != 0 and self.last_dir not in (0, new_dir):
            err += new_dir * self.backlash_deg
        max_step = self.max_slew_dps * dt
        step = max(-max_step, min(max_step, err))
        self.phys = max(self.min_deg, min(self.max_deg, self.phys + step))
        if abs(step) > 1e-6:
            self.last_dir = new_dir
        return self.phys

# ========================= pigpio ================================

def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running. Start with: sudo pigpiod")
    pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
    print(f"[INFO] pigpio ready (pins AZ={SERVO_AZ_PIN}, EL={SERVO_EL_PIN})")
    return pi

def park_servos_zero(pi: pigpio.pi, dwell_s: float = 0.5):
    us_az = servo_deg_to_us(0.0)
    us_el = servo_deg_to_us(0.0)
    pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
    pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
    print(f"[INIT] Parked servos to 0°/0° (µs {us_az:.0f}/{us_el:.0f}); dwell {dwell_s:.1f}s")
    time.sleep(dwell_s)

# ========================= MAVLink I/O ===========================

def connect_mavlink(endpoint: str, baud: Optional[int], require_heartbeat: bool,
                    req_us_gps: int, req_us_global: int) -> mavutil.mavfile:
    if baud:
        mav = mavutil.mavlink_connection(endpoint, baud=baud)
    else:
        mav = mavutil.mavlink_connection(endpoint)
    print(f"[MAV] Waiting for heartbeat on {endpoint}...")
    try:
        mav.wait_heartbeat(timeout=5)
        print(f"[MAV] Connected ({endpoint}) sys={mav.target_system} comp={mav.target_component}")
    except Exception as e:
        if require_heartbeat:
            raise
        print(f"[MAV] No heartbeat on {endpoint} (continuing): {e}")
    # Request high-rate streams (best-effort)
    for msg_id, interval in (
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, req_us_gps),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, req_us_global),
    ):
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

def start_reader(mav: mavutil.mavfile, target_dict: Dict[str, Optional[float]], lock: threading.Lock, timeout_s=0.2):
    def _run():
        while not stop_event.is_set():
            msg = mav.recv_match(type=["GPS_RAW_INT", "GLOBAL_POSITION_INT"], blocking=True, timeout=timeout_s)
            if not msg:
                continue
            vals = _msg_to_lat_lon_alt(msg)
            if vals:
                lat, lon, alt = vals
                with lock:
                    target_dict["lat"] = lat
                    target_dict["lon"] = lon
                    target_dict["alt"] = alt
                    target_dict["t"]   = time.time()
    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return t

# ========================= CSV logging ===========================

_log_writer = None
_log_file = None

def log_open(enabled: bool):
    if not enabled:
        return
    import csv
    from pathlib import Path
    Path("Tracker_Logs").mkdir(exist_ok=True)
    fn = Path(f"Tracker_Logs/Tracker_Log{datetime.now().strftime('%Y%m%d-%H%M%S')}.csv")
    global _log_file, _log_writer
    _log_file = fn.open("w", newline="")
    _log_writer = csv.writer(_log_file)
    _log_writer.writerow([
        "Time",
        "WorldAz","WorldEl",
        "PhysAz_tgt","PhysEl_tgt",
        "PhysAz_cmd","PhysEl_cmd",
        "ServoAz_deg","ServoEl_deg",
        "PulseAz_us","PulseEl_us",
        "DroneLat","DroneLon","DroneAlt","AgeD_ms",
        "BaseLat","BaseLon","BaseAlt","AgeB_ms"
    ])
    print(f"[INFO] CSV log: {fn}")

def log_row(enabled: bool, *row):
    if enabled and _log_writer:
        _log_writer.writerow(row)
        _log_file.flush()

def log_close(enabled: bool):
    if enabled and _log_file:
        _log_file.close()

# ========================= Snapshots =============================

def snapshot_sim() -> Optional[Tuple[Dict[str, float], Dict[str, float]]]:
    # Drone must be valid + fresh; Base comes from static config (always "fresh")
    with _drone_lock:
        d = drone_gps.copy()
    if None in (d["lat"], d["lon"], d["alt"], d["t"]):
        return None
    now = time.time()
    if (now - d["t"] > GPS_STALE_SEC):
        return None
    # synthesize "fresh" base from static
    b = {"lat": BASE_STATIC["lat"], "lon": BASE_STATIC["lon"], "alt": BASE_STATIC["alt"], "t": now}
    return d, b

def snapshot_ground() -> Optional[Tuple[Dict[str, float], Dict[str, float]]]:
    with _drone_lock:
        d = drone_gps.copy()
    with _base_lock:
        b = base_gps.copy()
    if None in (d["lat"], d["lon"], d["alt"], d["t"], b["lat"], b["lon"], b["alt"], b["t"]):
        return None
    now = time.time()
    if (now - d["t"] > GPS_STALE_SEC) or (now - b["t"] > GPS_STALE_SEC):
        return None
    return d, b

# ========================= Main =================================

def main():
    ap = argparse.ArgumentParser(description="Unified Antenna Tracker (sim/ground)")
    ap.add_argument("--mode", choices=["sim","ground"], default="sim")
    ap.add_argument("--drone-endpoint", default=None,
                    help="Override drone endpoint (e.g., udp:0.0.0.0:14551 or /dev/ttyACM0)")
    ap.add_argument("--drone-baud", type=int, default=None, help="Override drone baud (e.g., 115200)")
    ap.add_argument("--base-endpoint", default=None, help="Override base GPS endpoint (ground mode)")
    ap.add_argument("--base-baud", type=int, default=None, help="Override base GPS baud (ground mode)")
    ap.add_argument("--hz", type=float, default=20.0, help="Loop frequency (Hz), e.g., 20 or 10")
    ap.add_argument("--mav-hz", type=float, default=20.0, help="Requested MAVLink rate (Hz), e.g., 20 or 10")
    ap.add_argument("--print-every", type=int, default=DEFAULT_PRINT_EVERY, help="Print every N cycles")
    ap.add_argument("--no-park", action="store_true", help="Skip startup park to 0°/0°")
    ap.add_argument("--log", action="store_true", default=DEFAULT_LOG_TO_CSV, help="Log to CSV")
    ap.add_argument("--az-slew", type=float, default=MAX_AZ_SLEW_DPS, help="Az max slew (deg/s)")
    ap.add_argument("--el-slew", type=float, default=MAX_EL_SLEW_DPS, help="El max slew (deg/s)")
    ap.add_argument("--backlash", type=float, default=BACKLASH_DEG, help="Backlash compensation (deg)")
    args = ap.parse_args()

    # Derived timings
    update_period_s = max(0.01, 1.0 / args.hz)
    mav_us = int(max(1.0, 1.0 / max(1.0, args.mav_hz)) * 1_000_000)

    # Endpoints by mode
    if args.mode == "sim":
        drone_ep = args.drone-endpoint if args.drone_endpoint else SIM_DRONE_ENDPOINT
        base_ep  = None
        base_bd  = None
        snapshot_fn = snapshot_sim
    else:
        drone_ep = args.drone_endpoint if args.drone_endpoint else GR_DRONE_ENDPOINT
        drone_bd = args.drone_baud if args.drone_baud is not None else GR_DRONE_BAUD
        base_ep  = args.base_endpoint  if args.base_endpoint  else GR_BASE_ENDPOINT
        base_bd  = args.base_baud      if args.base_baud is not None else GR_BASE_BAUD
        snapshot_fn = snapshot_ground

    # Print config summary
    print("=== Unified Tracker ===")
    print(f"[CFG] Mode={args.mode} | Loop={args.hz:.1f} Hz | MAVLink req={args.mav_hz:.1f} Hz | PrintEvery={args.print_every}")
    print(f"[CFG] Drone endpoint={drone_ep} baud={drone_bd}")
    if args.mode == "ground":
        print(f"[CFG] Base  endpoint={base_ep} baud={base_bd}")
    else:
        print(f"[CFG] Base static lat={BASE_STATIC['lat']}, lon={BASE_STATIC['lon']}, alt={BASE_STATIC['alt']} m")
    print(f"[CFG] Servo: range={SERVO_RANGE_DEG}°, pulses={PULSE_MIN_US:.0f}-{PULSE_MAX_US:.0f} µs | Gear={GEAR_RATIO:.2f}:1")
    print(f"[CFG] Limits: AZ {AZ_PHYS_MIN}..{AZ_PHYS_MAX}°, EL {EL_PHYS_MIN}..{EL_PHYS_MAX}°")
    print(f"[CFG] Slew: AZ={args.az_slew}°/s, EL={args.el_slew}°/s, Backlash={args.backlash}°")
    print(f"[CFG] Park on start: {not args.no_park} | CSV log: {args.log}")

    # pigpio + optional park
    pi = setup_pigpio()
    if not args.no_park:
        park_servos_zero(pi, dwell_s=0.5)

    # CSV
    log_open(args.log)

    # Connect MAVLink and start readers
    mav_drone = connect_mavlink(drone_ep, drone_bd, True,  mav_us, mav_us)
    start_reader(mav_drone, drone_gps, _drone_lock, timeout_s=0.2)

    if args.mode == "ground":
        mav_base  = connect_mavlink(base_ep,  base_bd,  False, mav_us, mav_us)
        start_reader(mav_base,  base_gps,  _base_lock,  timeout_s=0.2)

    # Motion shapers
    az_shaper = AxisShaper360(args.az_slew, args.backlash)
    el_shaper = AxisShaperLinear(args.el_slew, EL_PHYS_MIN, EL_PHYS_MAX, args.backlash)

    cycle = 0
    last = time.time()
    try:
        while not stop_event.is_set():
            loop_start = time.time()
            dt = max(1e-3, loop_start - last)
            last = loop_start

            snap = snapshot_fn()
            if not snap:
                if (cycle % max(1, args.print_every)) == 0:
                    print("[WAIT] Waiting for recent GPS…")
                cycle += 1
                # pace to target rate
                sleep_left = update_period_s - (time.time() - loop_start)
                if sleep_left > 0: time.sleep(sleep_left)
                continue

            drone, base = snap
            age_d = loop_start - drone["t"]
            age_b = loop_start - base["t"]

            info = tracker.get_tracking_info(base["lat"], base["lon"], base["alt"],
                                             drone["lat"], drone["lon"], drone["alt"])
            if not info:
                sleep_left = update_period_s - (time.time() - loop_start)
                if sleep_left > 0: time.sleep(sleep_left)
                continue

            # Keep the ideal world angles, then form command angles
            ideal_world_az = info["azimuth"]
            ideal_world_el = info["elevation"]
            cmd_world_az   = info.get("adjusted_azimuth",  ideal_world_az)
            cmd_world_el   = info.get("adjusted_elevation", ideal_world_el)

            # World → calibrated → physical (target)
            cal_az, cal_el     = apply_calibration(cmd_world_az, cmd_world_el)
            phys_az_t, phys_el_t = world_to_physical(cal_az, cal_el)

            # Shaped physical command (unwrap + slew + backlash)
            phys_az_cmd = az_shaper.step(phys_az_t, dt)
            phys_el_cmd = el_shaper.step(phys_el_t, dt)

            # Servo mapping & drive
            s_az, s_el = physical_to_servo_deg(phys_az_cmd, phys_el_cmd)
            pulse_az   = servo_deg_to_us(s_az)
            pulse_el   = servo_deg_to_us(s_el)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, pulse_az)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, pulse_el)

            # Console
            if (cycle % args.print_every) == 0:
                az_err = shortest_delta_deg(ideal_world_az, cmd_world_az)
                el_err = ideal_world_el - cmd_world_el
                print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                      f"IDEAL {ideal_world_az:6.2f}/{ideal_world_el:5.2f}°  "
                      f"CMD {cmd_world_az:6.2f}/{cmd_world_el:5.2f}°  "
                      f"PHYS tgt {phys_az_t:6.2f}/{phys_el_t:5.2f}°  "
                      f"PHYS cmd {phys_az_cmd:6.2f}/{phys_el_cmd:5.2f}°  "
                      f"SERVO {s_az:6.2f}/{s_el:5.2f}°  "
                      f"µs {pulse_az:5.0f}/{pulse_el:5.0f}  "
                      f"Age D/B {age_d*1000:4.0f}/{age_b*1000:4.0f} ms  "
                      f"OFF Az {az_err:+5.1f}° El {el_err:+5.1f}°")

            # CSV
            log_row(args.log,
                datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                round(ideal_world_az,3), round(ideal_world_el,3),
                round(phys_az_t,3), round(phys_el_t,3),
                round(phys_az_cmd,3), round(phys_el_cmd,3),
                round(s_az,3), round(s_el,3),
                round(pulse_az,1), round(pulse_el,1),
                round(drone["lat"],7), round(drone["lon"],7), round(drone["alt"],2), round(age_d*1000,0),
                round(base["lat"],7),  round(base["lon"],7),  round(base["alt"],2),  round(age_b*1000,0)
            )

            cycle += 1
            # pace to target loop rate
            sleep_left = update_period_s - (time.time() - loop_start)
            if sleep_left > 0:
                time.sleep(sleep_left)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
    finally:
        stop_event.set()
        try:
            # Park to 0/0 on shutdown, then release PWM
            if not args.no_park:
                park_servos_zero(pi, dwell_s=0.2)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, 0)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, 0)
            pi.stop()
        except Exception as e:
            print(f"[WARN] pigpio cleanup: {e}")
        log_close(args.log)

if __name__ == "__main__":
    main()
