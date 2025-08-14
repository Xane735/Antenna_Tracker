#!/usr/bin/env python3
"""
geo_8.py — unified tracker with:
  • Modes: "sim" (SITL) or "ground" (USB Pixhawk + USB base GPS)
  • Base position modes (GROUND only):
      - dynamic: use live base GPS with rolling median filter
      - static : auto-snapshot base once stabilized, then hold fixed
  • Zero-reference capture (initial point-at-drone) + exponential smoothing
  • Rich CSV logging (+ optional raw GPS logging)

Why this helps your close-proximity tests:
  - "static" base mode mitigates short-range drift by locking the base coordinates
  - "dynamic" still uses two live streams but filters base jitter via a rolling median
  - Raw GPS logging lets you compare/diagnose stream precision later
"""

import argparse
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import math
import threading
from typing import Optional, Dict, Tuple, Callable

import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker

# ===================== Defaults (overridable by CLI) =====================

MODE_DEFAULT = "ground"      # "sim" or "ground"
BASE_MODE_DEFAULT = "dynamic"  # "dynamic" or "static"   (only used in ground mode)

# --- Endpoints ---
SIM_DRONE_ENDPOINT = "udp:0.0.0.0:14551"
SIM_DRONE_BAUD     = None
DRONE_ENDPOINT     = "/dev/ttyACM0"
DRONE_BAUD         = 115200
BASE_ENDPOINT      = "/dev/ttyUSB0"
BASE_BAUD          = 57600

# MAVLink stream requests (~5 Hz)
MAV_MSG_INTERVAL_US_GPS    = 200_000
MAV_MSG_INTERVAL_US_GLOBAL = 200_000

# Gear spokes → ratio
SPOKES_SMALL = 12
SPOKES_BIG   = 24
GEAR_RATIO   = SPOKES_BIG / SPOKES_SMALL      # == 2.0

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
UPDATE_PERIOD_S = 0.10   # 10 Hz default (set higher if your servos prefer slower updates)
PRINT_PERIOD_S  = 1.0
LOG_TO_CSV      = True
LOG_RAW_GPS     = True   # per-message GPS capture (for precision analysis)

# SIM base (used only in SIM mode)
base_static = {
    "lat": 13.0272255,
    "lon": 77.5630997,
    "alt": 931.13,          # metres ASL
}

# ============= Helpers & Small Utilities =============

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
    return az_servo, el_servo

def servo_deg_to_us(deg: float) -> float:
    deg = max(0.0, min(SERVO_RANGE_DEG, deg))
    return PULSE_MIN_US + (deg / SERVO_RANGE_DEG) * (PULSE_MAX_US - PULSE_MIN_US)

# Quick deg→meters conversion (approx), good enough for stability heuristics
def latlon_to_meters(lat_deg: float, lon_deg: float, ref_lat_deg: float) -> Tuple[float,float]:
    lat_m = lat_deg * 111_320.0
    lon_m = lon_deg * 111_320.0 * math.cos(math.radians(ref_lat_deg))
    return lat_m, lon_m

# ============= Thread-safe GPS sample stores =============

@dataclass
class GpsSample:
    t: float
    lat: float
    lon: float
    alt: float
    eph: Optional[float] = None     # horizontal accuracy (m) if available
    epv: Optional[float] = None     # vertical accuracy (m) if available
    fix_type: Optional[int] = None  # 0..6 (3=3D fix)
    sats: Optional[int] = None

def _extract_sample(msg) -> Optional[GpsSample]:
    t = time.time()
    tp = msg.get_type()
    if tp == "GPS_RAW_INT":
        # According to MAVLink: eph/epv in cm (or mm in some dialects); we map to meters conservatively if present
        eph = getattr(msg, "eph", None)
        epv = getattr(msg, "epv", None)
        # convert to meters if it looks like cm or mm
        def to_m(x):
            if x is None: return None
            if x > 1000:  # assume mm
                return x / 1000.0
            else:         # assume cm
                return x / 100.0
        return GpsSample(
            t=t,
            lat=msg.lat/1e7,
            lon=msg.lon/1e7,
            alt=msg.alt/1000.0,
            eph=to_m(eph),
            epv=to_m(epv),
            fix_type=getattr(msg,"fix_type",None),
            sats=getattr(msg,"satellites_visible",None)
        )
    if tp == "GLOBAL_POSITION_INT":
        return GpsSample(
            t=t,
            lat=msg.lat/1e7,
            lon=msg.lon/1e7,
            alt=msg.alt/1000.0
        )
    return None

class GpsBuffer:
    def __init__(self, name: str, maxlen: int = 120):
        self.name = name
        self._buf: deque[GpsSample] = deque(maxlen=maxlen)
        self._lock = threading.Lock()
    def push(self, s: GpsSample):
        with self._lock:
            self._buf.append(s)
    def latest(self) -> Optional[GpsSample]:
        with self._lock:
            return self._buf[-1] if self._buf else None
    def snapshot(self) -> Tuple[Optional[GpsSample], Tuple[GpsSample, ...]]:
        with self._lock:
            last = self._buf[-1] if self._buf else None
            return last, tuple(self._buf)

# ============= pigpio =============
def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running (sudo pigpiod)")
    pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
    print(f"[INFO] pigpio ready (AZ={SERVO_AZ_PIN}, EL={SERVO_EL_PIN})")
    return pi

# ============= MAVLink I/O =============
def connect_mav(endpoint: str, baud: Optional[int], hb_required: bool) -> mavutil.mavfile:
    mav = mavutil.mavlink_connection(endpoint, baud=baud) if baud else mavutil.mavlink_connection(endpoint)
    try:
        mav.wait_heartbeat(timeout=5)
        print(f"[MAV] Connected {endpoint} sys={mav.target_system} comp={mav.target_component}")
    except Exception as e:
        if hb_required:
            raise
        print(f"[MAV] No heartbeat on {endpoint} – continuing: {e}")
    for msg_id, interval in (
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,    MAV_MSG_INTERVAL_US_GPS),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAV_MSG_INTERVAL_US_GLOBAL),
    ):
        try:
            mav.mav.command_long_send(
                mav.target_system or 0, mav.target_component or 0,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval, 0,0,0,0,0
            )
        except Exception:
            pass
    return mav

def start_reader(mav: mavutil.mavfile, buf: GpsBuffer, on_raw: Optional[Callable[[str,GpsSample],None]] = None):
    def _run():
        while True:
            msg = mav.recv_match(type=["GPS_RAW_INT","GLOBAL_POSITION_INT"], blocking=True, timeout=1.5)
            if not msg:
                continue
            s = _extract_sample(msg)
            if s:
                buf.push(s)
                if on_raw:
                    on_raw(buf.name, s)
    th = threading.Thread(target=_run, daemon=True)
    th.start()
    return th

# ============= CSV logging =============
_log_writer = None
_log_file = None
_raw_writer = None
_raw_file = None

def log_open(prefix="Tracker"):
    global _log_writer, _log_file, _raw_writer, _raw_file
    if not LOG_TO_CSV and not LOG_RAW_GPS:
        return
    import csv, pathlib
    pathlib.Path("Tracker_Logs").mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    if LOG_TO_CSV:
        fn = pathlib.Path(f"Tracker_Logs/{prefix}_{ts}.csv")
        _log_file = fn.open("w", newline="")
        _log_writer = csv.writer(_log_file)
        _log_writer.writerow([
            "Time",
            "BaseMode","BaseLocked",
            "WorldAz","WorldEl","CalAz","CalEl",
            "PhysAz","PhysEl","ServoAz","ServoEl","µsAz","µsEl",
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

def log_row(*row):
    if _log_writer:
        _log_writer.writerow(row)
        _log_file.flush()

def log_raw(stream: str, s: GpsSample):
    if _raw_writer:
        _raw_writer.writerow([datetime.now().isoformat(timespec='seconds'),
                              stream, f"{s.lat:.7f}", f"{s.lon:.7f}", f"{s.alt:.2f}",
                              "" if s.eph is None else f"{s.eph:.2f}",
                              "" if s.epv is None else f"{s.epv:.2f}",
                              "" if s.fix_type is None else s.fix_type,
                              "" if s.sats is None else s.sats])
        _raw_file.flush()

def log_close():
    if _log_file: _log_file.close()
    if _raw_file: _raw_file.close()

# ============= Base filters & stabilizer =============

def rolling_median(values):
    s = sorted(values)
    n = len(s)
    if n == 0: return None
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5*(s[mid-1]+s[mid])

@dataclass
class BaseState:
    mode: str            # "dynamic" | "static"
    locked: bool         # True if static mode has locked
    lat: float
    lon: float
    alt: float
    sd_lat_m: float = 0.0
    sd_lon_m: float = 0.0
    fix_type: Optional[int] = None
    sats: Optional[int] = None

def compute_sd(values):
    if not values: return 0.0
    mean = sum(values) / len(values)
    var = sum((x-mean)**2 for x in values) / max(1, (len(values)-1))
    return math.sqrt(var)

def stabilize_base(base_buf: GpsBuffer,
                   window_sec: float,
                   sd_thresh_m: float,
                   min_samples: int = 15) -> Optional[BaseState]:
    """
    Wait until base GPS stabilizes over window_sec with both lat/lon stddev < sd_thresh_m,
    then return the mean position as locked BaseState.
    """
    t_end = time.time() + max(5.0, window_sec)  # never wait <5s
    while time.time() < t_end:
        last, snap = base_buf.snapshot()
        if len(snap) >= min_samples:
            # Filter by time window
            now = time.time()
            win = [s for s in snap if now - s.t <= window_sec]
            if len(win) >= min_samples:
                ref_lat = win[-1].lat
                lat_m = []
                lon_m = []
                for s in win:
                    lm, Lm = latlon_to_meters(s.lat - ref_lat, s.lon - win[-1].lon, ref_lat)
                    lat_m.append(lm)
                    lon_m.append(Lm)
                sd_lat = compute_sd(lat_m)
                sd_lon = compute_sd(lon_m)
                if sd_lat < sd_thresh_m and sd_lon < sd_thresh_m:
                    # lock to means (use arithmetic mean of degrees)
                    mean_lat = sum(s.lat for s in win)/len(win)
                    mean_lon = sum(s.lon for s in win)/len(win)
                    mean_alt = sum(s.alt for s in win)/len(win)
                    return BaseState(
                        mode="static", locked=True,
                        lat=mean_lat, lon=mean_lon, alt=mean_alt,
                        sd_lat_m=sd_lat, sd_lon_m=sd_lon,
                        fix_type=win[-1].fix_type, sats=win[-1].sats
                    )
        time.sleep(0.2)
    return None

def dynamic_base_filtered(base_buf: GpsBuffer,
                          window_sec: float,
                          min_samples: int = 5) -> Optional[BaseState]:
    last, snap = base_buf.snapshot()
    if not snap: return None
    now = time.time()
    win = [s for s in snap if now - s.t <= window_sec]
    if not win:
        return None
    ref_lat = win[-1].lat
    lat_vals = [s.lat for s in win]
    lon_vals = [s.lon for s in win]
    alt_vals = [s.alt for s in win]
    lat = rolling_median(lat_vals)
    lon = rolling_median(lon_vals)
    alt = rolling_median(alt_vals)
    # compute SD in meters for logging insight
    lat_m = [latlon_to_meters(s - ref_lat, 0.0, ref_lat)[0] for s in lat_vals]
    lon_m = [latlon_to_meters(0.0, s - lon_vals[-1], ref_lat)[1] for s in lon_vals]
    return BaseState(
        mode="dynamic", locked=False,
        lat=lat, lon=lon, alt=alt,
        sd_lat_m=compute_sd(lat_m), sd_lon_m=compute_sd(lon_m),
        fix_type=win[-1].fix_type, sats=win[-1].sats
    )

# ============= Main =============

def main():
    ap = argparse.ArgumentParser(description="Unified antenna tracker with static/dynamic base modes")
    ap.add_argument("--mode", choices=["sim","ground"], default=MODE_DEFAULT,
                    help="Run mode: SITL 'sim' or hardware 'ground' (default: ground)")

    ap.add_argument("--base-mode", choices=["dynamic","static"], default=BASE_MODE_DEFAULT,
                    help="Base position mode for 'ground': dynamic (filtered live) or static (auto-lock)")

    ap.add_argument("--static-window-sec", type=float, default=12.0,
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

    args = ap.parse_args()

    print("=== geo_8 unified (sim/ground) with base dynamic/static, zero-ref + smoothing ===")
    print(f"[CFG] Mode: {args.mode} | BaseMode: {args.base_mode} | Gear {GEAR_RATIO}:1 | Servo 180° @ {PULSE_MIN_US}-{PULSE_MAX_US}µs")
    print(f"[CFG] update_period={args.update_period:.2f}s, print_period={args.print_period:.1f}s, alpha={args.alpha}")

    pi = setup_pigpio()
    log_open(prefix="Tracker")

    # Prepare buffers and readers
    drone_buf = GpsBuffer("drone", maxlen=300)
    base_buf  = GpsBuffer("base",  maxlen=300)

    if args.mode == "sim":
        mav_drone = connect_mav(SIM_DRONE_ENDPOINT, SIM_DRONE_BAUD, True)
        start_reader(mav_drone, drone_buf, on_raw=log_raw if LOG_RAW_GPS else None)
        print(f"[CFG] SIM base @ {base_static['lat']}, {base_static['lon']}, {base_static['alt']}m")
    else:
        mav_drone = connect_mav(DRONE_ENDPOINT, DRONE_BAUD, True)
        mav_base  = connect_mav(BASE_ENDPOINT,  BASE_BAUD,  False)
        start_reader(mav_drone, drone_buf, on_raw=log_raw if LOG_RAW_GPS else None)
        start_reader(mav_base,  base_buf,  on_raw=log_raw if LOG_RAW_GPS else None)

    # ===== Zero-reference setup =====
    zero_world_az = None
    zero_world_el = None
    print("[INFO] Point the tracker at the drone and stabilize GPS.")
    print("[INFO] Waiting 10 seconds for zero reference…")
    time.sleep(10.0)

    # We need one snapshot of both drone and base for zeroing
    def get_zero_snapshot():
        t_end = time.time() + 5.0
        while time.time() < t_end:
            d = drone_buf.latest()
            if args.mode == "sim":
                b = GpsSample(time.time(), base_static["lat"], base_static["lon"], base_static["alt"])
            else:
                b = base_buf.latest()
            if d and b:
                return d, b
            time.sleep(0.2)
        return None, None

    d0, b0 = get_zero_snapshot()
    if d0 and b0:
        info0 = tracker.get_tracking_info(b0.lat, b0.lon, b0.alt, d0.lat, d0.lon, d0.alt)
        if info0:
            zero_world_az = info0.get('adjusted_azimuth', info0['azimuth'])
            zero_world_el = info0.get('adjusted_elevation', info0['elevation'])
            print(f"[INFO] Zero ref set: AZ={zero_world_az:.2f}°, EL={zero_world_el:.2f}°")
        else:
            print("[WARN] tracker.get_tracking_info failed for zeroing.")
    else:
        print("[WARN] Could not obtain initial GPS snapshot for zeroing. Proceeding with zero=(0,0).")

    # Park at servo (0,0) to start
    print("[INFO] Parking the tracker to initial position.")
    pi.set_servo_pulsewidth(SERVO_AZ_PIN, servo_deg_to_us(0.0))
    pi.set_servo_pulsewidth(SERVO_EL_PIN, servo_deg_to_us(0.0))
    time.sleep(2.0)

    # ===== Base mode management (GROUND only) =====
    base_state: Optional[BaseState] = None
    if args.mode == "ground":
        if args.base_mode == "static":
            print(f"[INFO] Static base mode: waiting up to {args.static_window_sec:.1f}s for stabilization (sd<{args.static_sd_thresh_m:.2f}m)…")
            bs = stabilize_base(base_buf, window_sec=args.static_window_sec, sd_thresh_m=args.static_sd_thresh_m)
            if bs:
                base_state = bs
                print(f"[INFO] Base locked: lat={bs.lat:.7f} lon={bs.lon:.7f} alt={bs.alt:.2f}  (sd≈{bs.sd_lat_m:.2f}/{bs.sd_lon_m:.2f} m)")
            else:
                print("[WARN] Base did not stabilize in time; falling back to dynamic filtered base.")
                args.base_mode = "dynamic"

    # Prepare smoothing variables and print pacing
    smoothed_az = None
    smoothed_el = None
    alpha = max(0.0, min(1.0, args.alpha))
    next_print = time.time()

    try:
        while True:
            # snapshot drone
            d = drone_buf.latest()
            if not d:
                time.sleep(0.05); continue

            # compute base according to mode
            if args.mode == "sim":
                b_lat, b_lon, b_alt = base_static["lat"], base_static["lon"], base_static["alt"]
                base_locked = True
                base_sd_lat = base_sd_lon = 0.0
                base_fix = base_sats = None
                base_mode_str = "static(SIM)"
            else:
                if base_state and base_state.locked:
                    b_lat, b_lon, b_alt = base_state.lat, base_state.lon, base_state.alt
                    base_locked = True
                    base_sd_lat, base_sd_lon = base_state.sd_lat_m, base_state.sd_lon_m
                    base_fix, base_sats = base_state.fix_type, base_state.sats
                    base_mode_str = "static"
                else:
                    bs = dynamic_base_filtered(base_buf, window_sec=args.dynamic_window_sec)
                    if not bs:
                        time.sleep(0.05); continue
                    b_lat, b_lon, b_alt = bs.lat, bs.lon, bs.alt
                    base_locked = False
                    base_sd_lat, base_sd_lon = bs.sd_lat_m, bs.sd_lon_m
                    base_fix, base_sats = bs.fix_type, bs.sats
                    base_mode_str = "dynamic"

            # Compute angles
            info = tracker.get_tracking_info(b_lat, b_lon, b_alt, d.lat, d.lon, d.alt)
            if not info:
                time.sleep(args.update_period); continue

            abs_az = info.get('adjusted_azimuth', info['azimuth'])
            abs_el = info.get('adjusted_elevation', info['elevation'])
            world_az = norm360(abs_az - (zero_world_az or 0.0))
            world_el = abs_el - (zero_world_el or 0.0)

            # Low-pass smoothing of world az/el
            if smoothed_az is None:
                smoothed_az, smoothed_el = world_az, world_el
            else:
                smoothed_az = (1-alpha)*smoothed_az + alpha*world_az
                smoothed_el = (1-alpha)*smoothed_el + alpha*world_el

            # Calibration → clamp → gearing → pulse
            cal_az, cal_el   = apply_calibration(smoothed_az, smoothed_el)
            phys_az, phys_el = world_to_physical(cal_az, cal_el)
            s_az, s_el       = physical_to_servo_deg(phys_az, phys_el)
            us_az            = servo_deg_to_us(s_az)
            us_el            = servo_deg_to_us(s_el)

            # Drive servos
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)

            # Console output (paced)
            if time.time() >= next_print:
                print(f"[{datetime.now():%H:%M:%S}] Base={base_mode_str}{'GPS locked' if base_locked else ''} "
                      f"sd≈{base_sd_lat:.2f}/{base_sd_lon:.2f}m | "
                      f"WORLD {smoothed_az:6.2f}/{smoothed_el:5.2f}° | "
                      f"SERVO {s_az:6.2f}/{s_el:5.2f}° | µs {us_az:5.0f}/{us_el:5.0f}")
                next_print = time.time() + max(0.2, args.print_period)

            # CSV log (padded with diagnostics)
            log_row(
                datetime.now().isoformat(timespec='seconds'),
                base_mode_str, 1 if base_locked else 0,
                round(smoothed_az,3), round(smoothed_el,3),
                round(cal_az,3), round(cal_el,3),
                round(phys_az,3), round(phys_el,3),
                round(s_az,3), round(s_el,3),
                round(us_az,1), round(us_el,1),
                round(d.lat,7), round(d.lon,7), round(d.alt,2),
                round(b_lat,7), round(b_lon,7), round(b_alt,2),
                round(base_sd_lat,3), round(base_sd_lon,3),
                "" if base_fix is None else base_fix,
                "" if base_sats is None else base_sats
            )

            time.sleep(args.update_period)

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

if __name__ == "__main__":
    main()
