# Removed Smoothing filter to get correct spikes

import argparse
import time
from dataclasses import dataclass
from datetime import datetime
import threading
from typing import Optional, Tuple, Callable

import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker

# ===================== Defaults (overridable by CLI) =====================

MODE_DEFAULT = "ground"         # "sim" or "ground"
BASE_MODE_DEFAULT = "static"   # "dynamic" or "static"   (only used in ground mode)

# --- Endpoints ---
SIM_DRONE_ENDPOINT = "udp:0.0.0.0:14550"
SIM_DRONE_BAUD     = None
DRONE_ENDPOINT     = "/dev/ttyACM0"
DRONE_BAUD         = 57600
BASE_ENDPOINT      = "/dev/ttyUSB0"
BASE_BAUD          = 57600

# MAVLink stream requests (~5 Hz)
MAV_MSG_INTERVAL_US_GPS    = 200_000
MAV_MSG_INTERVAL_US_GLOBAL = 50_000

# Gear spokes ratios
SPOKES_SMALL   = 12
SPOKES_BIG     = 24
AZ_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL    # default 2.0
EL_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL    # default 2.0

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
ELEVATION_ZERO_OFFSET_DEG = 90.0
AZIMUTH_INVERT   = True
ELEVATION_INVERT = False  # set True if your rig needs "up is up"

# pigpio GPIO pins (BCM numbering)
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 17  # per your hardware

# Loop timings
UPDATE_PERIOD_S = 0.05   # 10 Hz default (set higher if your servos prefer slower updates)
PRINT_PERIOD_S  = 1.5
LOG_TO_CSV      = True
LOG_RAW_GPS     = True   # per-message GPS capture (for precision analysis)

EL_MIN_WORLD_DEG = 0.0
EL_MAX_WORLD_DEG = 90.0

# SIM base (used only in SIM mode)
base_static = {
    "lat": 13.0281865,
    "lon": 77.5675790,
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
    az_raw = az_phys / AZ_GEAR_RATIO
    el_raw = el_phys / EL_GEAR_RATIO
    az_servo = max(0.0, min(SERVO_RANGE_DEG, az_raw))
    el_servo = max(0.0, min(SERVO_RANGE_DEG, el_raw))
    return az_servo, el_servo

def servo_deg_to_us(deg: float) -> float:
    deg = max(0.0, min(SERVO_RANGE_DEG, deg))
    return PULSE_MIN_US + (deg / SERVO_RANGE_DEG) * (PULSE_MAX_US - PULSE_MIN_US)

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

def _extract_sample(msg) -> Optional[GpsSample]:  #Optional[]-> Might not might not return a value
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

# ============= pigpio & Smooth Parking =============
def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running (sudo pigpiod)")
    pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
    print(f"[INFO] pigpio ready (AZ={SERVO_AZ_PIN}, EL={SERVO_EL_PIN})")
    return pi

def _bounded_us(x: float) -> float:
    return max(500.0, min(2500.0, x))

def _get_start_us(pi, pin: int, default_us: float = 1500.0) -> float:
    try:
        val = float(pi.get_servo_pulsewidth(pin))
        if 500.0 <= val <= 2500.0:
            return val
    except Exception:
        pass
    return default_us

def smooth_park(pi, target_servo_deg_az: float, target_servo_deg_el: float,
                duration_s: float = 1.5, rate_hz: float = 60.0):
    """Ramp both servos smoothly to target servo degrees over duration_s."""
    target_us_az = servo_deg_to_us(max(0.0, min(SERVO_RANGE_DEG, target_servo_deg_az)))
    target_us_el = servo_deg_to_us(max(0.0, min(SERVO_RANGE_DEG, target_servo_deg_el)))

    start_us_az = _get_start_us(pi, SERVO_AZ_PIN, default_us=1500.0)
    start_us_el = _get_start_us(pi, SERVO_EL_PIN, default_us=1500.0)

    steps = max(1, int(duration_s * rate_hz))
    for i in range(1, steps + 1):
        a = i / steps
        us_az = _bounded_us(start_us_az + (target_us_az - start_us_az) * a)
        us_el = _bounded_us(start_us_el + (target_us_el - start_us_el) * a)
        pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
        pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
        time.sleep(1.0 / rate_hz)

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

def start_reader(mav: mavutil.mavfile, stream_name: str, on_raw: Optional[Callable[[str,GpsSample],None]] = None):
    """
    Read GPS messages from `mav` and publish only the most recent sample
    into the global latest-store (no buffering).
    stream_name: "DRONE" or "BASE" (or anything you like for logging)
    """
    def _run():
        while True:
            msg = mav.recv_match(type=["GPS_RAW_INT","GLOBAL_POSITION_INT"], blocking=True, timeout=1.5)
            if not msg:
                continue
            s = _extract_sample(msg)
            if not s:
                continue

            # route to latest store
            if stream_name.upper() == "BASE":
                set_latest_base(s)
            else:
                set_latest_drone(s)

            # optional raw log
            if on_raw:
                on_raw(stream_name, s)

    th = threading.Thread(target=_run, daemon=True)
    th.start()
    return th

# ============= CSV logging =============
_log_writer = None
_log_file   = None
_raw_writer = None
_raw_file   = None

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

def log_row(*row):
    if _log_writer:
        _log_writer.writerow(row)
        if _log_file:
            _log_file.flush()

def log_raw(stream: str, s: GpsSample):
    if _raw_writer:
        _raw_writer.writerow([
            datetime.now().isoformat(timespec='seconds'),
            stream, f"{s.lat:.7f}", f"{s.lon:.7f}", f"{s.alt:.2f}",
            "" if s.eph is None else f"{s.eph:.2f}",
            "" if s.epv is None else f"{s.epv:.2f}",
            "" if s.fix_type is None else s.fix_type,
            "" if s.sats is None else s.sats
        ])
        if _raw_file:
            _raw_file.flush()

def log_close():
    if _log_file: _log_file.close()
    if _raw_file: _raw_file.close()

# ============= Latest-sample stores (thread-safe) ============= #

_latest_base    = None
_latest_drone   = None
_base_lock      = threading.Lock()
_drone_lock     = threading.Lock()

def set_latest_base(sample):  # sample: GpsSample or similar
    global _latest_base
    with _base_lock:
        _latest_base = sample

def set_latest_drone(sample):  # sample: GpsSample or similar
    global _latest_drone
    with _drone_lock:
        _latest_drone = sample

def get_latest_base():
    with _base_lock:
        return _latest_base

def get_latest_drone():
    with _drone_lock:
        return _latest_drone

# ============= Main =============

def main():
    ap = argparse.ArgumentParser(description="Unified antenna tracker with static/dynamic base modes + smooth parking")
    ap.add_argument("--mode", choices=["sim","ground"], default=MODE_DEFAULT,
                    help="Run mode: SITL 'sim' or hardware 'ground' (default: ground)")

    ap.add_argument("--print-period", type=float, default=PRINT_PERIOD_S,
                    help="Seconds between console prints")

    ap.add_argument("--update-period", type=float, default=UPDATE_PERIOD_S,
                    help="Main loop period seconds (servo update rate)")

    ap.add_argument("--base-mode", choices=["static","dynamic"], default=BASE_MODE_DEFAULT,
                help="Base position mode for 'ground': static (freeze after 10s) or dynamic (always latest)")

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
                    help="Override servo min pulse (us), e.g., 900")
    ap.add_argument("--servo-max-us", dest="servo_max_us", type=float, default=None,
                    help="Override servo max pulse (us), e.g., 1200")

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

    print("=== geo_10 (sim/ground) with base dynamic/static, zero-ref + smooth parking ===")
    print(f"[CFG] Mode: {args.mode} | BaseMode: {args.base_mode} | "
          f"Gear AZ {AZ_GEAR_RATIO}:1, EL {EL_GEAR_RATIO}:1 | "
          f"Servo 180° @ {PULSE_MIN_US}-{PULSE_MAX_US}us")
    print(f"[CFG] update_period={args.update_period:.2f}s, print_period={args.print_period:.1f}s")
    print(f"[CFG] parking: home=({args.park_home_az:.1f}°, {args.park_home_el:.1f}°) "
          f"face_drone(start={args.park_face_drone_start}, exit={args.park_face_drone_exit}) "
          f"duration={args.park_duration:.2f}s @ {args.park_rate_hz:.0f} Hz")

    pi = setup_pigpio()
    log_open(prefix="Tracker")

    # Start readers (latest-sample only; no buffers)
    if args.mode == "sim":
        mav_drone = connect_mav(SIM_DRONE_ENDPOINT, SIM_DRONE_BAUD, True)
        start_reader(mav_drone, "DRONE", on_raw=log_raw if LOG_RAW_GPS else None)
        print(f"[CFG] SIM base @ {base_static['lat']}, {base_static['lon']}, {base_static['alt']}m")
    else:
        mav_drone = connect_mav(DRONE_ENDPOINT, DRONE_BAUD, True)
        mav_base  = connect_mav(BASE_ENDPOINT,  BASE_BAUD,  False)
        start_reader(mav_drone, "DRONE", on_raw=log_raw if LOG_RAW_GPS else None)
        start_reader(mav_base,  "BASE",  on_raw=log_raw if LOG_RAW_GPS else None)

    # ----- Base mode handling (ground only) -----
    base_fixed = None  # (lat, lon, alt) when static
    if args.mode == "ground":
        if args.base_mode == "static":
            print("[INFO] Base mode=static → waiting 10s, then freezing base GPS.")
            t_end = time.time() + 10.0
            while time.time() < t_end:
                b_try = get_latest_base()
                if b_try:
                    base_fixed = (b_try.lat, b_try.lon, b_try.alt)
                time.sleep(0.1)
            if base_fixed is None:
                print("[WARN] No base GPS received during static lock window; will freeze on the first base sample in the loop.")
            else:
                print(f"[INFO] Base frozen to lat = {base_fixed[0]:.7f}, lon={base_fixed[1]:.7f}, alt={base_fixed[2]:.2f}")
        else:
            print("[INFO] Base mode = dynamic - always use latest base GPS.")

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
            d = get_latest_drone()
            if args.mode == "sim":
                b = GpsSample(time.time(), base_static["lat"], base_static["lon"], base_static["alt"])
            else:
                if args.base_mode == "static" and base_fixed is not None:
                    b = GpsSample(time.time(), base_fixed[0], base_fixed[1], base_fixed[2])
                else:
                    b = get_latest_base()
            if d and b:
                return d, b
            time.sleep(0.1)
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

    # Smoothly park to initial position
    print("[INFO] Smoothly parking the tracker to initial position…")
    if args.park_face_drone_start and d0 and b0:
        info_init = tracker.get_tracking_info(b0.lat, b0.lon, b0.alt, d0.lat, d0.lon, d0.alt)
        if info_init:
            abs_az0 = info_init.get("adjusted_azimuth", info_init["azimuth"])
            # EL = home (e.g., horizon)
            world_az0, world_el0 = norm360(abs_az0), args.park_home_el
            cal_az0, cal_el0 = apply_calibration(world_az0, world_el0)
            phys_az0, phys_el0 = world_to_physical(cal_az0, cal_el0)
            s_az0, s_el0 = physical_to_servo_deg(phys_az0, phys_el0)
            smooth_park(pi, s_az0, s_el0, duration_s=args.park_duration, rate_hz=args.park_rate_hz)
    else:
        cal_az0, cal_el0 = apply_calibration(args.park_home_az, args.park_home_el)
        phys_az0, phys_el0 = world_to_physical(cal_az0, cal_el0)
        s_az0, s_el0 = physical_to_servo_deg(phys_az0, phys_el0)
        smooth_park(pi, s_az0, s_el0, duration_s=args.park_duration, rate_hz=args.park_rate_hz)

    # Prepare smoothing variables and print pacing
    next_print = time.time()
    last_servo_az = 0.0
    last_servo_el = 0.0

    try:
        while True:
            # --- DRONE (latest) ---
            d = get_latest_drone()
            if not d:
                time.sleep(0.01); continue

            # --- BASE (by mode) ---
            if args.mode == "sim":
                b_lat, b_lon, b_alt = base_static["lat"], base_static["lon"], base_static["alt"]
                base_mode_str = "static(SIM)"
                base_fix = base_sats = None
            else:
                if args.base_mode == "static":
                    # If we didn't get one earlier, block until we have at least one sample
                    if base_fixed is None:
                        b_now = get_latest_base()
                        if not b_now:
                            time.sleep(0.01); continue
                        base_fixed = (b_now.lat, b_now.lon, b_now.alt)
                        print(f"[INFO] Base frozen late to lat={base_fixed[0]:.7f}, lon={base_fixed[1]:.7f}, alt={base_fixed[2]:.2f}")
                    b_lat, b_lon, b_alt = base_fixed
                    base_mode_str = "static"
                    base_fix = base_sats = None
                else:  # dynamic
                    b = get_latest_base()
                    if not b:
                        time.sleep(0.01); continue
                    b_lat, b_lon, b_alt = b.lat, b.lon, b.alt
                    base_mode_str = "dynamic"
                    base_fix = getattr(b, "fix_type", None)
                    base_sats = getattr(b, "sats", None)

            # Diagnostics (we don't compute SDs anymore)
            base_locked = (args.mode == "sim") or (args.base_mode == "static")
            base_sd_lat = base_sd_lon = 0.0

            # --- Angles (NO filtering) ---
            info = tracker.get_tracking_info(b_lat, b_lon, b_alt, d.lat, d.lon, d.alt)
            if not info:
                time.sleep(args.update_period); continue

            abs_az = info.get('adjusted_azimuth', info['azimuth'])
            abs_el = info.get('adjusted_elevation', info['elevation'])
            world_az = norm360(abs_az - (zero_world_az or 0.0))
            world_el = abs_el - (zero_world_el or 0.0)

            # Calibration → clamp → gearing → pulse (direct; no smoothing)
            cal_az, cal_el   = apply_calibration(world_az, world_el)
            phys_az, phys_el = world_to_physical(cal_az, cal_el)
            s_az, s_el       = physical_to_servo_deg(phys_az, phys_el)
            us_az            = servo_deg_to_us(s_az)
            us_el            = servo_deg_to_us(s_el)

            # Drive servos
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
            last_servo_az, last_servo_el = s_az, s_el

            # Console output (paced)
            if time.time() >= next_print:
                print(f"[{datetime.now():%H:%M:%S}] Base={base_mode_str} "
                    f"WORLD {world_az:6.2f}/{world_el:5.2f}° | "
                    f"SERVO {s_az:6.2f}/{s_el:5.2f}° | us {us_az:5.0f}/{us_el:5.0f}")
                next_print = time.time() + max(0.2, args.print_period)

            # CSV log (padded with diagnostics)
            log_row(
                    datetime.now().isoformat(timespec='seconds'),
                    base_mode_str, 1 if base_locked else 0,
                    round(world_az,3), round(world_el,3),
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
        print("[INFO] Smooth shutdown: parking…")
        try:
            if args.park_face_drone_exit:
                # Keep current AZ (last_servo_az), ramp EL → home
                cal_azH, cal_elH = apply_calibration(0.0, args.park_home_el)
                phys_azH, phys_elH = world_to_physical(cal_azH, cal_elH)
                s_azH = last_servo_az
                s_elH = physical_to_servo_deg(phys_azH, phys_elH)[1]
                smooth_park(pi, s_azH, s_elH, duration_s=args.park_duration, rate_hz=args.park_rate_hz)
            else:
                cal_azH, cal_elH = apply_calibration(args.park_home_az, args.park_home_el)
                phys_azH, phys_elH = world_to_physical(cal_azH, cal_elH)
                s_azH, s_elH = physical_to_servo_deg(phys_azH, phys_elH)
                smooth_park(pi, s_azH, s_elH, duration_s=args.park_duration, rate_hz=args.park_rate_hz)

            time.sleep(0.2)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, 0)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, 0)
            pi.stop()
        except Exception as e:
            print(f"[WARN] pigpio cleanup: {e}")
        log_close()

if __name__ == "__main__":
    main()
