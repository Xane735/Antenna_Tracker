# geo_11.py — zero-filter tracker with static/dynamic base modes

import argparse
import time
from dataclasses import dataclass
from datetime import datetime
import threading
from typing import Optional, Tuple, Callable

import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker

# ===================== Defaults =====================

MODE_DEFAULT = "ground"          # "sim" or "ground"
BASE_MODE_DEFAULT = "static"     # "static" (freeze after 10s) or "dynamic"

# --- Endpoints ---
SIM_DRONE_ENDPOINT = "udp:0.0.0.0:14550"
SIM_DRONE_BAUD     = None
DRONE_ENDPOINT     = "/dev/ttyUSB0"
DRONE_BAUD         = 57600
BASE_ENDPOINT      = "/dev/ttyACM0"
BASE_BAUD          = 57600

# MAVLink stream requests (adjust if you want faster UI response)
MAV_MSG_INTERVAL_US_GPS    = 200_000   # ~5 Hz (typical GPS)
MAV_MSG_INTERVAL_US_GLOBAL = 50_000   # ~5 Hz (raise to 50_000 for ~20 Hz if supported)

# Gear spokes ratios
SPOKES_SMALL   = 12
SPOKES_BIG     = 24
AZ_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL    # default 2.0
EL_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL    # default 2.0

# Physical limits
AZ_PHYS_MIN = 0.0
AZ_PHYS_MAX = 360.0
EL_PHYS_MIN = 0.0
EL_PHYS_MAX = 180.0

# Servo mapping — 180° servo
PULSE_MIN_US    = 900.0
PULSE_MAX_US    = 2100.0
SERVO_RANGE_DEG = 180.0

# Calibration
AZIMUTH_ZERO_OFFSET_DEG   = -30.0
ELEVATION_ZERO_OFFSET_DEG = 0.0
AZIMUTH_INVERT   = True
ELEVATION_INVERT = False

# pigpio GPIO pins (BCM)
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 17

# Loop timings
UPDATE_PERIOD_S = 0.09   # main loop period; try 0.02–0.05 for snappier updates
PRINT_PERIOD_S  = 1.0
LOG_TO_CSV      = True
LOG_RAW_GPS     = False # Make sure to remove once everything works. Most useless feature youve added *smh smh*

# SIM base (used only in SIM mode)
base_static = {"lat": 13.0276175, "lon": 77.5629830, "alt": 931.13}

# --- Tracking dynamics (snappy but safe) --- (Jump the values to 300-360, but can make it jerky or jumpy)
AZ_MAX_DEG_PER_SEC   = 240.0     # how fast the ANTENNA may rotate
EL_MAX_DEG_PER_SEC   = 240.0
# these convert to per-tick steps using your UPDATE_PERIOD_S

# --- Back-side flip logic (lets a 180° servo avoid wrap jumps) ---
ALLOW_BACKSIDE_FLIP = True      # set False if your antenna cannot be used backwards
FLIP_HYSTERESIS_DEG = 8.0       # require this servo-deg benefit to switch to the flipped pose | If you notice “flip thrash” when hovering near the seam, you can increase:


# ===================== Helpers =====================

def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

# --- Shortest-path wrap helpers ---
def wrap360(x: float) -> float:
    return (x + 360.0) % 360.0

def shortest_delta_deg(target: float, current: float) -> float:
    # returns signed delta in (−180, +180]
    return ((target - current + 540.0) % 360.0) - 180.0


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
    # guard types for static checkers
    mn = float(PULSE_MIN_US)
    mx = float(PULSE_MAX_US)
    rng = float(SERVO_RANGE_DEG)
    d = max(0.0, min(rng, float(deg)))
    return mn + (d / rng) * (mx - mn)

def choose_flipped_if_better(cal_az: float, cal_el: float,
                             last_saz: float, last_sel: float) -> Tuple[float, float, bool]:
    """
    From calibrated world angles, choose either:
      A) normal pointing  (az, el)
      B) backside pointing (az+180, 180-el)
    Return the chosen PHYSICAL angles (clamped) and a boolean used_flip.
    We pick the option that minimizes servo movement from last_saz/last_sel,
    with a small hysteresis so we don't thrash.
    """
    AZ_WEIGHT = 1.0
    EL_WEIGHT = 0.30        # 0.25–0.35 is a good range
    EDGE = 12.0             # servo-deg from 0/180 that arms the seam logic
    MIN_EL_FOR_FLIP = 3.0   # don’t flip if elevation is extremely close to 0/180

    # Candidate A: normal
    A_az, A_el = world_to_physical(cal_az, cal_el)
    A_saz, A_sel = physical_to_servo_deg(A_az, A_el)
    A_cost = abs(A_saz - last_saz) + abs(A_sel - last_sel)

    if not ALLOW_BACKSIDE_FLIP:
        return A_az, A_el, False

    # Candidate B: backside (mirror elevation, rotate az by 180)
    B_az = (A_az + 180.0) % 360.0
    B_el = max(EL_PHYS_MIN, min(EL_PHYS_MAX, 180.0 - A_el))
    B_saz, B_sel = physical_to_servo_deg(B_az, B_el)
    B_cost = abs(B_saz - last_saz) + abs(B_sel - last_sel)

    A_cost = AZ_WEIGHT * abs(A_saz - last_saz) + EL_WEIGHT * abs(A_sel - last_sel)
    B_cost = AZ_WEIGHT * abs(B_saz - last_saz) + EL_WEIGHT * abs(B_sel - last_sel)

    # Arm seam logic only near edges and if the normal candidate would "cross" to the opposite edge
    near_edge = (last_saz < EDGE) or (last_saz > 180.0 - EDGE)
    crosses_edge = (last_saz < EDGE and A_saz > 180.0 - EDGE) or (last_saz > 180.0 - EDGE and A_saz < EDGE)

    # Elevation safety: avoid flips at extreme el unless you KNOW it’s safe mechanically
    el_ok = (A_el >= MIN_EL_FOR_FLIP) and (A_el <= 180.0 - MIN_EL_FOR_FLIP)

    # Seam override: if we're about to teleport the az servo, prefer the back-side even if margin not met
    if near_edge and crosses_edge and el_ok:
        # Either B is already better, or allow a small "assist" by ignoring hysteresis here
        if (B_cost <= A_cost) or (B_cost + FLIP_HYSTERESIS_DEG <= A_cost):
            return B_az, B_el, True

    # Normal hysteresis (sticky) behavior elsewhere
    if B_cost + FLIP_HYSTERESIS_DEG < A_cost:
        return B_az, B_el, True
    else:
        return A_az, A_el, False

# ===================== Thread-safe latest GPS =====================

@dataclass
class GpsSample:
    t: float
    lat: float
    lon: float
    alt: float
    eph: Optional[float] = None
    epv: Optional[float] = None
    fix_type: Optional[int] = None
    sats: Optional[int] = None

def _extract_sample(msg) -> Optional[GpsSample]:
    t = time.time()
    tp = msg.get_type()
    if tp == "GPS_RAW_INT":
        eph = getattr(msg, "eph", None)
        epv = getattr(msg, "epv", None)
        def to_m(x):
            if x is None: return None
            return x / 1000.0 if x > 1000 else x / 100.0
        return GpsSample(
            t=t,
            lat=msg.lat/1e7, lon=msg.lon/1e7, alt=msg.alt/1000.0,
            eph=to_m(eph), epv=to_m(epv),
            fix_type=getattr(msg,"fix_type",None),
            sats=getattr(msg,"satellites_visible",None)
        )
    if tp == "GLOBAL_POSITION_INT":
        return GpsSample(t=t, lat=msg.lat/1e7, lon=msg.lon/1e7, alt=msg.alt/1000.0)
    return None

_latest_base: Optional[GpsSample] = None
_latest_drone: Optional[GpsSample] = None
_base_lock = threading.Lock()
_drone_lock = threading.Lock()

def set_latest_base(sample: GpsSample):
    global _latest_base
    with _base_lock:
        _latest_base = sample

def set_latest_drone(sample: GpsSample):
    global _latest_drone
    with _drone_lock:
        _latest_drone = sample

def get_latest_base() -> Optional[GpsSample]:
    with _base_lock:
        return _latest_base

def get_latest_drone() -> Optional[GpsSample]:
    with _drone_lock:
        return _latest_drone

# ===================== pigpio & parking =====================

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
    """Ramp both servos smoothly to target servo degrees over duration_s, without the initial twitch."""
    # Clamp to valid servo range
    t_az_deg = max(0.0, min(SERVO_RANGE_DEG, float(target_servo_deg_az)))
    t_el_deg = max(0.0, min(SERVO_RANGE_DEG, float(target_servo_deg_el)))

    target_us_az = servo_deg_to_us(t_az_deg)
    target_us_el = servo_deg_to_us(t_el_deg)

    # IMPORTANT: if pigpio returns 0 (unknown), assume we're already at TARGET to avoid jumping to 1500 µs first.
    start_us_az = _get_start_us(pi, SERVO_AZ_PIN, default_us=target_us_az)
    start_us_el = _get_start_us(pi, SERVO_EL_PIN, default_us=target_us_el)

    # Small deadband to suppress one-tick flicker
    DEAD_US = 8.0
    d_az = abs(target_us_az - start_us_az)
    d_el = abs(target_us_el - start_us_el)

    # If we're already "there", just enable PWM at target and exit
    if d_az <= DEAD_US and d_el <= DEAD_US:
        pi.set_servo_pulsewidth(SERVO_AZ_PIN, target_us_az)
        pi.set_servo_pulsewidth(SERVO_EL_PIN, target_us_el)
        return

    # Prime outputs to the measured start (turn PWM on cleanly), then ramp
    pi.set_servo_pulsewidth(SERVO_AZ_PIN, start_us_az)
    pi.set_servo_pulsewidth(SERVO_EL_PIN, start_us_el)
    time.sleep(0.02)

    steps = max(1, int(float(duration_s) * float(rate_hz)))
    for i in range(1, steps + 1):
        a = i / steps
        us_az = _bounded_us(start_us_az + (target_us_az - start_us_az) * a)
        us_el = _bounded_us(start_us_el + (target_us_el - start_us_el) * a)
        pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
        pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
        time.sleep(1.0 / rate_hz)

# ===================== MAVLink I/O =====================

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
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,        MAV_MSG_INTERVAL_US_GPS),
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

def start_reader(mav: mavutil.mavfile, stream_name: str,
                 on_raw: Optional[Callable[[str,GpsSample],None]] = None):
    def _run():
        while True:
            msg = mav.recv_match(type=["GPS_RAW_INT","GLOBAL_POSITION_INT"], blocking=True, timeout=1.5)
            if not msg:
                continue
            s = _extract_sample(msg)
            if not s:
                continue
            if stream_name.upper() == "BASE":
                set_latest_base(s)
            else:
                set_latest_drone(s)
            if on_raw:
                on_raw(stream_name, s)
    th = threading.Thread(target=_run, daemon=True)
    th.start()
    return th

# ===================== CSV logging =====================

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
            "BaseLatSDm","BaseLonSDm","BaseFix","BaseSats",
            "UsedFlip"])

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
        _raw_writer.writerow([datetime.now().isoformat(timespec='seconds'),
                              stream, f"{s.lat:.7f}", f"{s.lon:.7f}", f"{s.alt:.2f}",
                              "" if s.eph is None else f"{s.eph:.2f}",
                              "" if s.epv is None else f"{s.epv:.2f}",
                              "" if s.fix_type is None else s.fix_type,
                              "" if s.sats is None else s.sats])
        if _raw_file:
            _raw_file.flush()

def log_close():
    if _log_file: _log_file.close()
    if _raw_file: _raw_file.close()

# ===================== Main =====================

def main():
    ap = argparse.ArgumentParser(description="Antenna tracker with static/dynamic base, zero-ref + smooth parking (no filtering)")
    ap.add_argument("--mode", choices=["sim","ground"], default=MODE_DEFAULT,
                    help="SITL 'sim' or hardware 'ground'")
    ap.add_argument("--base-mode", choices=["static","dynamic"], default=BASE_MODE_DEFAULT,
                    help="Base position: static=freeze after window; dynamic=always latest")
    ap.add_argument("--static-window-sec", type=float, default=10.0,
                    help="Seconds to sample base before freezing (static mode only)")
    ap.add_argument("--print-period", type=float, default=PRINT_PERIOD_S,
                    help="Seconds between console prints")
    ap.add_argument("--update-period", type=float, default=UPDATE_PERIOD_S,
                    help="Main loop period seconds (servo update rate)")

    # Parking & overrides
    ap.add_argument("--park-home-az", type=float, default=0.0)
    ap.add_argument("--park-home-el", type=float, default=90.0)
    ap.add_argument("--park-face-drone-start", action="store_true")
    ap.add_argument("--park-face-drone-exit", action="store_true")
    ap.add_argument("--park-duration", type=float, default=1.5)
    ap.add_argument("--park-rate-hz", type=float, default=60.0)
    ap.add_argument("--az-gear-ratio", dest="az_gear_ratio", type=float, default=None)
    ap.add_argument("--el-gear-ratio", dest="el_gear_ratio", type=float, default=None)
    ap.add_argument("--servo-min-us", dest="servo_min_us", type=float, default=None)
    ap.add_argument("--servo-max-us", dest="servo_max_us", type=float, default=None)

    args = ap.parse_args()

    # Apply optional overrides
    global AZ_GEAR_RATIO, EL_GEAR_RATIO, PULSE_MIN_US, PULSE_MAX_US
    if args.az_gear_ratio is not None: AZ_GEAR_RATIO = float(args.az_gear_ratio)
    if args.el_gear_ratio is not None: EL_GEAR_RATIO = float(args.el_gear_ratio)
    if args.servo_min_us  is not None: PULSE_MIN_US  = float(args.servo_min_us)
    if args.servo_max_us  is not None: PULSE_MAX_US  = float(args.servo_max_us)
    AZ_GEAR_RATIO = float(AZ_GEAR_RATIO)
    EL_GEAR_RATIO = float(EL_GEAR_RATIO)
    PULSE_MIN_US  = float(PULSE_MIN_US)
    PULSE_MAX_US  = float(PULSE_MAX_US)

    print("=== geo_10 (sim/ground) — static/dynamic base, zero-ref, smooth parking — NO FILTERING ===")
    print(f"[CFG] Mode: {args.mode} | BaseMode: {args.base_mode} | Gear AZ {AZ_GEAR_RATIO}:1, EL {EL_GEAR_RATIO}:1 | Servo 180° @ {PULSE_MIN_US}-{PULSE_MAX_US}µs")
    print(f"[CFG] update_period={args.update_period:.2f}s, print_period={args.print_period:.1f}s")
    print(f"[CFG] parking: home=({args.park_home_az:.1f}°, {args.park_home_el:.1f}°) face_drone(start={args.park_face_drone_start}, exit={args.park_face_drone_exit}) duration={args.park_duration:.2f}s @ {args.park_rate_hz:.0f} Hz")

    pi = setup_pigpio()
    log_open(prefix="Tracker")

    # --- Initial parking happens BEFORE MAV readers start ---
    print("[INFO] Parking to home based on fixed zero (no learned zero).")

    # Park to world (0°, home-EL); this is your logical zero
    cal_az0, cal_el0 = apply_calibration(0.0, args.park_home_el)
    phys_az0, phys_el0 = world_to_physical(cal_az0, cal_el0)
    s_az0, s_el0 = physical_to_servo_deg(phys_az0, phys_el0)
    smooth_park(pi, s_az0, s_el0, duration_s=args.park_duration, rate_hz=args.park_rate_hz)
    time.sleep(0.1)
    curr_phys_az = phys_az0
    curr_phys_el = phys_el0

    # NEW: seed last-servo to match actual parked position
    last_servo_az = float(s_az0)
    last_servo_el = float(s_el0)


    # Start readers
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
            print(f"[INFO] Base mode=static → waiting {args.static_window_sec:.1f}s, then freezing base GPS.")
            t_end = time.time() + float(args.static_window_sec)
            while time.time() < t_end:
                b_try = get_latest_base()
                if b_try:
                    base_fixed = (b_try.lat, b_try.lon, b_try.alt)  # keep the most recent in the window
                time.sleep(0.1)
            if base_fixed is None:
                print("[WARN] No base GPS during static window; will freeze on first base sample in the main loop.")
            else:
                print(f"[INFO] Base frozen to lat={base_fixed[0]:.7f}, lon={base_fixed[1]:.7f}, alt={base_fixed[2]:.2f}")
        else:
            print("[INFO] Base mode=dynamic → always use latest base GPS.")

    # ===== Zero-reference setup =====
    print("[INFO] Point the tracker at the drone and stabilize GPS.")
    time.sleep(4.0)

    next_print = time.time()
    #last_servo_az = 0.0
    #last_servo_el = 0.0

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
                base_locked = True
            else:
                if args.base_mode == "static":
                    if base_fixed is None:
                        b_now = get_latest_base()
                        if not b_now:
                            time.sleep(0.01); continue
                        base_fixed = (b_now.lat, b_now.lon, b_now.alt)
                        print(f"[INFO] Base frozen late to lat={base_fixed[0]:.7f}, lon={base_fixed[1]:.7f}, alt={base_fixed[2]:.2f}")
                    b_lat, b_lon, b_alt = base_fixed
                    base_mode_str = "static"
                    base_fix = base_sats = None
                    base_locked = True
                else:
                    b = get_latest_base()
                    if not b:
                        time.sleep(0.01); continue
                    b_lat, b_lon, b_alt = b.lat, b.lon, b.alt
                    base_mode_str = "dynamic"
                    base_fix = getattr(b, "fix_type", None)
                    base_sats = getattr(b, "sats", None)
                    base_locked = False

            # --- Angles (NO filtering) ---
            info = tracker.get_tracking_info(b_lat, b_lon, b_alt, d.lat, d.lon, d.alt)
            if not info:
                time.sleep(args.update_period); continue

            abs_az = info.get('adjusted_azimuth', info['azimuth'])
            abs_el = info.get('adjusted_elevation', info['elevation'])
            world_az = abs_az
            world_el = abs_el

            # --- Calibration → pick normal vs backside by servo-distance ---
            cal_az, cal_el = apply_calibration(world_az, world_el)
            tgt_phys_az, tgt_phys_el, used_flip = choose_flipped_if_better(
                cal_az, cal_el, last_servo_az, last_servo_el
            )

            # --- Per-tick max step (deg/tick) using the current update period ---
            az_step_max = float(AZ_MAX_DEG_PER_SEC) * float(args.update_period)
            el_step_max = float(EL_MAX_DEG_PER_SEC) * float(args.update_period)

            # --- Shortest-path deltas in PHYSICAL space ---
            d_az = shortest_delta_deg(tgt_phys_az, curr_phys_az)   # in (−180, +180]
            d_el = tgt_phys_el - curr_phys_el                      # EL doesn't wrap

            # --- Clamp step to keep it snappy but safe (prevents 180° wrap jumps) ---
            if d_az >  az_step_max: d_az =  az_step_max
            if d_az < -az_step_max: d_az = -az_step_max
            if d_el >  el_step_max: d_el =  el_step_max
            if d_el < -el_step_max: d_el = -el_step_max

            # --- Advance the physical state; keep az wrapped and el clamped ---
            curr_phys_az = wrap360(curr_phys_az + d_az)
            curr_phys_el = max(EL_PHYS_MIN, min(EL_PHYS_MAX, curr_phys_el + d_el))

            # --- Now map CURRENT physical state → servo → pulse ---
            s_az, s_el = physical_to_servo_deg(curr_phys_az, curr_phys_el)
            us_az      = servo_deg_to_us(s_az)
            us_el      = servo_deg_to_us(s_el)

            # --- Drive servos ---
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
            last_servo_az, last_servo_el = s_az, s_el

            # Console output (paced)
            if time.time() >= next_print:
                print(f"[{datetime.now():%H:%M:%S}] Base={base_mode_str} "
                    f"WORLD {world_az:6.2f}/{world_el:5.2f}° | "
                    f"PHYS {curr_phys_az:6.2f}/{curr_phys_el:5.2f}° | "
                    f"SERVO {s_az:6.2f}/{s_el:5.2f}° | us {us_az:5.0f}/{us_el:5.0f}"
                    f"{' | FLIP' if used_flip else ''}")
                next_print += float(args.print_period)

            # CSV log
            log_row(
                datetime.now().isoformat(timespec='seconds'),
                base_mode_str, 1 if base_locked else 0,
                round(world_az,3), round(world_el,3),
                round(cal_az,3), round(cal_el,3),
                round(curr_phys_az,3), round(curr_phys_el,3),
                round(s_az,3), round(s_el,3),
                round(us_az,1), round(us_el,1),
                round(d.lat,7), round(d.lon,7), round(d.alt,2),
                round(b_lat,7), round(b_lon,7), round(b_alt,2),
                0.0, 0.0,
                "" if base_fix is None else base_fix,
                "" if base_sats is None else base_sats,
                1 if used_flip else 0    
            )

            time.sleep(args.update_period)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
    finally:
        print("[INFO] Smooth shutdown: parking…")
        try:
            if args.park_face_drone_exit:
                # Keep current AZ; ramp EL → home
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
