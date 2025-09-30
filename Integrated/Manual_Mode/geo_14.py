#Integration of Manual mode and Auto mode
# geo_14.py — Tracker with static/dynamic base modes + Manual Mode
# To test: Manual Calibration and Manual Mode.

import argparse
from dataclasses import dataclass
from datetime import datetime
import threading
from typing import Optional, Tuple, Callable
import time

import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker
from manual_control import ManualController

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

# MAVLink stream requests
MAV_MSG_INTERVAL_US_GPS    = 200_000   # microseconds - 5 Hz (typical GPS) 
MAV_MSG_INTERVAL_US_GLOBAL = 50_000    # microseconds - 20 Hz (raise to 50_000 for ~20 Hz if supported)

# Gear spokes ratios
SPOKES_SMALL   = 12
SPOKES_BIG     = 24
AZ_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL    
EL_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL

# Physical limits
AZ_PHYS_MIN = 0.0   # deg
AZ_PHYS_MAX = 360.0 # deg
EL_PHYS_MIN = 0.0   # deg
EL_PHYS_MAX = 180.0 # deg

# Servo mapping — 180° servo
PULSE_MIN_US    = 900.0  # microseconds
PULSE_MAX_US    = 1950.0 # microseconds
SERVO_RANGE_DEG = 180.0  # degrees

# Calibration
AZIMUTH_ZERO_OFFSET_DEG   = -60.0     # deg
ELEVATION_ZERO_OFFSET_DEG = 0.0     # deg
AZIMUTH_INVERT   = True             
ELEVATION_INVERT = True

# pigpio GPIO pins
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 17

# Loop timings
UPDATE_PERIOD_S = 0.05   # main loop period; try 0.02–0.05 for snappier updates
PRINT_PERIOD_S  = 1.0    # How crowded you want your CLI to look like
LOG_TO_CSV      = True
LOG_RAW_GPS     = False # Make sure to remove once everything works. Most useless feature I've added *smh smh*

# SIM base (used only in SIM mode)
base_static = {"lat": 13.0276802, "lon": 77.5629616, "alt": 924.36}

# --- Tracking dynamics (Raise the values to 300-360, but can make it jerky or jumpy) -> How fast the antenna rotates ---
AZ_MAX_DEG_PER_SEC   = 180.0
EL_MAX_DEG_PER_SEC   = 180.0

# --- Flip cooldown / stickiness ---
MIN_FLIP_DWELL_S            = 2     # block any re-flip for 2 s
MIN_AZ_DELTA_SINCE_FLIP_DEG = 20.0  # need to move away from seam this much before reconsidering
FLIP_EXTRA_MARGIN_DEG       = 12.0  # extra benefit required to flip to the other side

ALLOW_BACKSIDE_FLIP = True          # Master switch for the flip logic. To be disabled if the drone isnt going to fly beyond 180 degrees elevation

""" 
    Hysteresis prevents the tracker from rapidly flipping back and forth if the drone is hovering right at a point where both the front and back poses are equally "good."
    It makes the current position "stickier" by adding a penalty to the alternative.
    The tracker will only flip if the backside pose is at least 10 servo degrees cheaper in movement than staying in the front pose.
    If you see it oscillating or "hesitating" at the flip point, increase this value. If it seems reluctant to flip when it should, decrease it.

"""
# Flip behavior tuning
ONLY_FLIP_NEAR_EDGE  = True    # KEEP THIS TRUE or you will break the tracker :)
EDGE                 = 10.0    # How close to the edge will the tracker flip. 
FLIP_HYSTERESIS_DEG  = 14.0    # Was 10.0; lets the new side win sooner
MIN_EL_FOR_FLIP      = 6.0     # Edit this if the elvation is passing through the tracker 

# Cost weighting: make azimuth more important than elevation near the seam
"""
If you want to make flips happen more readily, you could slightly decrease EL_WEIGHT (e.g., to 0.25) to make elevation movements even "cheaper" in the cost calculation.
"""
AZ_WEIGHT = 1.0
EL_WEIGHT = 0.30                # 0.25–0.35 works well

# If your rig wants “mirror elevation” use mirror_el; else try keep_el. Play with this if the elevation seems off after a flip.
FLIP_STYLE = "keep_el"          # "mirror_el" or "keep_el"

""" Edit these paramters to add a small bias to the azimuth/elevation ONLY when a flip occurs."""
FLIP_AZ_CORR_DEG = 0.0          # add/subtract small az bias ONLY when flipped
FLIP_EL_CORR_DEG = 0.0          # add/subtract small el bias ONLY when flipped

MODE_AUTO = "auto"
MODE_MANUAL = "manual"
_current_mode_lock = threading.Lock()
_current_mode = MODE_AUTO  # will be overriden by --start-mode

CAL_FILE = "calibration.json"  # ADD
CAL = {"az_zero_offset_deg": 0.0, "el_zero_offset_deg": 0.0} 

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
    # add fixed trims + dynamic zero offsets, then handle inversions
    az = norm360(az + AZIMUTH_ZERO_OFFSET_DEG + CAL["az_zero_offset_deg"])
    el = el + ELEVATION_ZERO_OFFSET_DEG + CAL["el_zero_offset_deg"]
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

def clamp_physical(az, el):
    # clamp to your physical pan-tilt range; typical: el 0..180, az unbounded
    el = max(0.0, min(180.0, el))
    return az, el

def shortest_servo_delta(a, b):
    """Return signed shortest delta between two servo angles in [0,180].
       For 0-180 servos, this is just b - a (no wrap), but keep the helper
       in case you later map to a 0-360 continuous az servo."""
    return b - a

def pick_target(cal_az, cal_el, last_servo_az, last_servo_el):
    """Return (tgt_phys_az, tgt_phys_el, used_flip) after computing A/B options & costs."""
    # --- Build the two candidate poses in WORLD/PHYS/SE RVO space ---
    # A: normal

    if not ALLOW_BACKSIDE_FLIP:
        A_phys_az, A_phys_el = world_to_physical(cal_az, cal_el)
        return A_phys_az, A_phys_el, False

    A_world_az, A_world_el = cal_az, cal_el

    # B: backside (180° az shift; keep or mirror EL per your setting)
    B_world_az = wrap360(cal_az + 180.0)
    if FLIP_STYLE == "mirror_el":
        B_world_el = max(EL_PHYS_MIN, min(EL_PHYS_MAX, 180.0 - cal_el))
    else:
        B_world_el = cal_el

    # Small per-flip nudges (applied only if we actually flip later)
    def _apply_flip_corr(az, el):
        return wrap360(az + FLIP_AZ_CORR_DEG), max(EL_PHYS_MIN, min(EL_PHYS_MAX, el + FLIP_EL_CORR_DEG))

    # Map to PHYSICAL then SERVO for cost calc
    A_phys_az, A_phys_el = world_to_physical(A_world_az, A_world_el)
    B_phys_az, B_phys_el = world_to_physical(B_world_az, B_world_el)
    A_saz, A_sel = physical_to_servo_deg(A_phys_az, A_phys_el)
    B_saz, B_sel = physical_to_servo_deg(B_phys_az, B_phys_el)

    # Costs in SERVO degrees (what your motors actually move)
    cost_A = AZ_WEIGHT * abs(A_saz - last_servo_az) + EL_WEIGHT * abs(A_sel - last_servo_el)
    cost_B = AZ_WEIGHT * abs(B_saz - last_servo_az) + EL_WEIGHT * abs(B_sel - last_servo_el)

    # Respect horizon guard if you use it
    if B_world_el < MIN_EL_FOR_FLIP:
        used_flip = False
        return A_phys_az, A_phys_el, used_flip

    # Don’t allow flips far from seam if that’s your policy
    only_near_edge = ONLY_FLIP_NEAR_EDGE

    # Decide flip using the patched chooser
    used_flip, _chosen_saz = choose_flipped_if_better(A_saz, B_saz, cost_A, cost_B, only_flip_near_edge=only_near_edge)

    if used_flip:
        # Apply small corrections only when committing to flip
        B_world_az_corr, B_world_el_corr = _apply_flip_corr(B_world_az, B_world_el)
        B_phys_az, B_phys_el = world_to_physical(B_world_az_corr, B_world_el_corr)
        return B_phys_az, B_phys_el, True
    else:
        return A_phys_az, A_phys_el, False


def choose_flipped_if_better(A_saz, B_saz, cost_A, cost_B, only_flip_near_edge=True):
    """
    Decide whether to flip sides. Returns (use_flip: bool, chosen_saz: float)

    A_saz: current-side azimuth solution (deg)
    B_saz: flipped-side azimuth solution (deg)
    cost_A, cost_B: your existing cost values for A and B
    only_flip_near_edge: keep your existing behavior of allowing flips
                         only near the 0°/180° seam when True
    """
    now = time.time()

    # persistent state stored on the function itself
    st = choose_flipped_if_better.__dict__
    last_flip_t = st.get("_last_flip_t", 0.0)
    last_flip_saz = st.get("_last_flip_saz", None)

    def near_edge(saz: float) -> bool:
    # seam is at 0° and 180° in SERVO space
        return (saz <= EDGE) or (saz >= (180.0 - EDGE))

    # Gate flipping
    allow_flip = True
    if only_flip_near_edge:
        allow_flip = near_edge(A_saz) or near_edge(B_saz)

    # time-based cooldown
    if (now - last_flip_t) < MIN_FLIP_DWELL_S:
        allow_flip = False

    # movement-based cooldown (don't re-flip until we moved away from seam)
    if last_flip_saz is not None:
        if abs(B_saz - last_flip_saz) < MIN_AZ_DELTA_SINCE_FLIP_DEG:
            allow_flip = False

    # Stronger "stickiness": add hysteresis + extra margin to B
    thresholded_cost_B = cost_B + FLIP_HYSTERESIS_DEG + FLIP_EXTRA_MARGIN_DEG

    if allow_flip and (thresholded_cost_B < cost_A):
        # commit to flip and remember time/angle
        st["_last_flip_t"] = now
        st["_last_flip_saz"] = B_saz
        return True, B_saz
    else:
        return False, A_saz

# === Custom AZ mapping ===
# Fit through: (0°,900us), (180°,1400us), (360°,1950us)
# us(az_phys) = 900 + (az^2)/1296 + (95/36)*az
def az_phys_to_us(az_phys: float) -> float:
    az = max(0.0, min(360.0, float(az_phys)))
    us = 900.0 + (az * az) / 1296.0 + (95.0 / 36.0) * az
    return _bounded_us(us)

def servo_deg_to_us_az(servo_deg: float) -> float:
    # Convert servo degrees back to PHYSICAL az using gear ratio, then apply calibrated mapping
    d = max(0.0, min(SERVO_RANGE_DEG, float(servo_deg)))
    phys_az = d * float(AZ_GEAR_RATIO)
    return az_phys_to_us(phys_az)

def get_mode():
    with _current_mode_lock:
        return _current_mode

def set_mode(m):
    global _current_mode
    with _current_mode_lock:
        _current_mode = m

def toggle_mode():
    global _current_mode
    with _current_mode_lock:
        _current_mode = MODE_MANUAL if _current_mode == MODE_AUTO else MODE_AUTO
        print(f"[MODE] Switching ---> {_current_mode.upper()}")

def load_calibration():
    import json, os
    if os.path.exists(CAL_FILE):
        with open(CAL_FILE, "r") as f:
            CAL.update(json.load(f))
        print(f"[CAL] Loaded: AZ_OFF={CAL['az_zero_offset_deg']:.2f}°, EL_OFF={CAL['el_zero_offset_deg']:.2f}°")

def save_calibration():
    import json
    with open(CAL_FILE, "w") as f:
        json.dump(CAL, f, indent=2)
    print(f"[CAL] Saved: AZ_OFF={CAL['az_zero_offset_deg']:.2f}°, EL_OFF={CAL['el_zero_offset_deg']:.2f}°")

def calibrate_simple(pi, args, manual_ctrl, curr_phys_az, curr_phys_el, persist=True):
    """
    1) Switch to MANUAL
    2) Use WASD to aim the antenna where 'world zero' should be
    3) Press 'm' to finish; we set zero to that pose
    4) Restart automation cleanly and return updated state
    """
    print("\n[CAL] Quick calibration: use WASD (SHIFT = faster). Press 'm' when done.\n")
    set_mode(MODE_MANUAL)

    # compact manual loop (no extra features)
    next_print = time.time()
    while get_mode() == MODE_MANUAL:
        daz, delv = manual_ctrl.read_command(timeout=args.update_period)  # returns (Δaz°, Δel°)
        if daz or delv:
            curr_phys_az = wrap360(curr_phys_az + daz)
            curr_phys_el = max(EL_PHYS_MIN, min(EL_PHYS_MAX, curr_phys_el + delv))
            s_az, s_el = physical_to_servo_deg(curr_phys_az, curr_phys_el)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, servo_deg_to_us_az(s_az))
            pi.set_servo_pulsewidth(SERVO_EL_PIN, servo_deg_to_us(s_el))

        if time.time() >= next_print:
            print(f"[{datetime.now():%H:%M:%S}] CAL  PHYS={curr_phys_az:6.2f}/{curr_phys_el:5.2f}°  (press 'm' to accept)")
            next_print += float(args.print_period)

    # --- loop ends when user presses 'm' and mode flips back to AUTO ---

    # Compute offsets so WORLD(0,0) maps to the pose you just aimed at
    if AZIMUTH_INVERT:
        CAL["az_zero_offset_deg"] = norm360((360.0 - curr_phys_az) - AZIMUTH_ZERO_OFFSET_DEG)
    else:
        CAL["az_zero_offset_deg"] = norm360(curr_phys_az - AZIMUTH_ZERO_OFFSET_DEG)

    if ELEVATION_INVERT:
        CAL["el_zero_offset_deg"] = -(curr_phys_el) - ELEVATION_ZERO_OFFSET_DEG
    else:
        CAL["el_zero_offset_deg"] =  (curr_phys_el) - ELEVATION_ZERO_OFFSET_DEG

    if persist:
        save_calibration()

    # Clean automation restart so new offsets take effect smoothly
    last_servo_az, last_servo_el, curr_phys_az, curr_phys_el = reset_automation_state(pi, args)
    return last_servo_az, last_servo_el, curr_phys_az, curr_phys_el


def reset_automation_state(pi, args):
    """
    Called when switching MANUAL → AUTO to restart the automation cleanly.
    - Clears flip chooser history
    - Parks to home smoothly (or face-drone if you prefer)
    - Resets last/current servo/phys state aligned to park target
    Returns (last_servo_az, last_servo_el, curr_phys_az, curr_phys_el)
    """
    # Clear flip memory
    try:
        st = choose_flipped_if_better.__dict__
        st.pop("_last_flip_t", None)
        st.pop("_last_flip_saz", None)
    except Exception:
        pass

    # Park to home (you can choose face_drone if you like)
    cal_azH, cal_elH = apply_calibration(args.park_home_az, args.park_home_el)
    phys_azH, phys_elH = world_to_physical(cal_azH, cal_elH)
    s_azH, s_elH = physical_to_servo_deg(phys_azH, phys_elH)
    smooth_park(pi, s_azH, s_elH, duration_s=args.park_duration, rate_hz=args.park_rate_hz)
    time.sleep(0.05)

    # Align automation state to parked pos
    return float(s_azH), float(s_elH), float(phys_azH), float(phys_elH)

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

    target_us_az = servo_deg_to_us_az(t_az_deg)
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

# ===================== Manual Control =====================

# === add two tight loops with cooperative yielding ===
def run_manual_loop(pi, args, state_reader, manual_ctrl,
                    curr_phys_az, curr_phys_el):
    """
    state_reader(): returns a dict with keys:
        'world_az','world_el','b_lat','b_lon','b_alt','base_mode_str','base_locked','d','b'
    For manual loop, we don't use the world angles to move, but we DO keep logging & printing.
    """
    print("[MANUAL] Use WASD (hold SHIFT for faster). 'm' to return to AUTO. 'q' quick-park home.")
    next_print = time.time()
    last_servo_az, last_servo_el = physical_to_servo_deg(curr_phys_az, curr_phys_el)

    while get_mode() == MODE_MANUAL:
        daz, delv = manual_ctrl.read_command(timeout=args.update_period)
        if daz != 0.0 or delv != 0.0:
            # update physical angles with clamps/wrap
            curr_phys_az = wrap360(curr_phys_az + daz)
            curr_phys_el = max(EL_PHYS_MIN, min(EL_PHYS_MAX, curr_phys_el + delv))

            s_az, s_el = physical_to_servo_deg(curr_phys_az, curr_phys_el)
            us_az = servo_deg_to_us_az(s_az)
            us_el = servo_deg_to_us(s_el)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
            last_servo_az, last_servo_el = s_az, s_el

        # optional periodic console + CSV logging with latest telemetry
        info = state_reader()   # non-blocking snapshot
        if info and time.time() >= next_print:
            print(f"[{datetime.now():%H:%M:%S}] MANUAL "
                  f"PHYS {curr_phys_az:6.2f}/{curr_phys_el:5.2f}° | "
                  f"SERVO {last_servo_az:6.2f}/{last_servo_el:5.2f}°")
            next_print += float(args.print_period)

        # best-effort CSV row (mark UsedFlip=0 in manual)
        try:
            if info:
                d = info['d']; b_lat, b_lon, b_alt = info['b_lat'], info['b_lon'], info['b_alt']
                s_az, s_el = last_servo_az, last_servo_el
                us_az, us_el = servo_deg_to_us_az(s_az), servo_deg_to_us(s_el)
                log_row(
                    datetime.now().isoformat(timespec='seconds'),
                    "manual", 1 if info['base_locked'] else 0,
                    "", "",  "", "",
                    round(curr_phys_az,3), round(curr_phys_el,3),
                    round(s_az,3), round(s_el,3),
                    round(us_az,1), round(us_el,1),
                    "" if not d else round(d.lat,7), "" if not d else round(d.lon,7), "" if not d else round(d.alt,2),
                    round(b_lat,7), round(b_lon,7), round(b_alt,2),
                    0.0, 0.0,
                    "", "",
                    0
                )
        except Exception:
            pass

    # return latest physical angles to feed into AUTO restart logic if needed
    return curr_phys_az, curr_phys_el

def run_auto_tick(pi, args, last_servo_az, last_servo_el, curr_phys_az, curr_phys_el):
    """
    One iteration of your existing automation logic factored into a function.
    Returns updated (last_servo_az, last_servo_el, curr_phys_az, curr_phys_el).
    """
    d = get_latest_drone()
    if not d:
        time.sleep(0.01); return last_servo_az, last_servo_el, curr_phys_az, curr_phys_el

    # --- BASE selection copied from main loop ---
    if args.mode == "sim":
        b_lat, b_lon, b_alt = base_static["lat"], base_static["lon"], base_static["alt"]
        base_mode_str = "static(SIM)"
        base_fix = base_sats = None
        base_locked = True
    else:
        # static vs dynamic as in your code:
        # we reuse a cached 'base_fixed' via closure on outer variables (see integration below)
        if run_auto_tick._base_mode == "static":
            if run_auto_tick._base_fixed is None:
                b_now = get_latest_base()
                if not b_now:
                    time.sleep(0.01); return last_servo_az, last_servo_el, curr_phys_az, curr_phys_el
                run_auto_tick._base_fixed = (b_now.lat, b_now.lon, b_now.alt)
                print(f"[INFO] Base frozen late to lat={run_auto_tick._base_fixed[0]:.7f}, lon={run_auto_tick._base_fixed[1]:.7f}, alt={run_auto_tick._base_fixed[2]:.2f}")
            b_lat, b_lon, b_alt = run_auto_tick._base_fixed
            base_mode_str = "static"
            base_fix = base_sats = None
            base_locked = True
        else:
            b = get_latest_base()
            if not b:
                time.sleep(0.01); return last_servo_az, last_servo_el, curr_phys_az, curr_phys_el
            b_lat, b_lon, b_alt = b.lat, b.lon, b.alt
            base_mode_str = "dynamic"
            base_fix = getattr(b, "fix_type", None)
            base_sats = getattr(b, "sats", None)
            base_locked = False

    info = tracker.get_tracking_info(b_lat, b_lon, b_alt, d.lat, d.lon, d.alt)
    if not info:
        time.sleep(args.update_period); return last_servo_az, last_servo_el, curr_phys_az, curr_phys_el

    world_az = info['azimuth']
    world_el = info['elevation']

    cal_az, cal_el = apply_calibration(world_az, world_el)
    tgt_phys_az, tgt_phys_el, used_flip = pick_target(cal_az, cal_el, last_servo_az, last_servo_el)

    az_step_max = float(AZ_MAX_DEG_PER_SEC) * float(args.update_period)
    el_step_max = float(EL_MAX_DEG_PER_SEC) * float(args.update_period)

    d_az = shortest_delta_deg(tgt_phys_az, curr_phys_az)
    d_el = tgt_phys_el - curr_phys_el
    d_az = max(-az_step_max, min(az_step_max, d_az))
    d_el = max(-el_step_max, min(el_step_max, d_el))

    curr_phys_az = wrap360(curr_phys_az + d_az)
    curr_phys_el = max(EL_PHYS_MIN, min(EL_PHYS_MAX, curr_phys_el + d_el))

    s_az, s_el = physical_to_servo_deg(curr_phys_az, curr_phys_el)
    us_az = servo_deg_to_us_az(s_az)
    us_el = servo_deg_to_us(s_el)
    pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
    pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
    last_servo_az, last_servo_el = s_az, s_el

    # paced print
    now = time.time()
    if now >= run_auto_tick._next_print:
        print(f"[{datetime.now():%H:%M:%S}] Base={base_mode_str} "
              f"WORLD {world_az:6.2f}/{world_el:5.2f}° | "
              f"PHYS {curr_phys_az:6.2f}/{curr_phys_el:5.2f}° | "
              f"SERVO {s_az:6.2f}/{s_el:5.2f}° | us {us_az:5.0f}/{us_el:5.0f}"
              f"{' | FLIP' if used_flip else ''}")
        run_auto_tick._next_print += float(run_auto_tick._print_period)

    # CSV
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
    return last_servo_az, last_servo_el, curr_phys_az, curr_phys_el


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

    ap.add_argument("--start-mode", choices=[MODE_AUTO, MODE_MANUAL], default=MODE_AUTO,
                    help="Start in auto or manual mode (toggle with 'm')")
    ap.add_argument("--manual-step", type=float, default=2.0, help="WASD step in degrees")
    ap.add_argument("--manual-step-fast", type=float, default=8.0, help="Shift+WASD step in degrees")

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

    load_calibration()

    print("=== geo_13 + Manual Mode (sim/ground) — static/dynamic base, zero-ref, smooth parking ===")
    print(f"[CFG] Mode: {args.mode} | BaseMode: {args.base_mode} | StartMode: {args.start_mode}")
    print(f"[CFG] Gear AZ {AZ_GEAR_RATIO}:1, EL {EL_GEAR_RATIO}:1 | Servo 180° @ {PULSE_MIN_US}-{PULSE_MAX_US}µs")
    print(f"[CFG] update_period={args.update_period:.2f}s, print_period={args.print_period:.1f}s")
    print(f"[CFG] parking: home=({args.park_home_az:.1f}°, {args.park_home_el:.1f}°) face_drone(start={args.park_face_drone_start}, exit={args.park_face_drone_exit}) duration={args.park_duration:.2f}s @ {args.park_rate_hz:.0f} Hz")
    print(f"[CFG] manual steps: {args.manual_step}° / {args.manual_step_fast}° (with SHIFT)")


    # pigpio + logs
    pi = setup_pigpio()
    log_open(prefix="Tracker")

    # initial park to home
    print("[INFO] Parking to home based on fixed zero (no learned zero).")
    phys_az0, phys_el0 = world_to_physical(args.park_home_az, args.park_home_el)
    s_az0, s_el0 = physical_to_servo_deg(phys_az0, phys_el0)
    smooth_park(pi, s_az0, s_el0, duration_s=args.park_duration, rate_hz=args.park_rate_hz)
    time.sleep(0.1)

    last_servo_az = float(s_az0)
    last_servo_el = float(s_el0)
    curr_phys_az  = float(phys_az0)
    curr_phys_el  = float(phys_el0)

    # Connect MAVs + readers
    if args.mode == "sim":
        mav_drone = connect_mav(SIM_DRONE_ENDPOINT, SIM_DRONE_BAUD, True)
        start_reader(mav_drone, "DRONE", on_raw=log_raw if LOG_RAW_GPS else None)
        print(f"[CFG] SIM base @ {base_static['lat']}, {base_static['lon']}, {base_static['alt']}m")
    else:
        mav_drone = connect_mav(DRONE_ENDPOINT, DRONE_BAUD, True)
        mav_base  = connect_mav(BASE_ENDPOINT,  BASE_BAUD,  False)
        start_reader(mav_drone, "DRONE", on_raw=log_raw if LOG_RAW_GPS else None)
        start_reader(mav_base,  "BASE",  on_raw=log_raw if LOG_RAW_GPS else None)

    # Base mode bootstrap used inside run_auto_tick (closured variables)
    run_auto_tick._base_mode   = args.base_mode
    run_auto_tick._base_fixed  = None
    run_auto_tick._print_period = float(args.print_period)
    run_auto_tick._next_print   = time.time()

    # start manual controller (even if starting in AUTO; we need 'm' hotkey)
    set_mode(args.start_mode)
    manual_ctrl = ManualController(
        on_toggle_mode=toggle_mode,
        on_quick_park=lambda: smooth_park(pi, *physical_to_servo_deg(*world_to_physical(args.park_home_az, args.park_home_el)),
                                          duration_s=args.park_duration, rate_hz=args.park_rate_hz),
        step_deg=args.manual_step,
        step_deg_fast=args.manual_step_fast,
        poll_hz=max(30.0, 1.0/args.update_period)
    )
    manual_ctrl.start()

    # a light “state reader” for manual logging/printing
    def state_reader():
        d = get_latest_drone()
        if args.mode == "sim":
            b_lat, b_lon, b_alt = base_static["lat"], base_static["lon"], base_static["alt"]
            return {'d': d, 'b_lat': b_lat, 'b_lon': b_lon, 'b_alt': b_alt,
                    'base_mode_str': 'static(SIM)', 'base_locked': True}
        else:
            if args.base_mode == "static":
                if run_auto_tick._base_fixed:
                    b_lat, b_lon, b_alt = run_auto_tick._base_fixed
                else:
                    b = get_latest_base()
                    if not b: return None
                    b_lat, b_lon, b_alt = b.lat, b.lon, b.alt
                return {'d': d, 'b_lat': b_lat, 'b_lon': b_lon, 'b_alt': b_alt,
                        'base_mode_str': 'static', 'base_locked': True}
            else:
                b = get_latest_base()
                if not b: return None
                return {'d': d, 'b_lat': b.lat, 'b_lon': b.lon, 'b_alt': b.alt,
                        'base_mode_str': 'dynamic', 'base_locked': False}

    print("[INFO] Point the tracker at the drone and stabilize GPS.")
    time.sleep(4.0)

    try:
        while True:
            mode = get_mode()

            if mode == MODE_AUTO:
                # ADD: run calibration once on entry to AUTO
                if main._last_mode != MODE_AUTO:
                    (last_servo_az, last_servo_el,
                    curr_phys_az, curr_phys_el) = calibrate_simple(
                        pi, args, manual_ctrl, curr_phys_az, curr_phys_el, persist=True
                    )

                # existing AUTO tick
                last_servo_az, last_servo_el, curr_phys_az, curr_phys_el = run_auto_tick(
                    pi, args, last_servo_az, last_servo_el, curr_phys_az, curr_phys_el
                )

            elif mode == MODE_MANUAL:
                # optional: you may keep/run your standalone manual loop here
                curr_phys_az, curr_phys_el = run_manual_loop(pi, args, state_reader, manual_ctrl,
                                                            curr_phys_az, curr_phys_el)

            main._last_mode = mode
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
    finally:
        print("[INFO] Smooth shutdown: parking…")
        try:
            # same parking cleanup as your original
            if args.park_face_drone_exit:
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
