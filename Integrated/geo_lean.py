#!/usr/bin/env python3
"""
Core-only antenna tracker main.py
- Two GPS inputs (base + drone) via MAVLink
- World → (az, el) using your existing azi_elev_5.get_tracking_info
- Single flip policy: backside = (az+180, el)  # keep elevation
- Flip allowed only near seam with hysteresis + short cooldown
- Static base (averaged window) or dynamic base
- Deterministic world-space parking; smooth ramp to avoid twitch
- Servo mapping (180°) + rate limiting
- Minimal CSV logging (optional)

~200-ish lines, focused on accuracy and basic functionality.
"""
from __future__ import annotations
import argparse, time, threading, csv
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple

import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker  # uses your proven math

# ================== Defaults (edit to taste) ==================
MODE_DEFAULT = "ground"             # "ground" | "sim"
BASE_MODE_DEFAULT = "static"        # "static" | "dynamic"
STATIC_WINDOW_SEC_DEFAULT = 10.0     # averaging window for static base
UPDATE_PERIOD_S_DEFAULT = 0.06       # 60–100 ms is snappy & smooth
PRINT_PERIOD_S_DEFAULT = 1.0
LOG_TO_CSV_DEFAULT = True

# Endpoints
SIM_DRONE_ENDPOINT = "udp:0.0.0.0:14550"
SIM_DRONE_BAUD     = None
DRONE_ENDPOINT     = "/dev/ttyUSB0"
DRONE_BAUD         = 57600
BASE_ENDPOINT      = "/dev/ttyACM0"
BASE_BAUD          = 57600

# Gear & physical limits
AZ_GEAR_RATIO = 2.0
EL_GEAR_RATIO = 2.0
SERVO_RANGE_DEG = 180.0
PULSE_MIN_US, PULSE_MAX_US = 900.0, 2100.0
AZ_PHYS_MIN, AZ_PHYS_MAX = 0.0, 360.0
EL_PHYS_MIN, EL_PHYS_MAX = 0.0, 180.0

# Calibration (keep simple)
AZIMUTH_ZERO_OFFSET_DEG = 0.0
ELEVATION_ZERO_OFFSET_DEG = 0.0
AZIMUTH_INVERT = False
ELEVATION_INVERT = False

# GPIO pins (BCM)
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 17

# Dynamics (safe defaults)
AZ_MAX_DEG_PER_SEC = 180.0
EL_MAX_DEG_PER_SEC = 180.0

# Flip policy (single, clean)
ONLY_FLIP_NEAR_EDGE = True
EDGE = 28.0                 # seam arm distance in servo deg
FLIP_HYSTERESIS_DEG = 14.0  # stickiness once a side is chosen
MIN_EL_FOR_FLIP = 6.0       # no flip grazing horizon
FLIP_COOLDOWN_S = 1.2       # prevents ping-pong
FLIP_STYLE = "keep_el"      # requirement: keep elevation on backside

# Timestamp skew guard (dynamic base)
MAX_SAMPLE_SKEW_S = 0.20

# ================== Helpers ==================

def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

def wrap360(x: float) -> float:
    return (x + 360.0) % 360.0

def shortest_delta_deg(target: float, current: float) -> float:
    # signed delta in (−180, +180]
    return ((target - current + 540.0) % 360.0) - 180.0

def apply_calibration(az: float, el: float) -> Tuple[float, float]:
    az = norm360(az + AZIMUTH_ZERO_OFFSET_DEG)
    el = el + ELEVATION_ZERO_OFFSET_DEG
    if AZIMUTH_INVERT:
        az = norm360(360.0 - az)
    if ELEVATION_INVERT:
        el = -el
    return az, el

def world_to_physical(az: float, el: float) -> Tuple[float, float]:
    return (max(AZ_PHYS_MIN, min(AZ_PHYS_MAX, az)),
            max(EL_PHYS_MIN, min(EL_PHYS_MAX, el)))

def physical_to_servo_deg(az_phys: float, el_phys: float) -> Tuple[float, float]:
    az_servo = max(0.0, min(SERVO_RANGE_DEG, az_phys / AZ_GEAR_RATIO))
    el_servo = max(0.0, min(SERVO_RANGE_DEG, el_phys / EL_GEAR_RATIO))
    return az_servo, el_servo

def servo_deg_to_us(deg: float) -> float:
    d = max(0.0, min(SERVO_RANGE_DEG, float(deg)))
    return float(PULSE_MIN_US) + (d / SERVO_RANGE_DEG) * (float(PULSE_MAX_US) - float(PULSE_MIN_US))

# pigpio

def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running (sudo pigpiod)")
    pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
    return pi

def smooth_park(pi, saz: float, sel: float, duration_s: float = 1.2, rate_hz: float = 60.0):
    t_us_az, t_us_el = servo_deg_to_us(saz), servo_deg_to_us(sel)
    try:
        s_az = float(pi.get_servo_pulsewidth(SERVO_AZ_PIN))
        s_el = float(pi.get_servo_pulsewidth(SERVO_EL_PIN))
    except Exception:
        s_az = t_us_az; s_el = t_us_el
    if not (500.0 <= s_az <= 2500.0): s_az = t_us_az
    if not (500.0 <= s_el <= 2500.0): s_el = t_us_el
    steps = max(1, int(duration_s * rate_hz))
    for i in range(1, steps + 1):
        a = i / steps
        pi.set_servo_pulsewidth(SERVO_AZ_PIN, s_az + (t_us_az - s_az) * a)
        pi.set_servo_pulsewidth(SERVO_EL_PIN, s_el + (t_us_el - s_el) * a)
        time.sleep(1.0 / rate_hz)

# MAVLink I/O (lean)

@dataclass
class GpsSample:
    t: float
    lat: float
    lon: float
    alt: float
    fix: Optional[int] = None
    sats: Optional[int] = None

_latest_base: Optional[GpsSample] = None
_latest_drone: Optional[GpsSample] = None
_base_lock = threading.Lock()
_drone_lock = threading.Lock()

def _extract_sample(msg) -> Optional[GpsSample]:
    tnow = time.time()
    tp = msg.get_type()
    if tp == "GPS_RAW_INT":
        return GpsSample(tnow, msg.lat/1e7, msg.lon/1e7, msg.alt/1000.0,
                         getattr(msg, "fix_type", None), getattr(msg, "satellites_visible", None))
    if tp == "GLOBAL_POSITION_INT":
        return GpsSample(tnow, msg.lat/1e7, msg.lon/1e7, msg.alt/1000.0)
    return None

def connect_mav(endpoint: str, baud: Optional[int], require_hb: bool) -> mavutil.mavfile:
    mav = mavutil.mavlink_connection(endpoint, baud=baud) if baud else mavutil.mavlink_connection(endpoint)
    try:
        mav.wait_heartbeat(timeout=5)
    except Exception:
        if require_hb:
            raise
    # Request message intervals (helps UI snappiness if autopilot honors it)
    for mid, usec in ((mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 200_000),
                      (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200_000)):
        try:
            mav.mav.command_long_send(mav.target_system or 0, mav.target_component or 0,
                                      mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                      0, mid, usec, 0,0,0,0,0)
        except Exception:
            pass
    return mav

def start_reader(mav: mavutil.mavfile, is_base: bool):
    def _run():
        global _latest_base, _latest_drone
        while True:
            msg = mav.recv_match(type=["GPS_RAW_INT","GLOBAL_POSITION_INT"], blocking=True, timeout=1.5)
            if not msg: 
                continue
            s = _extract_sample(msg)
            if not s:
                continue
            if is_base:
                with _base_lock: _latest_base = s
            else:
                with _drone_lock: _latest_drone = s
    th = threading.Thread(target=_run, daemon=True)
    th.start(); return th

def get_latest_base() -> Optional[GpsSample]:
    with _base_lock: return _latest_base

def get_latest_drone() -> Optional[GpsSample]:
    with _drone_lock: return _latest_drone

# ================== Flip chooser (single policy) ==================
_last_flip_time = 0.0
_last_side_flip = False  # False = normal/front, True = backside

def near_seam(saz: float) -> bool:
    return (saz <= EDGE) or (saz >= (SERVO_RANGE_DEG - EDGE))

def pick_pose(cal_az: float, cal_el: float, last_saz: float, last_sel: float) -> Tuple[float,float,bool]:
    """Return PHYSICAL (az, el, used_flip) with keep_el backside when beneficial.
    Costs are in SERVO degrees (what motors move)."""
    global _last_flip_time, _last_side_flip
    # A) normal
    A_phys_az, A_phys_el = world_to_physical(cal_az, cal_el)
    A_saz, A_sel = physical_to_servo_deg(A_phys_az, A_phys_el)
    # B) backside (yaw only); keep elevation
    B_phys_az, B_phys_el = world_to_physical(wrap360(cal_az + 180.0), cal_el)
    B_saz, B_sel = physical_to_servo_deg(B_phys_az, B_phys_el)

    # Horizon guard
    if cal_el < MIN_EL_FOR_FLIP:
        return A_phys_az, A_phys_el, False

    # Costs from current servo pose
    def cost(saz, sel):
        return abs(saz - last_saz) + 0.30 * abs(sel - last_sel)
    cost_A, cost_B = cost(A_saz, A_sel), cost(B_saz, B_sel)

    # Flip gating
    allow = True
    if ONLY_FLIP_NEAR_EDGE:
        allow = near_seam(A_saz) or near_seam(B_saz)
    if (time.time() - _last_flip_time) < FLIP_COOLDOWN_S:
        allow = False

    # Hysteresis/stickiness: penalize changing side unless clearly better
    if _last_side_flip:  # we were flipped; discourage going back
        cost_A -= FLIP_HYSTERESIS_DEG
    else:
        cost_B -= FLIP_HYSTERESIS_DEG

    use_flip = allow and (cost_B < cost_A)
    if use_flip:
        _last_flip_time = time.time(); _last_side_flip = True
        return B_phys_az, B_phys_el, True
    else:
        _last_side_flip = False
        return A_phys_az, A_phys_el, False

# ================== Main ==================

def main():
    ap = argparse.ArgumentParser(description="Core antenna tracker")
    ap.add_argument("--mode", choices=["ground","sim"], default=MODE_DEFAULT)
    ap.add_argument("--base-mode", choices=["static","dynamic"], default=BASE_MODE_DEFAULT)
    ap.add_argument("--static-window-sec", type=float, default=STATIC_WINDOW_SEC_DEFAULT)
    ap.add_argument("--update-period", type=float, default=UPDATE_PERIOD_S_DEFAULT)
    ap.add_argument("--print-period", type=float, default=PRINT_PERIOD_S_DEFAULT)
    ap.add_argument("--park-home-az", type=float, default=0.0)
    ap.add_argument("--park-home-el", type=float, default=0.0)
    ap.add_argument("--log", choices=["on","off"], default=("on" if LOG_TO_CSV_DEFAULT else "off"))
    args = ap.parse_args()

    pi = setup_pigpio()

    # --- Park deterministically in WORLD (no calibration at park) ---
    s0_az, s0_el = physical_to_servo_deg(*world_to_physical(args.park_home_az, args.park_home_el))
    smooth_park(pi, s0_az, s0_el, duration_s=1.2, rate_hz=60.0)
    last_saz, last_sel = s0_az, s0_el
    curr_phys_az, curr_phys_el = world_to_physical(args.park_home_az, args.park_home_el)

    # MAVLink wiring
    if args.mode == "sim":
        md = connect_mav(SIM_DRONE_ENDPOINT, SIM_DRONE_BAUD, True)
        start_reader(md, is_base=False)
        base_fixed = (13.0276802, 77.5629616, 924.36)
        base_mode_str = "static(SIM)"; base_locked = True
    else:
        md = connect_mav(DRONE_ENDPOINT, DRONE_BAUD, True)
        mb = connect_mav(BASE_ENDPOINT,  BASE_BAUD,  False)
        start_reader(md, is_base=False)
        start_reader(mb, is_base=True)
        base_fixed = None; base_mode_str = args.base_mode; base_locked = (args.base_mode == "static")

    # Static base averaging
    if args.mode == "ground" and args.base_mode == "static":
        print(f"[INFO] Sampling base for {args.static_window_sec:.1f}s, freezing to average…")
        t_end = time.time() + float(args.static_window_sec)
        lats, lons, alts = [], [], []
        while time.time() < t_end:
            b = get_latest_base()
            if b:
                lats.append(b.lat); lons.append(b.lon); alts.append(b.alt)
            time.sleep(0.05)
        if lats:
            base_fixed = (sum(lats)/len(lats), sum(lons)/len(lons), sum(alts)/len(alts))
            print(f"[INFO] Base frozen: lat={base_fixed[0]:.7f} lon={base_fixed[1]:.7f} alt={base_fixed[2]:.2f}")
        else:
            print("[WARN] No base samples; will freeze on first seen in loop")

    # CSV log (optional)
    writer = None
    if args.log == "on":
        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        f = open(f"Tracker_{ts}.csv", "w", newline="")
        writer = csv.writer(f)
        writer.writerow(["Time","BaseMode","WorldAz","WorldEl","CalAz","CalEl","PhysAz","PhysEl","ServoAz","ServoEl","UsedFlip"])  # compact

    next_print = time.time()

    try:
        while True:
            d = get_latest_drone()
            if not d:
                time.sleep(0.01); continue

            # Base selection
            if args.mode == "sim":
                b_lat,b_lon,b_alt = base_fixed
            elif args.base_mode == "static":
                if base_fixed is None:
                    b = get_latest_base()
                    if not b: time.sleep(0.01); continue
                    base_fixed = (b.lat,b.lon,b.alt)
                b_lat,b_lon,b_alt = base_fixed
            else:
                b = get_latest_base()
                if not b: time.sleep(0.01); continue
                # skew guard while moving
                if abs(d.t - b.t) > MAX_SAMPLE_SKEW_S:
                    time.sleep(args.update_period); continue
                b_lat,b_lon,b_alt = b.lat,b.lon,b.alt

            # World angles (use raw for determinism while tuning)
            info = tracker.get_tracking_info(b_lat,b_lon,b_alt, d.lat,d.lon,d.alt)
            if not info: time.sleep(args.update_period); continue
            world_az, world_el = info['azimuth'], info['elevation']

            # Apply calibration → pick pose with clean flip policy
            cal_az, cal_el = apply_calibration(world_az, world_el)
            tgt_phys_az, tgt_phys_el, used_flip = pick_pose(cal_az, cal_el, last_saz, last_sel)

            # Rate-limited advance in PHYSICAL space
            az_step_max = AZ_MAX_DEG_PER_SEC * args.update_period
            el_step_max = EL_MAX_DEG_PER_SEC * args.update_period
            d_az = shortest_delta_deg(tgt_phys_az, curr_phys_az)
            d_el = tgt_phys_el - curr_phys_el
            d_az = max(-az_step_max, min(az_step_max, d_az))
            d_el = max(-el_step_max, min(el_step_max, d_el))
            curr_phys_az = wrap360(curr_phys_az + d_az)
            curr_phys_el = max(EL_PHYS_MIN, min(EL_PHYS_MAX, curr_phys_el + d_el))

            # Drive servos
            saz, sel = physical_to_servo_deg(curr_phys_az, curr_phys_el)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, servo_deg_to_us(saz))
            pi.set_servo_pulsewidth(SERVO_EL_PIN, servo_deg_to_us(sel))
            last_saz, last_sel = saz, sel

            # Print & log
            if time.time() >= next_print:
                print(f"[{datetime.now():%H:%M:%S}] {base_mode_str} WORLD {world_az:6.2f}/{world_el:5.2f}  PHYS {curr_phys_az:6.2f}/{curr_phys_el:5.2f}  SERVO {saz:6.2f}/{sel:5.2f}{' FLIP' if used_flip else ''}")
                next_print += args.print_period
            if writer:
                writer.writerow([datetime.now().isoformat(timespec='seconds'), base_mode_str,
                                 round(world_az,3), round(world_el,3), round(cal_az,3), round(cal_el,3),
                                 round(curr_phys_az,3), round(curr_phys_el,3), round(saz,3), round(sel,3), int(used_flip)])

            time.sleep(args.update_period)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
    finally:
        try:
            # Park back to home in WORLD
            sH_az, sH_el = physical_to_servo_deg(*world_to_physical(args.park_home_az, args.park_home_el))
            smooth_park(pi, sH_az, sH_el, duration_s=1.2, rate_hz=60.0)
            time.sleep(0.2)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, 0)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, 0)
            pi.stop()
        except Exception as e:
            print(f"[WARN] pigpio cleanup: {e}")
        if writer:
            try: writer.writerow(["END"]) ; writer = None
            except Exception: pass

if __name__ == "__main__":
    main()
