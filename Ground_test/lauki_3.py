# To be tested with Simulation and on Ground
#!/usr/bin/env python3
import time
from datetime import datetime
import threading
from typing import Optional, Dict, Tuple

import pigpio
from pymavlink import mavutil
import azi_elev_5 as tracker

# =========================================================
# ===================== Configuration =====================
# =========================================================

# --- MAVLink endpoints ---
DRONE_ENDPOINT = "/dev/ttyACM0"
DRONE_BAUD     = 115200
BASE_ENDPOINT  = "/dev/ttyUSB0"
BASE_BAUD      = 57600

# --- 20 Hz request (50_000 µs) ---
MAV_MSG_INTERVAL_US_GPS    = 50_000      # GPS_RAW_INT (ID 24)
MAV_MSG_INTERVAL_US_GLOBAL = 50_000      # GLOBAL_POSITION_INT (ID 33)

# --- Gear and physical limits ---
SPOKES_SMALL = 12
SPOKES_BIG   = 24
GEAR_RATIO   = SPOKES_BIG / SPOKES_SMALL      # == 2.0

AZ_PHYS_MIN = 0.0
AZ_PHYS_MAX = 350.0        # near full rotation with a little safety
EL_PHYS_MIN = 0.0
EL_PHYS_MAX = 180.0

# --- Servo electrical limits – 180° servo ---
PULSE_MIN_US    = 900.0
PULSE_MAX_US    = 2100.0
SERVO_RANGE_DEG = 180.0

# --- Calibration (world-frame) ---
AZIMUTH_ZERO_OFFSET_DEG   = 0.0
ELEVATION_ZERO_OFFSET_DEG = 0.0
AZIMUTH_INVERT   = False
ELEVATION_INVERT = False

# --- GPIO pins (BCM) ---
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 13

# --- Rates & filtering ---
UPDATE_PERIOD_S = 0.05      # 20 Hz loop
GPS_STALE_SEC   = 0.5       # drop samples older than 0.5 s

# --- Motion shaping ---
MAX_AZ_SLEW_DPS = 180.0     # azimuth max speed (deg/sec)
MAX_EL_SLEW_DPS = 120.0     # elevation max speed (deg/sec)
BACKLASH_DEG    = 0.5       # small nudge on reversal (set 0.0 to disable)

# --- Logging / prints ---
PRINT_EVERY = 1              # print every N cycles (1=20 Hz; try 4 for ~5 Hz)
LOG_TO_CSV  = True

# --- Startup behaviour ---
STARTUP_PARK_ZERO = True     # park to 0°/0° on startup
STARTUP_PARK_DWELL_S = 0.5

# =========================================================
# ===================== Shared State ======================
# =========================================================

# Add timestamps 't' to guard against stale GPS
drone_gps = {"lat": None, "lon": None, "alt": None, "t": None}
base_gps  = {"lat": None, "lon": None, "alt": None, "t": None}

_drone_lock = threading.Lock()
_base_lock  = threading.Lock()
stop_event  = threading.Event()

# =========================================================
# ===================== Small helpers =====================
# =========================================================

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

def signed_angle_err(a_desired: float, a_cmd: float) -> float:
    """Shortest signed angle (desired - cmd) in [-180..+180] deg."""
    return (a_desired - a_cmd + 540.0) % 360.0 - 180.0

# --- Motion shapers ----------------------------------------------------------

def shortest_delta_deg(target: float, current: float) -> float:
    """Signed shortest angle (target - current) in degrees, range [-180..+180]."""
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
    """Elevation shaper: linear (no wrap), with slew limit and clamping to [min,max]."""
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

# =========================================================
# ===================== pigpio setup ======================
# =========================================================

def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running. Start with: sudo pigpiod")
    pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
    print(f"[INFO] pigpio ready (pins AZ={SERVO_AZ_PIN}, EL={SERVO_EL_PIN})")
    return pi

def park_servos_zero(pi: pigpio.pi, dwell_s: float = 0.5):
    """Immediately drive both servos to 0° (servo degrees), then dwell."""
    us_az = servo_deg_to_us(0.0)
    us_el = servo_deg_to_us(0.0)
    pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
    pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
    print(f"[INIT] Parked servos to 0°/0° (µs {us_az:.0f}/{us_el:.0f}); dwell {dwell_s:.1f}s")
    time.sleep(dwell_s)

# =========================================================
# ===================== MAVLink I/O =======================
# =========================================================

def connect_mavlink(endpoint: str, baud: Optional[int] = None, require_heartbeat: bool = True) -> mavutil.mavfile:
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
        print(f"[MAV] No heartbeat on {endpoint} (continuing anyway): {e}")
    # Ask for high-rate messages
    for msg_id, interval in (
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,    MAV_MSG_INTERVAL_US_GPS),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAV_MSG_INTERVAL_US_GLOBAL),
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

def start_reader(mav: mavutil.mavfile, target_dict: Dict[str, Optional[float]], lock: threading.Lock):
    def _run():
        while not stop_event.is_set():
            msg = mav.recv_match(type=["GPS_RAW_INT", "GLOBAL_POSITION_INT"], blocking=True, timeout=0.2)
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

def snapshot_positions() -> Optional[Tuple[Dict[str, float], Dict[str, float]]]:
    with _drone_lock: d = drone_gps.copy()
    with _base_lock:  b = base_gps.copy()
    if None in (d["lat"], d["lon"], d["alt"], d["t"], b["lat"], b["lon"], b["alt"], b["t"]):
        return None
    now = time.time()
    if (now - d["t"] > GPS_STALE_SEC) or (now - b["t"] > GPS_STALE_SEC):
        return None
    return d, b

# =========================================================
# ===================== CSV logging =======================
# =========================================================

_log_writer = None
_log_file = None

def log_open():
    global _log_writer, _log_file
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
        "WorldAz","WorldEl",
        "PhysAz_tgt","PhysEl_tgt",
        "PhysAz_cmd","PhysEl_cmd",
        "ServoAz_deg","ServoEl_deg",
        "PulseAz_us","PulseEl_us",
        "DroneLat","DroneLon","DroneAlt","AgeD_ms",
        "BaseLat","BaseLon","BaseAlt","AgeB_ms"
    ])
    print(f"[INFO] CSV log: {fn}")

def log_row(*row):
    if LOG_TO_CSV and _log_writer:
        _log_writer.writerow(row)
        _log_file.flush()

def log_close():
    if LOG_TO_CSV and _log_file:
        _log_file.close()

# =========================================================
# ===================== Main Program ======================
# =========================================================

def main():
    print("=== Antenna Tracker (20 Hz, unwrap+slew, 2:1 gear, 180° servos) ===")
    pi = setup_pigpio()
    log_open()

    if STARTUP_PARK_ZERO:
        park_servos_zero(pi, dwell_s=STARTUP_PARK_DWELL_S)

    mav_drone = connect_mavlink(DRONE_ENDPOINT, DRONE_BAUD, require_heartbeat=True)
    mav_base  = connect_mavlink(BASE_ENDPOINT,  BASE_BAUD,  require_heartbeat=False)

    t_drone = start_reader(mav_drone, drone_gps, _drone_lock)
    t_base  = start_reader(mav_base,  base_gps,  _base_lock)

    # Motion shapers
    az_shaper = AxisShaper360(MAX_AZ_SLEW_DPS, BACKLASH_DEG)
    el_shaper = AxisShaperLinear(MAX_EL_SLEW_DPS, EL_PHYS_MIN, EL_PHYS_MAX, BACKLASH_DEG)

    cycle = 0
    last = time.time()
    try:
        while not stop_event.is_set():
            now = time.time()
            dt = max(1e-3, now - last)
            last = now

            snap = snapshot_positions()
            if not snap:
                if (cycle % max(1, PRINT_EVERY)) == 0:
                    print("[WAIT] Waiting for recent GPS from both endpoints…")
                cycle += 1
                # pace loop
                sl = UPDATE_PERIOD_S - (time.time() - now)
                if sl > 0: time.sleep(sl)
                continue

            drone, base = snap
            age_d = now - drone["t"]
            age_b = now - base["t"]

            info = tracker.get_tracking_info(
                base["lat"], base["lon"], base["alt"],
                drone["lat"], drone["lon"], drone["alt"]
            )
            if not info:
                # pace loop
                sl = UPDATE_PERIOD_S - (time.time() - now)
                if sl > 0: time.sleep(sl)
                continue

            world_az = info.get("adjusted_azimuth",  info["azimuth"])
            world_el = info.get("adjusted_elevation", info["elevation"])

            # Calibration → physical target
            cal_az, cal_el     = apply_calibration(world_az, world_el)
            phys_az_t, phys_el_t = world_to_physical(cal_az, cal_el)

            # Shaped physical command
            phys_az_cmd = az_shaper.step(phys_az_t, dt)
            phys_el_cmd = el_shaper.step(phys_el_t, dt)

            # Servo mapping
            s_az, s_el   = physical_to_servo_deg(phys_az_cmd, phys_el_cmd)
            pulse_az     = servo_deg_to_us(s_az)
            pulse_el     = servo_deg_to_us(s_el)

            # Drive servos
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, pulse_az)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, pulse_el)

            if (cycle % PRINT_EVERY) == 0:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                      f"WORLD {world_az:6.2f}/{world_el:5.2f}°  "
                      f"PHYS tgt {phys_az_t:6.2f}/{phys_el_t:5.2f}°  "
                      f"PHYS cmd {phys_az_cmd:6.2f}/{phys_el_cmd:5.2f}°  "
                      f"SERVO {s_az:6.2f}/{s_el:5.2f}°  "
                      f"µs {pulse_az:5.0f}/{pulse_el:5.0f}  "
                      f"Age D/B {age_d*1000:4.0f}/{age_b*1000:4.0f} ms")

            log_row(
                datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                round(world_az,3), round(world_el,3),
                round(phys_az_t,3), round(phys_el_t,3),
                round(phys_az_cmd,3), round(phys_el_cmd,3),
                round(s_az,3), round(s_el,3),
                round(pulse_az,1), round(pulse_el,1),
                round(drone["lat"],7), round(drone["lon"],7), round(drone["alt"],2), round(age_d*1000,0),
                round(base["lat"],7),  round(base["lon"],7),  round(base["alt"],2),  round(age_b*1000,0)
            )

            cycle += 1
            # pace to 20 Hz precisely
            sl = UPDATE_PERIOD_S - (time.time() - now)
            if sl > 0:
                time.sleep(sl)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")

    finally:
        stop_event.set()
        try:
            # Park safely to 0°/0°, then release PWM
            park_servos_zero(pi, dwell_s=0.2)
            pi.set_servo_pulsewidth(SERVO_AZ_PIN, 0)
            pi.set_servo_pulsewidth(SERVO_EL_PIN, 0)
            pi.stop()
        except Exception as e:
            print(f"[WARN] pigpio cleanup: {e}")
        log_close()

if __name__ == "__main__":
    main()
