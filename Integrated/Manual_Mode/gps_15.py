# geo_13_full.py — FastAPI + background tracker thread (runs together cleanly)

import argparse
import asyncio
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional, Tuple, Callable, Dict, Any
import copy
import pigpio
from pymavlink import mavutil
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Body, HTTPException
from fastapi.responses import HTMLResponse, Response
from fastapi.staticfiles import StaticFiles
import os, sqlite3
from fastapi import HTTPException
from fastapi.responses import Response
import azi_elev_5 as tracker
import sqlite3, os, struct

# ===================== FastAPI app =====================

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

# ---------- Shared state for UI ----------
MBTILES_PATH = "maps/local_2.mbtiles"  # put your path here
latest_gps_data: Dict[str, Any] = {
    "base":   {"lat": 0.0, "lon": 0.0, "alt": 0.0, "eph": None, "epv": None, "fix_type": None, "sats": None, "timestamp": None},
    "drone":  {"lat": 0.0, "lon": 0.0, "alt": 0.0, "eph": None, "epv": None, "fix_type": None, "sats": None, "timestamp": None},
    "tracker":{"mode": "auto", "azimuth": 0.0, "elevation": 0.0, "phys_az": 0.0, "phys_el": 0.0}
}
EVENT_LOOP: asyncio.AbstractEventLoop | None = None

# ===== Local offline map (MBTiles) =====
MBTILES_PATH = "maps/local.mbtiles"   # put your .mbtiles here
_mb_conn: sqlite3.Connection | None = None
_mb_format: str = "png"               # will read from metadata if present


# ---------- WS manager ----------
class ConnectionManager:
    def __init__(self):
        self.active: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket):
        await ws.accept()
        async with self._lock:
            self.active.add(ws)

    def disconnect(self, ws: WebSocket):
        if ws in self.active:
            self.active.remove(ws)

    async def broadcast(self, payload: Dict[str, Any]):
        dead = []
        for ws in list(self.active):
            try:
                await ws.send_json(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)

manager = ConnectionManager()

def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()

def _sample_to_dict(sample) -> Dict[str, Any]:
    return {
        "lat": float(getattr(sample, "lat", 0.0)),
        "lon": float(getattr(sample, "lon", 0.0)),
        "alt": float(getattr(sample, "alt", 0.0)),
        "eph": getattr(sample, "eph", None),
        "epv": getattr(sample, "epv", None),
        "fix_type": getattr(sample, "fix_type", None),
        "sats": getattr(sample, "sats", None),
        "timestamp": getattr(sample, "timestamp", None) or _now_iso(),
    }

async def _push_ws_update():
    #with state_lock:
    payload = copy.deepcopy(latest_gps_data)
    await manager.broadcast(payload)

def _schedule_ws_update():
    if EVENT_LOOP and EVENT_LOOP.is_running():
        asyncio.run_coroutine_threadsafe(_push_ws_update(), EVENT_LOOP)

# ---------- WebSocket ----------
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        # initial snapshot
        #with state_lock:
        payload = copy.deepcopy(latest_gps_data)
        await websocket.send_json(payload)
        # periodic keep-alive
        while True:
            await asyncio.sleep(0.1)
            #with state_lock:
            payload = copy.deepcopy(latest_gps_data)
            await manager.broadcast(payload)
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# ---------- REST ----------
@app.get("/api/gps")
async def get_gps_data():
    #with state_lock:
    return copy.deepcopy(latest_gps_data)
@app.get("/")
async def get_ui():
    with open("ui.html", "r", encoding="utf-8") as f:
        html_content = f.read()
    return HTMLResponse(content=html_content, status_code=200)

@app.post("/api/manual_control")
async def api_manual_control(payload: dict = Body(...)):
    """
    Payload shapes:
      {"action":"mode_toggle"}
      {"action":"move","azimuth_delta": <deg>, "elevation_delta": <deg>}
    """
    global MANUAL_MODE
    action = (payload.get("action") or "").lower()

    if action == "mode_toggle":
        MANUAL_MODE = not MANUAL_MODE
        latest_gps_data["tracker"]["mode"] = "manual" if MANUAL_MODE else "auto"
        return {"ok": True, "mode": latest_gps_data["tracker"]["mode"]}

    if action == "move":
        daz = float(payload.get("azimuth_delta", 0.0) or 0.0)
        delv = float(payload.get("elevation_delta", 0.0) or 0.0)
        with MANUAL_LOCK:
            MANUAL_CMD["daz"] += daz
            MANUAL_CMD["del"] += delv
        return {"ok": True}

    return {"ok": False, "error": "unknown action"}

def _flip_y_slippy_to_tms(z, y):
    return (1 << z) - 1 - y

def _open_mb():
    if not os.path.exists(MBTILES_PATH):
        raise FileNotFoundError(MBTILES_PATH)
    conn = sqlite3.connect(MBTILES_PATH, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    return conn

_mb_conn = _open_mb()


@app.post("/api/calibrate")
async def api_calibrate():
    """
    One-tap zero-ref:
    Capture the *current computed world* az/el and shift offsets so that
    future tracking uses that as zero reference.
    """
    global AZIMUTH_ZERO_OFFSET_DEG, ELEVATION_ZERO_OFFSET_DEG

    world_az = float(latest_gps_data["tracker"].get("azimuth", 0.0) or 0.0)
    world_el = float(latest_gps_data["tracker"].get("elevation", 0.0) or 0.0)

    # Shift world so the current direction becomes (0, 0) after apply_calibration()
    AZIMUTH_ZERO_OFFSET_DEG   = norm360(-world_az)
    ELEVATION_ZERO_OFFSET_DEG = -world_el

    return {
        "ok": True,
        "new_offsets": {
            "azimuth_zero_offset_deg": AZIMUTH_ZERO_OFFSET_DEG,
            "elevation_zero_offset_deg": ELEVATION_ZERO_OFFSET_DEG,
        }
    }

# ===================== Tracker config & constants =====================

MODE_DEFAULT = "ground"          # "sim" or "ground"
BASE_MODE_DEFAULT = "static"     # "static" or "dynamic"

SIM_DRONE_ENDPOINT = "udp:0.0.0.0:14550"
SIM_DRONE_BAUD     = None
DRONE_ENDPOINT     = "/dev/ttyUSB0"
DRONE_BAUD         = 57600
BASE_ENDPOINT      = "/dev/ttyACM0"
BASE_BAUD          = 57600

MAV_MSG_INTERVAL_US_GPS    = 200_000   # 5Hz
MAV_MSG_INTERVAL_US_GLOBAL = 50_000    # 20Hz (if supported)

SPOKES_SMALL   = 12
SPOKES_BIG     = 24
AZ_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL
EL_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL

AZ_PHYS_MIN = 0.0
AZ_PHYS_MAX = 360.0
EL_PHYS_MIN = 0.0
EL_PHYS_MAX = 180.0

PULSE_MIN_US    = 900.0
PULSE_MAX_US    = 1950.0
SERVO_RANGE_DEG = 180.0

AZIMUTH_ZERO_OFFSET_DEG   = 0.0
ELEVATION_ZERO_OFFSET_DEG = 0.0
AZIMUTH_INVERT   = True
ELEVATION_INVERT = False

SERVO_AZ_PIN = 18
SERVO_EL_PIN = 17

UPDATE_PERIOD_S = 0.05
PRINT_PERIOD_S  = 1.0
LOG_TO_CSV      = False
LOG_RAW_GPS     = False

base_static = {"lat": 13.0276802, "lon": 77.5629616, "alt": 924.36}

AZ_MAX_DEG_PER_SEC   = 180.0
EL_MAX_DEG_PER_SEC   = 180.0

MIN_FLIP_DWELL_S            = 2
MIN_AZ_DELTA_SINCE_FLIP_DEG = 20.0
FLIP_EXTRA_MARGIN_DEG       = 12.0

ALLOW_BACKSIDE_FLIP = True
EL_DEADBAND_DEG = 1.5
EL_MAX_DEG_PER_SEC = 60.0

ONLY_FLIP_NEAR_EDGE  = True
EDGE                 = 10.0
FLIP_HYSTERESIS_DEG  = 14.0
MIN_EL_FOR_FLIP      = 6.0

AZ_WEIGHT = 1.0
EL_WEIGHT = 0.30
FLIP_STYLE = "keep_el"
FLIP_AZ_CORR_DEG = 0.0
FLIP_EL_CORR_DEG = 0.0

MANUAL_LOCK = threading.Lock()
MANUAL_MODE = False
MANUAL_CMD = {"daz": 0.0, "del": 0.0}  # accumulated deltas from joystick

# Make calibration offsets mutable (leave names as-is so apply_calibration keeps working)
AZIMUTH_ZERO_OFFSET_DEG   = 0.0   # was constant; keep as global float
ELEVATION_ZERO_OFFSET_DEG = 0.0   # was constant; keep as global float

def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

def wrap360(x: float) -> float:
    return (x + 360.0) % 360.0

def shortest_delta_deg(target: float, current: float) -> float:
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

def _bounded_us(x: float) -> float:
    return max(500.0, min(2500.0, x))

def servo_deg_to_us(deg: float) -> float:
    mn = float(PULSE_MIN_US)
    mx = float(PULSE_MAX_US)
    rng = float(SERVO_RANGE_DEG)
    d = max(0.0, min(rng, float(deg)))
    return mn + (d / rng) * (mx - mn)

# Custom az mapping (0..360 phys -> μs)
def az_phys_to_us(az_phys: float) -> float:
    az = max(0.0, min(360.0, float(az_phys)))
    us = 900.0 + (az * az) / 1296.0 + (95.0 / 36.0) * az
    return _bounded_us(us)

def servo_deg_to_us_az(servo_deg: float) -> float:
    d = max(0.0, min(SERVO_RANGE_DEG, float(servo_deg)))
    phys_az = d * float(AZ_GEAR_RATIO)
    return az_phys_to_us(phys_az)

def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running (sudo pigpiod)")
    pi.set_mode(SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_EL_PIN, pigpio.OUTPUT)
    print(f"[INFO] pigpio ready (AZ={SERVO_AZ_PIN}, EL={SERVO_EL_PIN})")
    return pi

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
    t_az_deg = max(0.0, min(SERVO_RANGE_DEG, float(target_servo_deg_az)))
    t_el_deg = max(0.0, min(SERVO_RANGE_DEG, float(target_servo_deg_el)))
    target_us_az = servo_deg_to_us_az(t_az_deg)
    target_us_el = servo_deg_to_us(t_el_deg)
    start_us_az = _get_start_us(pi, SERVO_AZ_PIN, default_us=target_us_az)
    start_us_el = _get_start_us(pi, SERVO_EL_PIN, default_us=target_us_el)
    DEAD_US = 8.0
    d_az = abs(target_us_az - start_us_az)
    d_el = abs(target_us_el - start_us_el)
    if d_az <= DEAD_US and d_el <= DEAD_US:
        pi.set_servo_pulsewidth(SERVO_AZ_PIN, target_us_az)
        pi.set_servo_pulsewidth(SERVO_EL_PIN, target_us_el)
        return
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

def _xyz_to_tms_y(z: int, y: int) -> int:
    # MBTiles uses TMS (origin bottom-left); Leaflet uses XYZ (origin top-left)
    return (1 << z) - 1 - y

def _open_mbtiles():
    global _mb_conn, _mb_format
    if not os.path.exists(MBTILES_PATH):
        print(f"[MAP] No MBTiles found at {MBTILES_PATH} (map layer will be unavailable)")
        return
    _mb_conn = sqlite3.connect(MBTILES_PATH, check_same_thread=False)
    _mb_conn.row_factory = sqlite3.Row
    try:
        row = _mb_conn.execute("SELECT value FROM metadata WHERE name='format'").fetchone()
        if row and row["value"]:
            val = row["value"].lower()
            _mb_format = "jpg" if val in ("jpeg", "jpg") else val
    except Exception:
        pass
    print(f"[MAP] MBTiles ready: {MBTILES_PATH} (format={_mb_format})")
# ===================== GPS samples & threads =====================
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
    global _latest_base, latest_gps_data
    data = _sample_to_dict(sample)
    with _base_lock:
        _latest_base = sample
    #with state_lock:
        latest_gps_data["base"] = data
    _schedule_ws_update()

def set_latest_drone(sample: GpsSample):
    global _latest_drone, latest_gps_data
    data = _sample_to_dict(sample)
    with _drone_lock:
        _latest_drone = sample
    #with state_lock:
        latest_gps_data["drone"] = data
    _schedule_ws_update()

def get_latest_base() -> Optional[GpsSample]:
    with _base_lock:
        return _latest_base

def get_latest_drone() -> Optional[GpsSample]:
    with _drone_lock:
        return _latest_drone

def connect_mav(endpoint: str, baud: Optional[int], hb_required: bool) -> mavutil.mavfile:
    """
    Hardened open (tries alternates & disables DTR/RTS).
    """
    candidates = [endpoint]
    if endpoint.startswith("/dev/tty"):
        candidates += ["/dev/ttyUSB1", "/dev/ttyAMA0", "/dev/serial0"]
    last_err = None
    for dev in candidates:
        try:
            mav = (mavutil.mavlink_connection(
                        dev, baud=baud, autoreconnect=True,
                        rtscts=False, dsrdtr=False, xonxoff=False)
                   if baud else
                   mavutil.mavlink_connection(
                        dev, autoreconnect=True,
                        rtscts=False, dsrdtr=False, xonxoff=False))
            try:
                mav.wait_heartbeat(timeout=5)
                print(f"[MAV] Connected {dev} sys={mav.target_system} comp={mav.target_component}")
            except Exception as e:
                if hb_required:
                    raise
                print(f"[MAV] No heartbeat on {dev} – continuing: {e}")
            # request streams (best-effort)
            for msg_id, interval in (
                (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,         MAV_MSG_INTERVAL_US_GPS),
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
        except Exception as e:
            print(f"[MAV] Open failed on {dev}: {e}")
            last_err = e
    raise last_err

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

# ===================== Flip chooser =====================
def choose_flipped_if_better(A_saz, B_saz, cost_A, cost_B, only_flip_near_edge=True):
    now = time.time()
    st = choose_flipped_if_better.__dict__
    last_flip_t = st.get("_last_flip_t", 0.0)
    last_flip_saz = st.get("_last_flip_saz", None)

    def near_edge(saz: float) -> bool:
        return (saz <= EDGE) or (saz >= (180.0 - EDGE))

    allow_flip = True
    if only_flip_near_edge:
        allow_flip = near_edge(A_saz) or near_edge(B_saz)
    if (now - last_flip_t) < MIN_FLIP_DWELL_S:
        allow_flip = False
    if last_flip_saz is not None:
        if abs(B_saz - last_flip_saz) < MIN_AZ_DELTA_SINCE_FLIP_DEG:
            allow_flip = False

    thresholded_cost_B = cost_B + FLIP_HYSTERESIS_DEG + FLIP_EXTRA_MARGIN_DEG
    if allow_flip and (thresholded_cost_B < cost_A):
        st["_last_flip_t"] = now
        st["_last_flip_saz"] = B_saz
        return True, B_saz
    else:
        return False, A_saz

def pick_target(cal_az, cal_el, last_servo_az, last_servo_el):
    if not ALLOW_BACKSIDE_FLIP:
        A_phys_az, A_phys_el = world_to_physical(cal_az, cal_el)
        return A_phys_az, A_phys_el, False

    A_world_az, A_world_el = cal_az, cal_el
    B_world_az = wrap360(cal_az + 180.0)
    if FLIP_STYLE == "mirror_el":
        B_world_el = max(EL_PHYS_MIN, min(EL_PHYS_MAX, 180.0 - cal_el))
    else:
        B_world_el = cal_el

    def _apply_flip_corr(az, el):
        return wrap360(az + FLIP_AZ_CORR_DEG), max(EL_PHYS_MIN, min(EL_PHYS_MAX, el + FLIP_EL_CORR_DEG))

    A_phys_az, A_phys_el = world_to_physical(A_world_az, A_world_el)
    B_phys_az, B_phys_el = world_to_physical(B_world_az, B_world_el)
    A_saz, A_sel = physical_to_servo_deg(A_phys_az, A_phys_el)
    B_saz, B_sel = physical_to_servo_deg(B_phys_az, B_phys_el)

    cost_A = AZ_WEIGHT * abs(A_saz - last_servo_az) + EL_WEIGHT * abs(A_sel - last_servo_el)
    cost_B = AZ_WEIGHT * abs(B_saz - last_servo_az) + EL_WEIGHT * abs(B_sel - last_servo_el)

    if B_world_el < MIN_EL_FOR_FLIP:
        return A_phys_az, A_phys_el, False

    used_flip, _ = choose_flipped_if_better(A_saz, B_saz, cost_A, cost_B, only_flip_near_edge=ONLY_FLIP_NEAR_EDGE)
    if used_flip:
        B_world_az_corr, B_world_el_corr = _apply_flip_corr(B_world_az, B_world_el)
        B_phys_az, B_phys_el = world_to_physical(B_world_az_corr, B_world_el_corr)
        return B_phys_az, B_phys_el, True
    else:
        return A_phys_az, A_phys_el, False

# ===================== CSV logging (minimal) =====================
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
            "Time","BaseMode","BaseLocked",
            "WorldAz","WorldEl","CalAz","CalEl",
            "PhysAz","PhysEl","ServoAz","ServoEl",
            "Az(us)","El(us)",
            "DroneLat","DroneLon","DroneAlt",
            "BaseLat","BaseLon","BaseAlt",
            "BaseFix","BaseSats","UsedFlip"
        ])
        print(f"[INFO] CSV log → {fn}")

def log_row(*row):
    if _log_writer:
        _log_writer.writerow(row)
        if _log_file:
            _log_file.flush()

def log_close():
    if _log_file: _log_file.close()

# ===================== Tracker thread runner =====================

@dataclass
class TrackerConfig:
    mode: str = MODE_DEFAULT            # "sim" | "ground"
    base_mode: str = BASE_MODE_DEFAULT  # "static" | "dynamic"
    static_window_sec: float = 10.0
    print_period: float = PRINT_PERIOD_S
    update_period: float = UPDATE_PERIOD_S
    park_home_az: float = 0.0
    park_home_el: float = 90.0
    park_face_drone_start: bool = False
    park_face_drone_exit: bool = False
    park_duration: float = 1.5
    park_rate_hz: float = 60.0

class TrackerRunner:
    def __init__(self, cfg: TrackerConfig):
        self.cfg = cfg
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._started = threading.Event()

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._started.wait(timeout=5)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)

    def _run(self):
        self._started.set()
        cfg = self.cfg

        print("=== tracker thread — static/dynamic base, zero-ref, smooth parking ===")
        print(f"[CFG] Mode: {cfg.mode} | BaseMode: {cfg.base_mode} | Gear AZ {AZ_GEAR_RATIO}:1, EL {EL_GEAR_RATIO}:1 | Servo 180° @ {PULSE_MIN_US}-{PULSE_MAX_US}µs")
        print(f"[CFG] update_period={cfg.update_period:.2f}s, print_period={cfg.print_period:.1f}s")
        print(f"[CFG] parking: home=({cfg.park_home_az:.1f}°, {cfg.park_home_el:.1f}°) "
              f"face_drone(start={cfg.park_face_drone_start}, exit={cfg.park_face_drone_exit}) "
              f"duration={cfg.park_duration:.2f}s @ {cfg.park_rate_hz:.0f} Hz")

        try:
            pi = setup_pigpio()
        except Exception as e:
            print(f"[ERROR] pigpio: {e}")
            return

        log_open(prefix="Tracker")

        # Park to home (world 0/park_home_el)
        phys_az0, phys_el0 = world_to_physical(cfg.park_home_az, cfg.park_home_el)
        s_az0, s_el0 = physical_to_servo_deg(phys_az0, phys_el0)
        smooth_park(pi, s_az0, s_el0, duration_s=cfg.park_duration, rate_hz=cfg.park_rate_hz)
        time.sleep(0.1)

        last_servo_az = float(s_az0)
        last_servo_el = float(s_el0)
        curr_phys_az = phys_az0
        curr_phys_el = phys_el0

        # Start MAV readers
        try:
            if cfg.mode == "sim":
                mav_drone = connect_mav(SIM_DRONE_ENDPOINT, SIM_DRONE_BAUD, True)
                start_reader(mav_drone, "DRONE", on_raw=None)
                print(f"[CFG] SIM base @ {base_static['lat']}, {base_static['lon']}, {base_static['alt']}m")
            else:
                mav_drone = connect_mav(DRONE_ENDPOINT, DRONE_BAUD, True)
                start_reader(mav_drone, "DRONE", on_raw=None)
                try:
                    mav_base  = connect_mav(BASE_ENDPOINT,  BASE_BAUD,  False)
                    start_reader(mav_base,  "BASE",  on_raw=None)
                except Exception as e:
                    print(f"[WARN] Base stream unavailable: {e}")
        except Exception as e:
            print(f"[ERROR] MAVLink connect: {e}")
            # Optional: fall back to SIM
            # cfg.mode = "sim"

        print("[INFO] Point the tracker at the drone and stabilize GPS.")
        time.sleep(4.0)

        base_fixed = None
        next_print = time.time()

        try:
            while not self._stop.is_set():
                d = get_latest_drone()
                if not d:
                    time.sleep(0.01); continue
            # --- Manual override: consume joystick deltas and move servos ---
                if MANUAL_MODE:
                    # Pull and clear accumulated deltas atomically
                    with MANUAL_LOCK:
                        daz = MANUAL_CMD["daz"]; MANUAL_CMD["daz"] = 0.0
                        delv = MANUAL_CMD["del"]; MANUAL_CMD["del"] = 0.0

                    # Scale/sanitize (optional): you can keep joystick deltas small; here we just apply directly
                    if daz or delv:
                        curr_phys_az = wrap360(curr_phys_az + daz)
                        curr_phys_el = max(EL_PHYS_MIN, min(EL_PHYS_MAX, curr_phys_el + delv))

                        s_az, s_el = physical_to_servo_deg(curr_phys_az, curr_phys_el)
                        us_az = servo_deg_to_us_az(s_az)
                        us_el = servo_deg_to_us(s_el)

                        try:
                            pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
                            pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
                        except Exception as e:
                            print(f"[WARN] pigpio write (manual): {e}")

                        latest_gps_data["tracker"]["phys_az"] = curr_phys_az
                        latest_gps_data["tracker"]["phys_el"] = curr_phys_el

                    time.sleep(cfg.update_period)
                    continue  # skip auto-tracking this cycle


                # Base selection
                if cfg.mode == "sim":
                    b_lat, b_lon, b_alt = base_static["lat"], base_static["lon"], base_static["alt"]
                    base_mode_str = "static(SIM)"
                    base_fix = base_sats = None
                    base_locked = True
                else:
                    if cfg.base_mode == "static":
                        # Freeze to average for window
                        if base_fixed is None:
                            t_end = time.time() + float(cfg.static_window_sec)
                            lats, lons, alts = [], [], []
                            while time.time() < t_end and not self._stop.is_set():
                                b_try = get_latest_base()
                                if b_try:
                                    lats.append(b_try.lat); lons.append(b_try.lon); alts.append(b_try.alt)
                                time.sleep(0.05)
                            if lats:
                                base_fixed = (sum(lats)/len(lats), sum(lons)/len(lons), sum(alts)/len(alts))
                                print(f"[INFO] Base frozen avg: lat={base_fixed[0]:.7f}, lon={base_fixed[1]:.7f}, alt={base_fixed[2]:.2f}")
                            else:
                                # If still none: freeze on first available
                                b_now = get_latest_base()
                                if not b_now:
                                    time.sleep(0.01); continue
                                base_fixed = (b_now.lat, b_now.lon, b_now.alt)
                                print(f"[INFO] Base frozen late: lat={base_fixed[0]:.7f}, lon={base_fixed[1]:.7f}, alt={base_fixed[2]:.2f}")
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

                # Angles
                info = tracker.get_tracking_info(b_lat, b_lon, b_alt, d.lat, d.lon, d.alt)
                if not info:
                    time.sleep(cfg.update_period); continue

                world_az = info['azimuth']
                world_el = info['elevation']

                # Update UI tracker preview (optional live numbers)
                latest_gps_data["tracker"]["azimuth"] = world_az
                latest_gps_data["tracker"]["elevation"] = world_el

                # Calibration & flip logic
                cal_az, cal_el = apply_calibration(world_az, world_el)
                tgt_phys_az, tgt_phys_el, used_flip = pick_target(cal_az, cal_el, last_servo_az, last_servo_el)

                # Rate limits
                az_step_max = float(AZ_MAX_DEG_PER_SEC) * float(cfg.update_period)
                el_step_max = float(EL_MAX_DEG_PER_SEC) * float(cfg.update_period)

                d_az = shortest_delta_deg(tgt_phys_az, curr_phys_az)
                d_el = tgt_phys_el - curr_phys_el
                if abs(d_el) < EL_DEADBAND_DEG: d_el = 0.0
                d_az = max(-az_step_max, min(az_step_max, d_az))
                d_el = max(-el_step_max, min(el_step_max, d_el))

                curr_phys_az = wrap360(curr_phys_az + d_az)
                curr_phys_el = max(EL_PHYS_MIN, min(EL_PHYS_MAX, curr_phys_el + d_el))

                s_az, s_el = physical_to_servo_deg(curr_phys_az, curr_phys_el)
                us_az      = servo_deg_to_us_az(s_az)
                us_el      = servo_deg_to_us(s_el)

                # Drive servos
                try:
                    pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
                    pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
                except Exception as e:
                    print(f"[WARN] pigpio write: {e}")

                last_servo_az, last_servo_el = s_az, s_el

                # expose phys pose to UI
                latest_gps_data["tracker"]["phys_az"] = curr_phys_az
                latest_gps_data["tracker"]["phys_el"] = curr_phys_el

                if time.time() >= next_print:
                    print(f"[{datetime.now():%H:%M:%S}] Base={base_mode_str} "
                          f"WORLD {world_az:6.2f}/{world_el:5.2f}° | "
                          f"PHYS {curr_phys_az:6.2f}/{curr_phys_el:5.2f}° | "
                          f"SERVO {s_az:6.2f}/{s_el:6.2f}° | us {us_az:5.0f}/{us_el:5.0f}"
                          f"{' | FLIP' if used_flip else ''}")
                    next_print += float(cfg.print_period)

                # CSV log (minimal)
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
                    "" if base_fix is None else base_fix,
                    "" if base_sats is None else base_sats,
                    1 if used_flip else 0
                )

                time.sleep(cfg.update_period)

        except Exception as e:
            print(f"[ERROR] Tracker loop: {e}")
        finally:
            print("[INFO] Smooth shutdown: parking…")
            try:
                cal_azH, cal_elH = apply_calibration(0.0, 90.0)
                phys_azH, phys_elH = world_to_physical(cal_azH, cal_elH)
                s_azH, s_elH = physical_to_servo_deg(phys_azH, phys_elH)
                smooth_park(pi, s_azH, s_elH, duration_s=1.5, rate_hz=60.0)
                time.sleep(0.2)
                pi.set_servo_pulsewidth(SERVO_AZ_PIN, 0)
                pi.set_servo_pulsewidth(SERVO_EL_PIN, 0)
                pi.stop()
            except Exception as e:
                print(f"[WARN] pigpio cleanup: {e}")
            log_close()

# ===================== FastAPI lifecycle: start/stop tracker =====================
TRACKER_CONFIG: TrackerConfig | None = None
tracker_runner: TrackerRunner | None = None

@app.on_event("startup")
async def _startup():
    global EVENT_LOOP, tracker_runner
    EVENT_LOOP = asyncio.get_running_loop()
    _open_mbtiles()  # <-- add this line
    cfg = TRACKER_CONFIG or TrackerConfig()
    tracker_runner = TrackerRunner(cfg)
    tracker_runner.start()

@app.on_event("shutdown")
async def _shutdown():
    if tracker_runner:
        tracker_runner.stop()
    global _mb_conn
    if _mb_conn is not None:
        try:
            _mb_conn.close()
            print("[MAP] MBTiles closed")
        except Exception:
            pass
        _mb_conn = None


@app.get("/tiles/{z}/{x}/{y}.pbf")
def get_vector_tile(z: int, x: int, y: int):
    try:
        tms_y = _flip_y_slippy_to_tms(z, y)
        cur = _mb_conn.cursor()
        cur.execute(
            "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
            (z, x, tms_y),
        )
        row = cur.fetchone()
        if not row:
            raise HTTPException(status_code=204, detail="No tile")
        data = row["tile_data"]

        # Many MBTiles store MVT compressed with gzip.
        # If starts with gzip magic (0x1f,0x8b), set Content-Encoding.
        is_gzip = len(data) >= 2 and data[0] == 0x1F and data[1] == 0x8B
        headers = {}
        if is_gzip:
            headers["Content-Encoding"] = "gzip"

        return Response(
            content=data,
            media_type="application/x-protobuf",
            headers=headers,
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/tiles/{z}/{x}/{y}.jpg")
async def get_tile_jpg(z: int, x: int, y: int):
    return await _get_tile(z, x, y, force_format="jpg")

async def _get_tile(z: int, x: int, y: int):
    if _mb_conn is None:
        raise HTTPException(status_code=404, detail="MBTiles not loaded")

    # Determine format once from metadata
    fmt = (_mb_format or "png").lower()
    if fmt in ("pbf", "mvt"):
        raise HTTPException(status_code=415, detail="Vector MBTiles detected; raster tiles expected")

    tms_y = _xyz_to_tms_y(z, y)
    row = _mb_conn.execute(
        "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
        (z, x, tms_y)
    ).fetchone()
    if not row:
        raise HTTPException(status_code=404, detail="Tile not found")

    media = "image/jpeg" if fmt in ("jpg", "jpeg") else "image/png"
    # Optional long cache; safe for immutable tiles
    headers = {"Cache-Control": "public, max-age=31536000, immutable"}
    return Response(bytes(row["tile_data"]), media_type=media, headers=headers)

# ===================== CLI + uvicorn run =====================
def parse_args_to_config() -> TrackerConfig:
    ap = argparse.ArgumentParser(description="Antenna tracker + FastAPI (threaded)")
    ap.add_argument("--mode", choices=["sim","ground"], default=MODE_DEFAULT)
    ap.add_argument("--base-mode", choices=["static","dynamic"], default=BASE_MODE_DEFAULT)
    ap.add_argument("--static-window-sec", type=float, default=10.0)
    ap.add_argument("--print-period", type=float, default=PRINT_PERIOD_S)
    ap.add_argument("--update-period", type=float, default=UPDATE_PERIOD_S)
    ap.add_argument("--park-home-az", type=float, default=0.0)
    ap.add_argument("--park-home-el", type=float, default=90.0)
    ap.add_argument("--park-face-drone-start", action="store_true")
    ap.add_argument("--park-face-drone-exit", action="store_true")
    ap.add_argument("--park-duration", type=float, default=1.5)
    ap.add_argument("--park-rate-hz", type=float, default=60.0)
    args = ap.parse_args()
    return TrackerConfig(
        mode=args.mode,
        base_mode=args.base_mode,
        static_window_sec=args.static_window_sec,
        print_period=args.print_period,
        update_period=args.update_period,
        park_home_az=args.park_home_az,
        park_home_el=args.park_home_el,
        park_face_drone_start=args.park_face_drone_start,
        park_face_drone_exit=args.park_face_drone_exit,
        park_duration=args.park_duration,
        park_rate_hz=args.park_rate_hz,
    )

if __name__ == "__main__":
    # Parse CLI for tracker settings (applies when running as script)
    TRACKER_CONFIG = parse_args_to_config()

    # Start uvicorn (this blocks); tracker thread starts in startup hook
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
