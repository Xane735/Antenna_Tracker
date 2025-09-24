from pymavlink import mavutil
from typing import Optional, Callable
import threading
import config
import time
from dataclasses import dataclass

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

_latest_base: Optional[GpsSample] = None
_latest_drone: Optional[GpsSample] = None
_base_lock = threading.Lock()
_drone_lock = threading.Lock()


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
    
# ===================== MAVLink I/O =====================

def connect_mav(endpoint: str, baud: Optional[int], hb_required: bool) -> mavutil.mavfile:
    mav = mavutil.mavlink_connection(endpoint, baud=baud) if baud else mavutil.mavlink_connection(endpoint)
    try:
        mav.wait_heartbeat(timeout=5)
        print(f"[MAV] Connected {endpoint} sys={mav.target_system} comp={mav.target_component}")
    except Exception as e:
        if hb_required:
            raise
        print(f"[MAV] No heartbeat on {endpoint} - continuing: {e}")
    for msg_id, interval in (
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,        config.MAV_MSG_INTERVAL_US_GPS),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, config.MAV_MSG_INTERVAL_US_GLOBAL),
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