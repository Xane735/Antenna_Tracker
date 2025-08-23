import CONFIG as C
from dataclasses import dataclass
from collections import deque
from datetime import datetime
from typing import Optional, Tuple, Callable
from pymavlink import mavutil

@dataclass
class GpsSample:
    """GPS sample data structure for logging"""
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
        return GPsSample(
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
        return GPsSample(
            t=t,
            lat=msg.lat/1e7,
            lon=msg.lon/1e7,
            alt=msg.alt/1000.0
        )
    return None

class GpsBuffer:
    def __init__(self, name: str, maxlen: int = 120):
        self.name = name
        self._buf: deque[GPsSample] = deque(maxlen=maxlen)
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
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, C.MAV_MSG_INTERVAL_US_GPS),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, C.MAV_MSG_INTERVAL_US_GLOBAL),
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
