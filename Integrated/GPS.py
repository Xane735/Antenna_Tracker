from dataclasses import dataclass
from typing import Optional, Tuple
import time

from collections import deque

import threading

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
