# telemetry_payload.py
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any
import threading, time

@dataclass
class ServoState:
    phys_az: Optional[float] = None
    phys_el: Optional[float] = None
    servo_az: Optional[float] = None
    servo_el: Optional[float] = None
    us_az:   Optional[int]   = None
    us_el:   Optional[int]   = None

@dataclass
class AngleState:
    world_az: Optional[float] = None
    world_el: Optional[float] = None
    used_flip: Optional[bool] = None

@dataclass
class BaseMeta:
    mode: Optional[str] = None
    locked: Optional[bool] = None

class TelemetryState:
    def __init__(self):
        self._lock = threading.Lock()
        self._servo = ServoState()
        self._angles = AngleState()
        self._base   = BaseMeta()

    def set_servo(self, phys_az, phys_el, servo_az, servo_el, us_az, us_el):
        with self._lock:
            self._servo = ServoState(phys_az, phys_el, servo_az, servo_el, us_az, us_el)

    def set_angles(self, world_az, world_el, used_flip):
        with self._lock:
            self._angles = AngleState(world_az, world_el, bool(used_flip))

    def set_base_meta(self, mode: str, locked: bool):
        with self._lock:
            self._base = BaseMeta(mode, bool(locked))

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "servos": asdict(self._servo),
                "angles": asdict(self._angles),
                "meta":   asdict(self._base),
            }

def _gps_to_dict(s) -> Optional[Dict[str, Any]]:
    if s is None:
        return None
    return {
        "t": s.t,
        "lat": s.lat, "lon": s.lon, "alt": s.alt,
        "hdop": s.eph, "vdop": s.epv,
        "fix": s.fix_type, "sats": s.sats,
    }

def build_tracker_payload(state: TelemetryState, base_sample, drone_sample) -> Dict[str, Any]:
    snap = state.snapshot()
    return {
        "ts": time.time(),
        "base":  _gps_to_dict(base_sample),
        "drone": _gps_to_dict(drone_sample),
        "angles": snap["angles"],
        "servos": snap["servos"],
        "meta":   snap["meta"],
    }
