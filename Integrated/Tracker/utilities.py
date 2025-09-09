from typing import Tuple
import config


def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

# --- Shortest-path wrap helpers ---
def wrap360(x: float) -> float:
    return (x + 360.0) % 360.0

def shortest_delta_deg(target: float, current: float) -> float:
    # returns signed delta in (−180, +180]
    return ((target - current + 540.0) % 360.0) - 180.0

def apply_calibration(az: float, el: float) -> Tuple[float, float]:
    az = norm360(az + config.AZIMUTH_ZERO_OFFSET_DEG)
    el += config.ELEVATION_ZERO_OFFSET_DEG
    if config.AZIMUTH_INVERT:
        az = norm360(360.0 - az)
    if config.ELEVATION_INVERT:
        el = -el
    return az, el

def world_to_physical(az: float, el: float) -> Tuple[float, float]:
    return (max(config.AZ_PHYS_MIN, min(config.AZ_PHYS_MAX, az)),
            max(config.EL_PHYS_MIN, min(config.EL_PHYS_MAX, el)))

def clamp_physical(az, el):
    # clamp to your physical pan-tilt range; typical: el 0..180, az unbounded
    el = max(0.0, min(180.0, el))
    return az, el

def shortest_servo_delta(a, b):
    """Return signed shortest delta between two servo angles in [0,180].
       For 0-180 servos, this is just b - a (no wrap), but keep the helper
       in case you later map to a 0-360 continuous az servo."""
    return b - a
