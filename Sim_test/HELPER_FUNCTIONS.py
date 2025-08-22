import CONFIG as C
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import math
from typing import Optional, Tuple, Callable

def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def world_el_to_us(world_el_deg: float) -> int:
    # clamp to 0..90 so you never drive below horizon or beyond straight up
    w = _clamp(world_el_deg, C.EL_MIN_WORLD_DEG, C.EL_MAX_WORLD_DEG)

    # mapping for your 2:1 gear and 1500 µs (servo 90°) = sky:
    # world_el: 0→horizon, +90→sky
    # servo_deg = (world_el + 90)/2
    servo_deg = (w + 90.0) / 2.0

    us = 1500.0 + (servo_deg - 90.0) * C.US_PER_SERVO_DEG
    # final safety clamp to mechanical µs range
    if us < C.EL_US_MIN: us = C.EL_US_MIN
    if us > C.EL_US_MAX: us = C.EL_US_MAX
    return int(us)

# ============= Helpers & Small Utilities =============

def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

def _calibratioapplyn(az: float, el: float) -> Tuple[float, float]:
    az = norm360(az + C.AZIMUTH_ZERO_OFFSET_DEG)
    el += C.ELEVATION_ZERO_OFFSET_DEG
    if C.AZIMUTH_INVERT:
        az = norm360(360.0 - az)
    if C.ELEVATION_INVERT:
        el = -el
    return az, el

def world_to_physical(az: float, el: float) -> Tuple[float, float]:
    return (max(C.AZ_PHYS_MIN, min(C.AZ_PHYS_MAX, az)),
            max(C.EL_PHYS_MIN, min(C.EL_PHYS_MAX, el)))

def physical_to_servo_deg(az_phys: float, el_phys: float) -> Tuple[float, float]:
    az_raw = az_phys / C.AZ_GEAR_RATIO
    el_raw = el_phys / C.EL_GEAR_RATIO
    az_servo = max(0.0, min(C.SERVO_RANGE_DEG, az_raw))
    el_servo = max(0.0, min(C.SERVO_RANGE_DEG, el_raw))
    return az_servo, el_servo

def servo_deg_to_us(deg: float) -> float:
    deg = max(0.0, min(C.SERVO_RANGE_DEG, deg))
    return C.PULSE_MIN_US + (deg / C.SERVO_RANGE_DEG) * (C.PULSE_MAX_US - C.PULSE_MIN_US)

# Quick deg→meters conversion (approx), good enough for stability heuristics
def latlon_to_meters(lat_deg: float, lon_deg: float, ref_lat_deg: float) -> Tuple[float,float]:
    lat_m = lat_deg * 111_320.0
    lon_m = lon_deg * 111_320.0 * math.cos(math.radians(ref_lat_deg))
    return lat_m, lon_m

