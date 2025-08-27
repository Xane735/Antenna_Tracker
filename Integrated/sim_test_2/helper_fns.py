# -*- coding: utf-8 -*-
"""
Created on Sat Aug 23 09:25:27 2025

@author: Dell
"""

from typing import Tuple
import constants
import math

# ============= Helpers & Small Utilities =============

def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

def apply_calibration(az: float, el: float) -> Tuple[float, float]:
    az = norm360(az + constants.AZIMUTH_ZERO_OFFSET_DEG)
    el += constants.ELEVATION_ZERO_OFFSET_DEG
    if constants.AZIMUTH_INVERT:
        az = norm360(360.0 - az)
    if constants.ELEVATION_INVERT:
        el = -el
    return az, el

def world_to_physical(az: float, el: float) -> Tuple[float, float]:
    return (max(constants.AZ_PHYS_MIN, min(constants.AZ_PHYS_MAX, az)),
            max(constants.EL_PHYS_MIN, min(constants.EL_PHYS_MAX, el)))

def physical_to_servo_deg(az_phys: float, el_phys: float) -> Tuple[float, float]:
    az_raw = az_phys / constants.AZ_GEAR_RATIO
    el_raw = el_phys / constants.EL_GEAR_RATIO
    az_servo = max(0.0, min(constants.SERVO_RANGE_DEG, az_raw))
    el_servo = max(0.0, min(constants.SERVO_RANGE_DEG, el_raw))
    return az_servo, el_servo

def servo_deg_to_us(deg: float) -> float:
    deg = max(0.0, min(constants.SERVO_RANGE_DEG, deg))
    return constants.PULSE_MIN_US + (deg / constants.SERVO_RANGE_DEG) * (constants.PULSE_MAX_US - constants.PULSE_MIN_US)

# Quick deg→meters conversion (approx), good enough for stability heuristics
def latlon_to_meters(lat_deg: float, lon_deg: float, ref_lat_deg: float) -> Tuple[float,float]:
    lat_m = lat_deg * 111_320.0
    lon_m = lon_deg * 111_320.0 * math.cos(math.radians(ref_lat_deg))
    return lat_m, lon_m
