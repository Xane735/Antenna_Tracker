# -*- coding: utf-8 -*-
"""
Created on Sat Aug 23 10:04:05 2025

@author: Dell
"""

from dataclasses import dataclass
from typing import Optional
import math
import GPS as gps
import time
import helper_fns

# ============= Base filters & stabilizer =============

def rolling_median(values):
    s = sorted(values)
    n = len(s)
    if n == 0: return None
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5*(s[mid-1]+s[mid])

@dataclass
class BaseState:
    mode: str            # "dynamic" | "static"
    locked: bool         # True if static mode has locked
    lat: float
    lon: float
    alt: float
    sd_lat_m: float = 0.0
    sd_lon_m: float = 0.0
    fix_type: Optional[int] = None
    sats: Optional[int] = None

def compute_sd(values):
    if not values: return 0.0
    mean = sum(values) / len(values)
    var = sum((x-mean)**2 for x in values) / max(1, (len(values)-1))
    return math.sqrt(var)

def stabilize_base(base_buf: gps.GpsBuffer,window_sec: float,sd_thresh_m: float,min_samples: int = 15) -> Optional[BaseState]:
    """
    Wait until base GPS stabilizes over window_sec with both lat/lon stddev < sd_thresh_m,
    then return the mean position as locked BaseState.
    """
    t_end = time.time() + max(5.0, window_sec)  # never wait <5s
    while time.time() < t_end:
        last, snap = base_buf.snapshot()
        if len(snap) >= min_samples:
            # Filter by time window
            now = time.time()
            win = [s for s in snap if now - s.t <= window_sec]
            if len(win) >= min_samples:
                ref_lat = win[-1].lat
                lat_m = []
                lon_m = []
                for s in win:
                    lm, Lm = helper_fns.latlon_to_meters(s.lat - ref_lat, s.lon - win[-1].lon, ref_lat)
                    lat_m.append(lm)
                    lon_m.append(Lm)
                sd_lat = compute_sd(lat_m)
                sd_lon = compute_sd(lon_m)
                if sd_lat < sd_thresh_m and sd_lon < sd_thresh_m:
                    # lock to means (use arithmetic mean of degrees)
                    mean_lat = sum(s.lat for s in win)/len(win)
                    mean_lon = sum(s.lon for s in win)/len(win)
                    mean_alt = sum(s.alt for s in win)/len(win)
                    return BaseState(
                        mode="static", locked=True,
                        lat=mean_lat, lon=mean_lon, alt=mean_alt,
                        sd_lat_m=sd_lat, sd_lon_m=sd_lon,
                        fix_type=win[-1].fix_type, sats=win[-1].sats
                    )
        time.sleep(0.2)
    return None

def dynamic_base_filtered(base_buf: gps.GpsBuffer,
                          window_sec: float,
                          min_samples: int = 5) -> Optional[BaseState]:
    last, snap = base_buf.snapshot()
    if not snap: return None
    now = time.time()
    win = [s for s in snap if now - s.t <= window_sec]
    if not win:
        return None
    ref_lat = win[-1].lat
    lat_vals = [s.lat for s in win]
    lon_vals = [s.lon for s in win]
    alt_vals = [s.alt for s in win]
    lat = rolling_median(lat_vals)
    lon = rolling_median(lon_vals)
    alt = rolling_median(alt_vals)
    # compute SD in meters for logging insight
    lat_m = [helper_fns.latlon_to_meters(s - ref_lat, 0.0, ref_lat)[0] for s in lat_vals]
    lon_m = [helper_fns.latlon_to_meters(0.0, s - lon_vals[-1], ref_lat)[1] for s in lon_vals]
    
    return BaseState(
        mode="dynamic", locked=False,
        lat=lat, lon=lon, alt=alt,
        sd_lat_m=compute_sd(lat_m), sd_lon_m=compute_sd(lon_m),
        fix_type=win[-1].fix_type, sats=win[-1].sats
    )
