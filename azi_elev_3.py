#Version 3 of azi_elev.py, currently working code

import math
def calculate_azimuth(lat1, lon1, lat2, lon2):
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)
    d_lon = lon2 - lon1

    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    azimuth = math.degrees(math.atan2(y, x))
    azimuth = (azimuth + 360) % 360

    return wrap_angle_0_to_180(azimuth)

def calculate_elevation(lat1, lon1, lat2, lon2, alt1, alt2):
    R = 6371000  # Earth radius in meters
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)
    d_lon = lon2 - lon1

    ground_dist = math.acos(
        math.sin(lat1) * math.sin(lat2) +
        math.cos(lat1) * math.cos(lat2) * math.cos(d_lon)
    ) * R

    d_alt = alt2 - alt1
    if ground_dist == 0:
        elev_angle = 90.0 if d_alt > 0 else 0.0
    else:
        elev_angle = math.degrees(math.atan2(d_alt, ground_dist))

    return wrap_angle_0_to_180(elev_angle)

def wrap_angle_0_to_180(angle):
    angle = angle % 360
    if angle > 180:
        angle = 360 - angle
    return round(angle, 2)
