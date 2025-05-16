import math

def calculate_azimuth(GCS_LAT, GCS_LON, lat2, lon2):
    GCS_LAT, GCS_LON = math.radians(GCS_LAT), math.radians(GCS_LON)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)
    d_lon = lon2 - GCS_LON

    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(GCS_LAT)*math.sin(lat2) - math.sin(GCS_LAT)*math.cos(lat2)*math.cos(d_lon)
    azimuth = math.degrees(math.atan2(y, x))
    return (azimuth + 360) % 360

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
        return 90.0 if d_alt > 0 else 0.0
    elev_angle = math.degrees(math.atan2(d_alt, ground_dist))
    return round(elev_angle, 2)

