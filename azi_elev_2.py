import math

def calculate_azimuth(lat_gcs, lon_gcs, lat_uav, lon_uav):
    lat1 = math.radians(lat_gcs)
    lon1 = math.radians(lon_gcs)
    lat2 = math.radians(lat_uav)
    lon2 = math.radians(lon_uav)

    d_lon = lon2 - lon1

    x = math.sin(d_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)

    azimuth = math.atan2(x, y)
    azimuth_deg = (math.degrees(azimuth) + 360) % 360

    return round(azimuth_deg, 2)
def calculate_elevation(lat_gcs, lon_gcs, lat_uav, lon_uav, alt_uav):
    R = 6371000  # Radius of Earth in meters
    alt_gcs = 1  # Altitude of GCS in meters

    # Convert degrees to radians
    lat1 = math.radians(lat_gcs)
    lon1 = math.radians(lon_gcs)
    lat2 = math.radians(lat_uav)
    lon2 = math.radians(lon_uav)

    # Haversine formula for horizontal distance
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1
    a = math.sin(d_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    horizontal_distance = R * c  # In meters

    vertical_difference = alt_uav - alt_gcs

    if horizontal_distance == 0:
        return 90.0 if vertical_difference > 0 else 0.0

    elevation_rad = math.atan2(vertical_difference, horizontal_distance)
    elevation_deg = math.degrees(elevation_rad)

    return round(elevation_deg, 2)
