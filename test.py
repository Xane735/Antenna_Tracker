import math

def calculate_elevation(lat_gcs, lon_gcs, lat_uav, lon_uav, alt_gcs, alt_uav):
    """
    Calculate elevation angle from GCS to UAV.
    
    Parameters:
        lat_gcs, lon_gcs : float - Latitude and longitude of GCS in degrees
        lat_uav, lon_uav : float - Latitude and longitude of UAV in degrees
        alt_gcs, alt_uav : float - Altitudes of GCS and UAV in meters

    Returns:
        Elevation angle in degrees (0° = horizon, 90° = directly above)
    """
    # Convert degrees to radians
    lat1 = math.radians(lat_gcs)
    lon1 = math.radians(lon_gcs)
    lat2 = math.radians(lat_uav)
    lon2 = math.radians(lon_uav)

    # Earth radius
    R = 6371000  # meters

    # Haversine formula to calculate horizontal ground distance
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1

    a = math.sin(d_lat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    ground_distance = R * c  # in meters

    # Altitude difference
    delta_h = alt_uav - alt_gcs

    # Elevation angle (radians)
    if ground_distance == 0 and delta_h > 0:
        elevation_angle_deg = 90.0
    elif ground_distance == 0 and delta_h <= 0:
        elevation_angle_deg = 0.0
    else:
        elevation_angle_rad = math.atan2(delta_h, ground_distance)
        elevation_angle_deg = math.degrees(elevation_angle_rad)

    return round(elevation_angle_deg, 2)
