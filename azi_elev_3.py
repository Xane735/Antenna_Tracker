#Version 3 of azi_elev.py, currently working code
import math

def adjust_angles_for_servo_limits(azimuth, elevation):
    """
    Adjust azimuth and elevation so that azimuth stays within 0-180°,
    and elevation is inverted if azimuth exceeds 180°.
    """
    if azimuth > 180:
        azimuth = azimuth - 180
        elevation = 180 - elevation
    return wrap_angle_0_to_180(azimuth), wrap_angle_0_to_180(elevation)

def calculate_azimuth(lat1, lon1, lat2, lon2):
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)
    d_lon = lon2 - lon1

    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    azimuth = math.degrees(math.atan2(y, x))
    azimuth = (azimuth + 360) % 360  # Normalize to 0–360
    return azimuth  # Return full azimuth; flip logic happens separately

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

    return elev_angle  # Return full elevation; flip logic happens separately

def wrap_angle_0_to_180(angle):
    """Wrap any angle to 0-180 degrees range."""
    return angle % 180

# Test Bench for the functions:

def test_servo_angle_logic():
    # Base location (your station)
    base_lat = 12.9716    # Example: Bengaluru
    base_lon = 77.5946
    base_alt = 900        # in meters

    # Sample targets in various directions
    targets = [
        {"name": "North",  "lat": 13.0716, "lon": 77.5946, "alt": 1000},
        {"name": "South",  "lat": 12.8716, "lon": 77.5946, "alt": 1000},
        {"name": "East",   "lat": 12.9716, "lon": 77.6946, "alt": 1000},
        {"name": "West",   "lat": 12.9716, "lon": 77.4946, "alt": 1000},
        {"name": "NorthEast", "lat": 13.0716, "lon": 77.6946, "alt": 1000},
        {"name": "SouthWest", "lat": 12.8716, "lon": 77.4946, "alt": 1000},
        {"name": "Overhead",  "lat": 12.9716, "lon": 77.5946, "alt": 1100},
        {"name": "BehindHigh", "lat": 12.8716, "lon": 77.5946, "alt": 1200},  # Behind, high
        {"name": "BehindLow", "lat": 12.8716, "lon": 77.5946, "alt": 800},   # Behind, low
    ]

    for target in targets:
        az = calculate_azimuth(base_lat, base_lon, target["lat"], target["lon"])
        el = calculate_elevation(base_lat, base_lon, target["lat"], target["lon"], base_alt, target["alt"])
        az_adj, el_adj = adjust_angles_for_servo_limits(az, el)

        print(f"\nTarget: {target['name']}")
        print(f"Raw Azimuth: {az:.2f}°, Raw Elevation: {el:.2f}°")
        print(f"Adjusted for Servos => Azimuth: {az_adj:.2f}°, Elevation: {el_adj:.2f}°")

# Run the test
test_servo_angle_logic()
