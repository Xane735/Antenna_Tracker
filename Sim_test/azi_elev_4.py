# Does not use Spherical Trignometry, Assume ENU (Simple Local Tangenet Plane - Flat Earth Model)
import math

# Earth approximation constants
LAT_TO_METERS = 111_320  # meters per degree latitude (roughly constant)

def get_lon_to_meters(lat_deg):
    return 111_320 * math.cos(math.radians(lat_deg))  # varies with latitude

def calculate_azimuth(lat1, lon1, lat2, lon2):
    lon_scale = get_lon_to_meters(lat1)
    dx = (lon2 - lon1) * lon_scale
    dy = (lat2 - lat1) * LAT_TO_METERS
    angle = math.degrees(math.atan2(dx, dy))  # atan2(x, y)
    return (angle + 360) % 360  # Normalize to [0, 360)

def calculate_elevation(lat1, lon1, lat2, lon2, alt1, alt2):
    lon_scale = get_lon_to_meters(lat1)
    dx = (lon2 - lon1) * lon_scale
    dy = (lat2 - lat1) * LAT_TO_METERS
    horizontal_dist = math.hypot(dx, dy)
    d_alt = alt2 - alt1

    if horizontal_dist == 0:
        return 90.0 if d_alt > 0 else 0.0
    return math.degrees(math.atan2(d_alt, horizontal_dist))

def adjust_angles_for_servo_limits(azimuth, elevation):
    azimuth = azimuth % 360
    elevation = max(0, min(180, elevation))
    return azimuth, elevation

def test_servo_angle_logic():
    # Base location (your station)
    base_lat = 12.9716   
    base_lon = 77.5946
    base_alt = 10        # in meters

    # Sample targets in various directions
    targets = [
    {"name": "North",       "lat": 13.0716, "lon": 77.5946, "alt": 1000},
    {"name": "South",       "lat": 12.8716, "lon": 77.5946, "alt": 1000},
    {"name": "East",        "lat": 12.9716, "lon": 77.6946, "alt": 1000},
    {"name": "West",        "lat": 12.9716, "lon": 77.4946, "alt": 1000},
    {"name": "NorthEast",   "lat": 13.0716, "lon": 77.6946, "alt": 1000},
    {"name": "SouthWest",   "lat": 12.8716, "lon": 77.4946, "alt": 1000},
    {"name": "Overhead",    "lat": 12.9716, "lon": 77.5946, "alt": 1100},
    {"name": "BehindHigh",  "lat": 12.8716, "lon": 77.5946, "alt": 1200},
    {"name": "BehindLow",   "lat": 12.8716, "lon": 77.5946, "alt": 800},
    ]

    for target in targets:
        az = calculate_azimuth(base_lat, base_lon, target["lat"], target["lon"])
        el = calculate_elevation(base_lat, base_lon, target["lat"], target["lon"], base_alt, target["alt"])
        az_adj, el_adj = az, el

        print(f"\nTarget: {target['name']}")
        print(f"Raw Azimuth: {az:.2f}°, Raw Elevation: {el:.2f}°")
        print(f"Adjusted for Servos => Azimuth: {az_adj:.2f}°, Elevation: {el_adj:.2f}°")

# Run the test
#test_servo_angle_logic()