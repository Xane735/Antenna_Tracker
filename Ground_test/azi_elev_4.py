# Needs to be tested
# Enhanced azimuth/elevation calculations with servo constraint support
import math
from geopy.distance import geodesic
from geopy import Point

# Servo configuration constants
GEAR_RATIO = 2.0  # 2:1 gear ratio
SERVO_MAX_ANGLE = 180.0  # Physical servo limit for both Azimuth and Elevation servos
EFFECTIVE_AZIMUTH_RANGE = SERVO_MAX_ANGLE * GEAR_RATIO  # 360 degrees

# Practical elevation constraints for a typical antenna tracker
MIN_ELEVATION_ANGLE = 0.0
MAX_ELEVATION_ANGLE = 90.0

def calculate_azimuth_elevation(base_lat, base_lon, base_alt, target_lat, target_lon, target_alt):
    """
    Calculate azimuth and elevation angles from base to target using geopy for accurate geodetic calculations.
    
    Args:
        base_lat, base_lon: Base station coordinates (degrees, 7 decimal precision)
        base_alt: Base station altitude (meters)
        target_lat, target_lon: Target coordinates (degrees, 7 decimal precision)
        target_alt: Target altitude (meters)
    
    Returns:
        tuple: (azimuth, elevation) in degrees, rounded to 2 decimal places
    """
    try:
        # Create geopy Point objects
        base_point = Point(latitude=base_lat, longitude=base_lon)
        target_point = Point(latitude=target_lat, longitude=target_lon)
        
        # Calculate horizontal distance using geodesic (great circle) distance
        horizontal_distance = geodesic(base_point, target_point).meters
        
        # Calculate bearing (azimuth) from base to target
        azimuth = calculate_bearing(base_lat, base_lon, target_lat, target_lon)
        
        # Calculate elevation angle
        altitude_difference = target_alt - base_alt
        
        if horizontal_distance == 0:
            # Target is directly overhead or at same location
            elevation = 90.0 if altitude_difference > 0 else 0.0
        else:
            # Calculate elevation angle using arctangent
            elevation = math.degrees(math.atan2(altitude_difference, horizontal_distance))
        
        # Round to 2 decimal places for servo precision
        azimuth = round(azimuth, 2)
        elevation = round(elevation, 2)
        
        return azimuth, elevation
        
    except Exception as e:
        print(f"Error in calculate_azimuth_elevation: {e}")
        return None, None

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing (azimuth) between two points using spherical trigonometry.
    
    Args:
        lat1, lon1: Starting point coordinates (degrees)
        lat2, lon2: Ending point coordinates (degrees)
    
    Returns:
        float: Bearing in degrees (0-360, where 0 is North)
    """
    # Convert to radians
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    lon1_rad = math.radians(lon1)
    lon2_rad = math.radians(lon2)
    
    # Calculate difference in longitude
    dlon = lon2_rad - lon1_rad
    
    # Calculate bearing using spherical trigonometry
    y = math.sin(dlon) * math.cos(lat2_rad)
    x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
         math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon))
    
    # Calculate bearing in radians
    bearing_rad = math.atan2(y, x)
    
    # Convert to degrees and normalize to 0-360
    bearing_deg = math.degrees(bearing_rad)
    bearing_deg = (bearing_deg + 360) % 360
    
    return bearing_deg

def calculate_distance_and_bearing(base_lat, base_lon, target_lat, target_lon):
    """
    Calculate both distance and bearing between two points.
    
    Args:
        base_lat, base_lon: Base coordinates (degrees)
        target_lat, target_lon: Target coordinates (degrees)
    
    Returns:
        tuple: (distance_meters, bearing_degrees)
    """
    base_point = Point(latitude=base_lat, longitude=base_lon)
    target_point = Point(latitude=target_lat, longitude=target_lon)
    
    # Calculate distance
    distance = geodesic(base_point, target_point).meters
    
    # Calculate bearing
    bearing = calculate_bearing(base_lat, base_lon, target_lat, target_lon)
    
    return round(distance, 2), round(bearing, 2)

def check_servo_reachability(azimuth):
    """
    Check if the given azimuth is reachable with servo constraints.
    
    Args:
        azimuth: Target azimuth in degrees (0-360)
    
    Returns:
        tuple: (is_reachable, servo_angle, notes)
    """
    # Normalize azimuth
    azimuth = azimuth % 360
    
    # Calculate required servo angle
    servo_angle = azimuth / GEAR_RATIO
    
    # Check reachability
    is_reachable = 0 <= servo_angle <= SERVO_MAX_ANGLE
    
    notes = []
    if not is_reachable:
        notes.append(f"Azimuth servo angle {servo_angle:.1f}° exceeds limit of {SERVO_MAX_ANGLE}°.")
        # This case should ideally not happen with a 360 effective range
    
    # This check is technically redundant if GEAR_RATIO * SERVO_MAX_ANGLE >= 360
    if azimuth >= EFFECTIVE_AZIMUTH_RANGE:
        notes.append(f"Azimuth {azimuth}° exceeds effective range of {EFFECTIVE_AZIMUTH_RANGE}°.")
        is_reachable = False
        
    return is_reachable, round(servo_angle, 2), notes

def adjust_angles_for_servo_limits(azimuth, elevation):
    """
    Adjust angles for servo mechanical limits with constraint checking.
    
    Args:
        azimuth: Desired azimuth angle (0-360 degrees)
        elevation: Desired elevation angle (degrees)
    
    Returns:
        tuple: (adjusted_azimuth, adjusted_elevation, final_azimuth_servo_angle, final_elevation_servo_angle, is_reachable, notes)
    """
    notes = []
    
    # 1. Handle Azimuth
    # Normalize azimuth to the 0-360 range.
    adjusted_azimuth = azimuth % 360
    azimuth_servo_angle = adjusted_azimuth / GEAR_RATIO
    
    # With a 2:1 gear ratio, any azimuth is reachable. This check is a safeguard.
    if not (0 <= azimuth_servo_angle <= SERVO_MAX_ANGLE):
        notes.append(f"AZIMUTH ERROR: Required servo angle {azimuth_servo_angle:.2f}° is out of 0-180° range.")
        azimuth_reachable = False
    else:
        azimuth_reachable = True

    # 2. Handle Elevation
    # Clamp elevation to the practical mechanical range [0, 90] degrees.
    adjusted_elevation = max(MIN_ELEVATION_ANGLE, min(elevation, MAX_ELEVATION_ANGLE))
    
    if adjusted_elevation != elevation:
        notes.append(f"ELEVATION CLAMPED: Original {elevation}° was outside the {MIN_ELEVATION_ANGLE}°-{MAX_ELEVATION_ANGLE}° range.")

    # A 180-degree servo can easily cover the 0-90 degree elevation range.
    # We assume a direct 1:1 mapping for simplicity.
    # The servo angle is the same as the adjusted elevation.
    elevation_servo_angle = adjusted_elevation
    
    if not (MIN_ELEVATION_ANGLE <= elevation_servo_angle <= SERVO_MAX_ANGLE):
         notes.append(f"ELEVATION ERROR: Required servo angle {elevation_servo_angle:.2f}° is out of 0-180° range.")
         elevation_reachable = False
    else:
         elevation_reachable = True

    # 3. Final Assessment
    # The target is reachable only if both azimuth and elevation are valid.
    # The primary real-world constraint will be elevation (target not below horizon).
    is_reachable = azimuth_reachable and elevation_reachable and (elevation >= MIN_ELEVATION_ANGLE)

    if not is_reachable and not any("CLAMPED" in s for s in notes):
        notes.append("Target is below the horizon and cannot be tracked.")

    return (round(adjusted_azimuth, 2), 
            round(adjusted_elevation, 2), 
            round(azimuth_servo_angle, 2), 
            round(elevation_servo_angle, 2),
            is_reachable, 
            notes)

# --- Main Execution Block ---
if __name__ == "__main__":
    # Define Base Station (e.g., a location in San Francisco)
    BASE_LAT = 37.7749295
    BASE_LON = -122.4194155
    BASE_ALT = 16  # meters

    print(f"Antenna Base Station:")
    print(f"  Coords: (Lat: {BASE_LAT}, Lon: {BASE_LON})")
    print(f"  Altitude: {BASE_ALT} m\n")
    print("-" * 40)

    # --- Test Cases ---
    test_cases = {
        "1. Standard Reachable Target": {
            "lat": 37.802352, "lon": -122.405834, "alt": 5000
        },
        "2. Target Below Horizon": {
            "lat": 37.757694, "lon": -122.472618, "alt": -50  # Negative altitude ensures it's below horizon
        },
        "3. Target Directly Overhead": {
            "lat": BASE_LAT, "lon": BASE_LON, "alt": 10000
        },
        "4. High Azimuth Angle (West)": {
            "lat": 37.7749295, "lon": -122.5194155, "alt": 2000
        }
    }

    for name, target in test_cases.items():
        print(f"\nProcessing Test Case: {name}")
        print(f"  Target Coords: (Lat: {target['lat']}, Lon: {target['lon']}, Alt: {target['alt']}m)")

        # 1. Calculate initial Azimuth and Elevation
        az, el = calculate_azimuth_elevation(BASE_LAT, BASE_LON, BASE_ALT, target['lat'], target['lon'], target['alt'])
        
        if az is None:
            print("  Could not calculate initial angles.")
            continue
            
        print(f"  => Initial Calculated Angles: Azimuth={az}°, Elevation={el}°")

        # 2. Adjust for servo limits and get final servo commands
        (adj_az, adj_el, az_servo, el_servo, reachable, notes) = adjust_angles_for_servo_limits(az, el)

        print(f"  => Final Adjusted Angles:   Azimuth={adj_az}°, Elevation={adj_el}°")
        print(f"  => Required Servo Angles:   Azimuth Servo={az_servo}°, Elevation Servo={el_servo}°")
        print(f"  => Is Reachable? {reachable}")

        if notes:
            print("  - Notes:")
            for note in notes:
                print(f"    - {note}")
        print("-" * 40)