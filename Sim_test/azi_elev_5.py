# Current working code for calculating angles using geopy

import math
from geopy.distance import geodesic
from geopy import Point

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
        base_point = Point(latitude = base_lat, longitude = base_lon)
        target_point = Point(latitude = target_lat, longitude = target_lon)
        
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

def adjust_angles_for_servo_limits(azimuth, elevation):
    """
    Adjust angles for servo mechanical limits with 2 decimal precision.
    
    Args:
        azimuth: Desired azimuth angle (0-360 degrees)
        elevation: Desired elevation angle (degrees)
    
    Returns:
        tuple: (adjusted_azimuth, adjusted_elevation) rounded to 2 decimal places
    """
    # Normalize azimuth to 0-360 range
    azimuth = azimuth % 360
    
    # Apply elevation limits (typically 0-90 degrees for most antenna trackers)
    # Adjust these limits based on your specific servo/mechanical constraints
    min_elevation = 0.0   # Minimum elevation (horizon)
    max_elevation = 90.0  # Maximum elevation (zenith)
    
    elevation = max(min_elevation, min(max_elevation, elevation))
    
    # Handle negative elevations (target below horizon)
    if elevation < 0:
        # Option 1: Clamp to horizon
        elevation = 0.0
        
        # Option 2: Point to opposite azimuth (uncomment if needed)
        # azimuth = (azimuth + 180) % 360
        # elevation = abs(elevation)
    
    # Apply azimuth limits if your servo has restricted rotation
    # Uncomment and adjust if your azimuth servo has limits
    # min_azimuth = 0.0
    # max_azimuth = 360.0
    # azimuth = max(min_azimuth, min(max_azimuth, azimuth))
    
    return round(azimuth, 2), round(elevation, 2)

def get_tracking_info(base_lat, base_lon, base_alt, target_lat, target_lon, target_alt):
    """
    Get comprehensive tracking information including distance, bearing, and angles.
    
    Args:
        base_lat, base_lon, base_alt: Base station coordinates and altitude
        target_lat, target_lon, target_alt: Target coordinates and altitude
    
    Returns:
        dict: Comprehensive tracking information
    """
    try:
        # Calculate azimuth and elevation
        azimuth, elevation = calculate_azimuth_elevation(
            base_lat, base_lon, base_alt, target_lat, target_lon, target_alt
        )
        
        # Calculate horizontal distance
        distance, bearing = calculate_distance_and_bearing(base_lat, base_lon, target_lat, target_lon)
        
        # Calculate 3D distance (slant range)
        altitude_diff = target_alt - base_alt
        slant_range = math.sqrt(distance**2 + altitude_diff**2)
        
        # Adjust angles for servo limits
        adj_azimuth, adj_elevation = adjust_angles_for_servo_limits(azimuth, elevation)
        
        return {
            'azimuth': azimuth,
            'elevation': elevation,
            'adjusted_azimuth': adj_azimuth,
            'adjusted_elevation': adj_elevation,
            'horizontal_distance': round(distance, 2),
            'slant_range': round(slant_range, 2),
            'altitude_difference': round(altitude_diff, 2),
            'bearing': bearing
        }
        
    except Exception as e:
        print(f"Error in get_tracking_info: {e}")
        return None
