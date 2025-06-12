# Version 4: Dual GPS integration for Antenna Tracker - FIXED
import threading
from pymavlink import mavutil
import time
import azi_elev_4
import RPi.GPIO as GPIO
import math

# === Configuration ===
DEBUG = True              # Enable/disable debug messages
VERBOSE_GPS = False       # Enable verbose GPS logging (can be noisy)
VERBOSE_MOVEMENT = False  # Enable verbose servo movement logging
CONNECTION_TIMEOUT = 10   # MAVLink connection timeout in seconds
GPS_TIMEOUT = 5          # GPS message timeout in seconds
TRACKING_UPDATE_RATE = 1  # Tracking loop update rate in seconds

# === Constants ===
GCS_LAT = 13.0269709
GCS_LON = 77.5630563
GCS_ALT = 1  # meters

GEAR_RATIO = 2.0  # 2:1 gear ratio => 1 input degree = 2 output degree
STEP_SIZE = 1.0   # In degrees, Set the stepsize for the servos to move in

# Set Initial Position after PWM Start
servo_azimuth_angle = 90.0
servo_elevation_angle = 45.0

# Thread-safe GPS data with locks
drone_gps = {"lat": None, "lon": None, "alt": None}
base_gps = {"lat": GCS_LAT, "lon": GCS_LON, "alt": GCS_ALT}

# Thread locks for GPS data
drone_gps_lock = threading.Lock()
base_gps_lock = threading.Lock()

def debug_print(message, level="INFO"):
    """Print debug messages with timestamp"""
    if DEBUG:
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")

def gps_print(message):
    """Print GPS-specific messages"""
    if VERBOSE_GPS:
        debug_print(message, "GPS")

def movement_print(message):
    """Print movement-specific messages"""
    if VERBOSE_MOVEMENT:
        debug_print(message, "MOVE")

# === GPIO Setup ===
try:
    GPIO.setmode(GPIO.BCM)
    SERVO_AZI_PIN = 18
    SERVO_ELE_PIN = 13
    GPIO.setup(SERVO_AZI_PIN, GPIO.OUT)
    GPIO.setup(SERVO_ELE_PIN, GPIO.OUT)
    
    servo_azi_pwm = GPIO.PWM(SERVO_AZI_PIN, 50)
    servo_ele_pwm = GPIO.PWM(SERVO_ELE_PIN, 50)
    servo_azi_pwm.start(0)
    servo_ele_pwm.start(0)
    debug_print("GPIO setup completed successfully")
except Exception as e:
    debug_print(f"GPIO setup failed: {e}", "ERROR")
    raise

# === MAVLink Connections ===
def connect_mavlink():
    """Establish MAVLink connections with error handling"""
    debug_print("Attempting to connect to MAVLink devices...")
    
    try:
        mav_drone = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
        debug_print("Drone connection established on /dev/ttyACM0")
    except Exception as e:
        debug_print(f"Failed to connect to drone: {e}", "ERROR")
        return None, None
    
    try:
        mav_base = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
        debug_print("Base station connection established on /dev/ttyUSB0")
    except Exception as e:
        debug_print(f"Failed to connect to base station: {e}", "ERROR")
        return mav_drone, None
    
    debug_print("Waiting for heartbeats...")
    
    # Wait for heartbeats with timeout
    try:
        debug_print("Waiting for drone heartbeat...")
        mav_drone.wait_heartbeat(timeout=CONNECTION_TIMEOUT)
        debug_print("Drone heartbeat received")
    except Exception as e:
        debug_print(f"Drone heartbeat timeout: {e}", "ERROR")
        return None, None
    
    try:
        debug_print("Waiting for base station heartbeat...")
        mav_base.wait_heartbeat(timeout=CONNECTION_TIMEOUT)
        debug_print("Base station heartbeat received")
    except Exception as e:
        debug_print(f"Base station heartbeat timeout: {e}", "ERROR")
        return mav_drone, None
    
    debug_print("Connected to both systems successfully")
    return mav_drone, mav_base

# Initialize connections
mav_drone, mav_base = connect_mavlink()

def set_angle(azi_angle, ele_angle):
    """Set servo angles with bounds checking and error handling"""
    try:
        # Bounds checking
        azi_angle = max(0, min(360, azi_angle))
        ele_angle = max(0, min(180, ele_angle))
        
        # Convert to servo PWM-compatible angles (gear corrected)
        gear_corrected_azi = azi_angle / GEAR_RATIO
        gear_corrected_ele = ele_angle / GEAR_RATIO
        
        # Calculate duty cycles (assuming 0-180 degree servos)
        # Formula: duty_cycle = 2.5 + (angle * 10.0 / 180.0)
        azi_duty = 2.5 + (gear_corrected_azi * 10.0 / 180.0)
        ele_duty = 2.5 + (gear_corrected_ele * 10.0 / 180.0)
        
        # Bounds check duty cycles
        azi_duty = max(2.5, min(12.5, azi_duty))
        ele_duty = max(2.5, min(12.5, ele_duty))
        
        servo_azi_pwm.ChangeDutyCycle(azi_duty)
        servo_ele_pwm.ChangeDutyCycle(ele_duty)
        
        debug_print(f"Servo angles set - Azi: {azi_angle:.2f}° (duty: {azi_duty:.2f}%), "
                   f"Ele: {ele_angle:.2f}° (duty: {ele_duty:.2f}%)")
        
        time.sleep(0.5)
        servo_azi_pwm.ChangeDutyCycle(0)
        servo_ele_pwm.ChangeDutyCycle(0)
        
    except Exception as e:
        debug_print(f"Error setting servo angles: {e}", "ERROR")

# Setting Servo to initial position
debug_print(f"Setting initial servo position: Azi={servo_azimuth_angle}°, Ele={servo_elevation_angle}°")
set_angle(servo_azimuth_angle, servo_elevation_angle)
time.sleep(1)

def update_drone_gps():
    """Update drone GPS data with error handling"""
    debug_print("Starting drone GPS update thread")
    if not mav_drone:
        debug_print("No drone connection available", "ERROR")
        return
        
    consecutive_errors = 0
    max_errors = 10
    
    while consecutive_errors < max_errors:
        try:
            msg = mav_drone.recv_match(type='GPS_RAW_INT', blocking=True, timeout=GPS_TIMEOUT)
            if msg:
                with drone_gps_lock:
                    drone_gps["lat"] = msg.lat / 1e7
                    drone_gps["lon"] = msg.lon / 1e7
                    drone_gps["alt"] = msg.alt / 1000
                
                gps_print(f"[Drone GPS] Lat: {drone_gps['lat']:.7f}, "
                         f"Lon: {drone_gps['lon']:.7f}, Alt: {drone_gps['alt']:.1f} m")
                consecutive_errors = 0
            else:
                debug_print("No drone GPS message received (timeout)", "WARN")
                consecutive_errors += 1
                
        except Exception as e:
            debug_print(f"Drone GPS update error: {e}", "ERROR")
            consecutive_errors += 1
            time.sleep(1)
    
    debug_print("Drone GPS thread stopped due to too many errors", "ERROR")

def update_base_gps():
    """Update base station GPS data with error handling"""
    debug_print("Starting base GPS update thread")
    if not mav_base:
        debug_print("No base station connection available", "ERROR")
        return
        
    consecutive_errors = 0
    max_errors = 10
    
    while consecutive_errors < max_errors:
        try:
            msg = mav_base.recv_match(type='GPS_RAW_INT', blocking=True, timeout=GPS_TIMEOUT)
            if msg:
                with base_gps_lock:
                    base_gps["lat"] = msg.lat / 1e7
                    base_gps["lon"] = msg.lon / 1e7
                    base_gps["alt"] = msg.alt / 1000
                
                gps_print(f"[Base GPS] Lat: {base_gps['lat']:.7f}, "
                         f"Lon: {base_gps['lon']:.7f}, Alt: {base_gps['alt']:.1f} m")
                consecutive_errors = 0
            else:
                debug_print("No base GPS message received (timeout)", "WARN")
                consecutive_errors += 1
                
        except Exception as e:
            debug_print(f"Base GPS update error: {e}", "ERROR")
            consecutive_errors += 1
            time.sleep(1)
    
    debug_print("Base GPS thread stopped due to too many errors", "ERROR")

def adjust_angles_for_servo_limits(azimuth, elevation):
    """
    Adjust angles for servo mechanical limits
    This is a placeholder function - customize based on your servo setup
    """
    # Normalize azimuth to 0-360 range
    azimuth = azimuth % 360
    
    # Limit elevation to reasonable servo range (0-180 degrees)
    elevation = max(0, min(180, elevation))
    
    # Handle negative elevation (target below horizon)
    if elevation < 0:
        # Point antenna horizontally if target is below horizon
        elevation = 0
        debug_print(f"Target below horizon, setting elevation to 0°", "WARN")
    
    # Handle very high elevation (near zenith)
    if elevation > 90:
        debug_print(f"High elevation angle: {elevation:.2f}°", "INFO")
    
    return azimuth, elevation

def move_antenna_to_target(target_az, target_el, step_size=1.0, delay=0.05, threshold=1.0):
    """Move antenna to target position with smooth motion"""
    global servo_azimuth_angle, servo_elevation_angle
    
    debug_print(f"Moving antenna to target: Az={target_az:.2f}°, El={target_el:.2f}°")
    
    # Normalize target values
    target_az = target_az % 360
    target_el = max(0, min(180, target_el))
    
    step_count = 0
    max_steps = 500  # Prevent infinite loops
    
    while step_count < max_steps:
        # Compute deltas
        delta_az = (target_az - servo_azimuth_angle + 540) % 360 - 180  # shortest angular path
        delta_el = target_el - servo_elevation_angle
        
        if step_count % 10 == 0:  # Reduce debug frequency
            debug_print(f"Step {step_count}: Current Az={servo_azimuth_angle:.2f}°, El={servo_elevation_angle:.2f}°, "
                       f"Delta Az={delta_az:.2f}°, Delta El={delta_el:.2f}°")
        
        # Break if within threshold
        if abs(delta_az) <= threshold and abs(delta_el) <= threshold:
            debug_print("Target reached within threshold")
            break
        
        # Compute step increments
        step_az = step_size * (1 if delta_az > 0 else -1) if abs(delta_az) > step_size else delta_az
        step_el = step_size * (1 if delta_el > 0 else -1) if abs(delta_el) > step_size else delta_el
        
        # Tentative new angles
        new_az = (servo_azimuth_angle + step_az) % 360
        new_el = max(0, min(180, servo_elevation_angle + step_el))
        
        # Adjust for servo limits
        adj_az, adj_el = adjust_angles_for_servo_limits(new_az, new_el)
        
        # Only move if change is significant
        if abs(adj_az - servo_azimuth_angle) > 0.1 or abs(adj_el - servo_elevation_angle) > 0.1:
            servo_azimuth_angle = adj_az
            servo_elevation_angle = adj_el
            set_angle(servo_azimuth_angle, servo_elevation_angle)
            
            if step_count % 5 == 0:  # Reduce print frequency
                movement_print(f"Moved to Azimuth: {servo_azimuth_angle:.2f}° | Elevation: {servo_elevation_angle:.2f}°")
        
        time.sleep(delay)
        step_count += 1
    
    if step_count >= max_steps:
        debug_print("Maximum steps reached, stopping movement", "WARN")
    
    # Final move to precise target if necessary
    final_az, final_el = adjust_angles_for_servo_limits(target_az, target_el)
    
    if abs(final_az - servo_azimuth_angle) > threshold or abs(final_el - servo_elevation_angle) > threshold:
        servo_azimuth_angle = final_az
        servo_elevation_angle = final_el
        set_angle(servo_azimuth_angle, servo_elevation_angle)
        debug_print(f"Final Position -> Azimuth: {servo_azimuth_angle:.2f}° | Elevation: {servo_elevation_angle:.2f}°")

def calculate_tracking_angles():
    """Calculate azimuth and elevation for tracking"""
    with drone_gps_lock, base_gps_lock:
        # Check if drone GPS data is valid
        if not all([drone_gps["lat"], drone_gps["lon"], drone_gps["alt"]]):
            debug_print("Drone GPS data not available", "WARN")
            return None, None
        
        # Check if GPS data is reasonable (not zero or clearly invalid)
        if (abs(drone_gps["lat"]) < 0.001 or abs(drone_gps["lon"]) < 0.001 or
            abs(drone_gps["lat"]) > 90 or abs(drone_gps["lon"]) > 180):
            debug_print(f"Invalid drone GPS data: Lat={drone_gps['lat']}, Lon={drone_gps['lon']}", "WARN")
            return None, None
        
        try:
            # Calculate azimuth and elevation using separate functions from azi_elev_4 module
            azimuth = azi_elev_4.calculate_azimuth(
                base_gps["lat"], base_gps["lon"],
                drone_gps["lat"], drone_gps["lon"]
            )
            
            elevation = azi_elev_4.calculate_elevation(
                base_gps["lat"], base_gps["lon"],
                drone_gps["lat"], drone_gps["lon"],
                base_gps["alt"], drone_gps["alt"]
            )
            
            debug_print(f"Raw calculations: Az={azimuth:.2f}°, El={elevation:.2f}°")
            debug_print(f"Base: Lat={base_gps['lat']:.6f}, Lon={base_gps['lon']:.6f}, Alt={base_gps['alt']:.1f}m")
            debug_print(f"Drone: Lat={drone_gps['lat']:.6f}, Lon={drone_gps['lon']:.6f}, Alt={drone_gps['alt']:.1f}m")
            
            return azimuth, elevation
            
        except Exception as e:
            debug_print(f"Error calculating tracking angles: {e}", "ERROR")
            return None, None

def tracking_loop():
    """Main tracking loop"""
    debug_print("Starting tracking loop")
    
    while True:
        try:
            azimuth, elevation = calculate_tracking_angles()
            
            if azimuth is not None and elevation is not None:
                debug_print(f"Calculated tracking angles: Az={azimuth:.2f}°, El={elevation:.2f}°")
                move_antenna_to_target(azimuth, elevation, STEP_SIZE)
            else:
                debug_print("Cannot calculate tracking angles, waiting...", "WARN")
            
            time.sleep(TRACKING_UPDATE_RATE)  # Update rate
            
        except KeyboardInterrupt:
            debug_print("Tracking interrupted by user")
            break
        except Exception as e:
            debug_print(f"Error in tracking loop: {e}", "ERROR")
            time.sleep(5)  # Wait before retrying

def cleanup():
    """Clean up resources"""
    debug_print("Cleaning up resources...")
    try:
        servo_azi_pwm.stop()
        servo_ele_pwm.stop()
        GPIO.cleanup()
        debug_print("GPIO cleanup completed")
    except Exception as e:
        debug_print(f"Error during cleanup: {e}", "ERROR")

def main():
    debug_print("Starting Antenna Tracker System")
    
    try:
        # Start GPS update threads
        if mav_drone:
            drone_thread = threading.Thread(target=update_drone_gps, daemon=True)
            drone_thread.start()
            debug_print("Drone GPS thread started")
        
        if mav_base:
            base_thread = threading.Thread(target=update_base_gps, daemon=True)
            base_thread.start()
            debug_print("Base GPS thread started")
        
        # Wait a bit for GPS data to populate
        debug_print("Waiting for initial GPS data...")
        time.sleep(5)
        
        # Start tracking
        tracking_loop()
        
    except KeyboardInterrupt:
        debug_print("System interrupted by user")
    except Exception as e:
        debug_print(f"System error: {e}", "ERROR")
    finally:
        cleanup()

if __name__ == "__main__":
    main()