# Version 6: Enhanced Dual GPS integration with 180° servo constraint handling and data logging
import threading
from pymavlink import mavutil
import time
import azi_elev_4
import RPi.GPIO as GPIO
import math
import csv
import os
from datetime import datetime

# === Constants ===
GCS_LAT = 13.0269709  # Fallback coordinates if base GPS fails
GCS_LON = 77.5630563
GCS_ALT = 1  # meters

GEAR_RATIO = 2.0  # 2:1 gear ratio => 1 input degree = 2 output degree
STEP_SIZE = 1.0   # In degrees, Set the stepsize for the servos to move in

# Servo physical constraints
SERVO_MAX_ANGLE = 180.0  # Physical servo limit (180 degrees)
EFFECTIVE_RANGE = SERVO_MAX_ANGLE * GEAR_RATIO  # 360 degrees with gear ratio

# Set Initial Position after PWM Start
servo_azimuth_angle = 90.0
servo_elevation_angle = 45.0

# Thread-safe GPS data with locks
drone_gps = {"lat": None, "lon": None, "alt": None}
base_gps = {"lat": GCS_LAT, "lon": GCS_LON, "alt": GCS_ALT, "is_dynamic": False}

# Thread locks for GPS data
drone_gps_lock = threading.Lock()
base_gps_lock = threading.Lock()

# Debug flags
DEBUG = True
VERBOSE_GPS = True
SHOW_TRACKING_INFO = True

# Data logging setup
LOG_ENABLED = True
LOG_DIRECTORY = "antenna_tracker_logs"
LOG_FILENAME = None
log_file_handle = None
log_writer = None

def setup_logging():
    """Setup CSV logging for tracking data"""
    # FIX: All global declarations must be at the top of the function
    global LOG_FILENAME, log_file_handle, log_writer, LOG_ENABLED
    
    if not LOG_ENABLED:
        return
    
    try:
        # Create log directory if it doesn't exist
        if not os.path.exists(LOG_DIRECTORY):
            os.makedirs(LOG_DIRECTORY)
        
        # Create filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        LOG_FILENAME = os.path.join(LOG_DIRECTORY, f"tracker_log_{timestamp}.csv")
        
        # Open file and create CSV writer
        log_file_handle = open(LOG_FILENAME, 'w', newline='')
        log_writer = csv.writer(log_file_handle)
        
        # Write header
        header = [
            'timestamp', 'drone_lat', 'drone_lon', 'drone_alt',
            'base_lat', 'base_lon', 'base_alt', 'base_is_dynamic',
            'calculated_azimuth', 'calculated_elevation',
            'servo_azimuth', 'servo_elevation',
            'horizontal_distance', 'slant_range', 'altitude_difference'
        ]
        log_writer.writerow(header)
        log_file_handle.flush()
        
        debug_print(f"Logging enabled: {LOG_FILENAME}")
        
    except Exception as e:
        debug_print(f"Failed to setup logging: {e}", "ERROR")
        LOG_ENABLED = False

def log_tracking_data(drone_data, base_data, tracking_info, servo_az, servo_el):
    """Log tracking data to CSV file"""
    if not LOG_ENABLED or not log_writer:
        return
    
    try:
        timestamp = datetime.now().isoformat()
        
        row = [
            timestamp,
            drone_data.get('lat', ''),
            drone_data.get('lon', ''),
            drone_data.get('alt', ''),
            base_data.get('lat', ''),
            base_data.get('lon', ''),
            base_data.get('alt', ''),
            base_data.get('is_dynamic', False),
            tracking_info.get('azimuth', '') if tracking_info else '',
            tracking_info.get('elevation', '') if tracking_info else '',
            servo_az,
            servo_el,
            tracking_info.get('horizontal_distance', '') if tracking_info else '',
            tracking_info.get('slant_range', '') if tracking_info else '',
            tracking_info.get('altitude_difference', '') if tracking_info else ''
        ]
        
        log_writer.writerow(row)
        log_file_handle.flush()
        
    except Exception as e:
        debug_print(f"Error logging data: {e}", "ERROR")

def debug_print(message, level="INFO"):
    """Print debug messages with timestamp"""
    if DEBUG:
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")

def handle_servo_constraints(target_azimuth):
    """
    Handle 180° servo constraint with 2:1 gear ratio
    Maps 360° azimuth range to 180° servo range
    
    Args:
        target_azimuth: Desired azimuth (0-360°)
    
    Returns:
        tuple: (servo_angle, is_valid)
        - servo_angle: Angle to send to servo (0-180°)
        - is_valid: Whether the target is reachable
    """
    # Normalize azimuth to 0-360 range
    target_azimuth = target_azimuth % 360
    
    # With 2:1 gear ratio, servo angle = azimuth / 2
    servo_angle = target_azimuth / GEAR_RATIO
    
    # Check if within servo limits
    if 0 <= servo_angle <= SERVO_MAX_ANGLE:
        return round(servo_angle, 2), True
    else:
        # This shouldn't happen with 2:1 ratio covering full 360°, but add safety
        servo_angle = max(0, min(SERVO_MAX_ANGLE, servo_angle))
        debug_print(f"Servo angle clamped to {servo_angle}° for azimuth {target_azimuth}°", "WARN")
        return round(servo_angle, 2), False

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
        debug_print(f"Failed to connect to base station: {e}", "WARNING")
        debug_print("Will use static base coordinates", "INFO")
        return mav_drone, None
    
    debug_print("Waiting for heartbeats...")
    
    # Wait for heartbeats with timeout
    try:
        debug_print("Waiting for drone heartbeat...")
        mav_drone.wait_heartbeat(timeout=10)
        debug_print("Drone heartbeat received")
    except Exception as e:
        debug_print(f"Drone heartbeat timeout: {e}", "ERROR")
        return None, None
    
    if mav_base:
        try:
            debug_print("Waiting for base station heartbeat...")
            mav_base.wait_heartbeat(timeout=10)
            debug_print("Base station heartbeat received")
        except Exception as e:
            debug_print(f"Base station heartbeat timeout: {e}", "WARNING")
            debug_print("Will use static base coordinates", "INFO")
            return mav_drone, None
    
    debug_print("Connected to systems successfully")
    return mav_drone, mav_base

# Initialize connections
mav_drone, mav_base = connect_mavlink()

def set_angle(azi_angle, ele_angle):
    """Set servo angles with bounds checking and constraint handling"""
    try:
        # Handle azimuth servo constraints
        servo_azi_angle, is_valid = handle_servo_constraints(azi_angle)
        
        if not is_valid:
            debug_print(f"Azimuth {azi_angle}° is outside reachable range", "WARN")
        
        # Handle elevation constraints (standard 0-180° servo)
        ele_angle = max(0, min(180, ele_angle))
        servo_ele_angle = ele_angle / GEAR_RATIO  # Apply gear ratio to elevation too
        servo_ele_angle = max(0, min(90, servo_ele_angle))  # Elevation servo limit
        
        # Calculate duty cycles for servo control
        # Standard servo: 1ms (2.5% duty) = 0°, 2ms (12.5% duty) = 180°
        azi_duty = 2.5 + (servo_azi_angle * 10.0 / 180.0)
        ele_duty = 2.5 + (servo_ele_angle * 10.0 / 90.0)  # Adjusted for elevation range
        
        # Bounds check duty cycles
        azi_duty = max(2.5, min(12.5, azi_duty))
        ele_duty = max(2.5, min(12.5, ele_duty))
        
        servo_azi_pwm.ChangeDutyCycle(azi_duty)
        servo_ele_pwm.ChangeDutyCycle(ele_duty)
        
        debug_print(f"Servo control - Target Az: {azi_angle}°, Servo Az: {servo_azi_angle}° (duty: {azi_duty:.2f}%), "
                  f"Target El: {ele_angle}°, Servo El: {servo_ele_angle}° (duty: {ele_duty:.2f}%)")
        
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
            msg = mav_drone.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if msg:
                with drone_gps_lock:
                    drone_gps["lat"] = round(msg.lat / 1e7, 7)
                    drone_gps["lon"] = round(msg.lon / 1e7, 7)
                    drone_gps["alt"] = round(msg.alt / 1000, 2)
                
                if VERBOSE_GPS:
                    debug_print(f"[Drone GPS] Lat: {drone_gps['lat']:.7f}, "
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
        debug_print("No base station connection - using static coordinates", "INFO")
        return
        
    consecutive_errors = 0
    max_errors = 10
    
    while consecutive_errors < max_errors:
        try:
            msg = mav_base.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if msg:
                with base_gps_lock:
                    base_gps["lat"] = round(msg.lat / 1e7, 7)
                    base_gps["lon"] = round(msg.lon / 1e7, 7)
                    base_gps["alt"] = round(msg.alt / 1000, 2)
                    base_gps["is_dynamic"] = True
                
                if VERBOSE_GPS:
                    debug_print(f"[Base GPS] Lat: {base_gps['lat']:.7f}, "
                               f"Lon: {base_gps['lon']:.7f}, Alt: {base_gps['alt']:.1f} m (Dynamic)")
                consecutive_errors = 0
            else:
                debug_print("No base GPS message received (timeout)", "WARN")
                consecutive_errors += 1
                
        except Exception as e:
            debug_print(f"Base GPS update error: {e}", "ERROR")
            consecutive_errors += 1
            time.sleep(1)
    
    debug_print("Base GPS thread stopped - reverting to static coordinates", "WARN")
    with base_gps_lock:
        base_gps["is_dynamic"] = False

def move_antenna_to_target(target_az, target_el, step_size=1.0, delay=0.05, threshold=1.0):
    """Move antenna to target position with smooth motion and constraint handling"""
    global servo_azimuth_angle, servo_elevation_angle
    
    # Check if target azimuth is reachable
    servo_target_az, is_reachable = handle_servo_constraints(target_az)
    
    if not is_reachable:
        debug_print(f"Target azimuth {target_az}° is not reachable with current servo constraints", "WARN")
        return
    
    # Convert back to effective azimuth for tracking
    effective_target_az = servo_target_az * GEAR_RATIO
    target_el = max(0, min(180, target_el))
    
    debug_print(f"Moving antenna to target: Az={effective_target_az}°, El={target_el}°")
    
    step_count = 0
    max_steps = 500
    
    while step_count < max_steps:
        # Compute deltas
        delta_az = (effective_target_az - servo_azimuth_angle + 540) % 360 - 180
        delta_el = target_el - servo_elevation_angle
        
        if step_count % 10 == 0:
            debug_print(f"Step {step_count}: Current Az={servo_azimuth_angle}°, El={servo_elevation_angle}°, "
                       f"Delta Az={delta_az:.2f}°, Delta El={delta_el:.2f}°", "DEBUG")
        
        # Break if within threshold
        if abs(delta_az) <= threshold and abs(delta_el) <= threshold:
            debug_print("Target reached within threshold")
            break
        
        # Compute step increments
        step_az = step_size * (1 if delta_az > 0 else -1) if abs(delta_az) > step_size else delta_az
        step_el = step_size * (1 if delta_el > 0 else -1) if abs(delta_el) > step_size else delta_el
        
        # Calculate new angles
        new_az = (servo_azimuth_angle + step_az) % 360
        new_el = max(0, min(180, servo_elevation_angle + step_el))
        
        # Check servo constraints for new azimuth
        _, az_valid = handle_servo_constraints(new_az)
        if not az_valid:
            debug_print("New azimuth position not reachable, stopping movement", "WARN")
            break
        
        # Only move if change is significant
        if abs(new_az - servo_azimuth_angle) > 0.1 or abs(new_el - servo_elevation_angle) > 0.1:
            servo_azimuth_angle = new_az
            servo_elevation_angle = new_el
            set_angle(servo_azimuth_angle, servo_elevation_angle)
            
            if step_count % 10 == 0:
                debug_print(f"Moved to Azimuth: {servo_azimuth_angle}° | Elevation: {servo_elevation_angle}°")
        
        time.sleep(delay)
        step_count += 1
    
    if step_count >= max_steps:
        debug_print("Maximum steps reached, stopping movement", "WARN")

def calculate_tracking_angles():
    """Calculate azimuth and elevation for tracking using geopy"""
    with drone_gps_lock, base_gps_lock:
        if not all([drone_gps["lat"], drone_gps["lon"], drone_gps["alt"]]):
            debug_print("Drone GPS data not available", "WARN")
            return None, None, None
        
        try:
            tracking_info = azi_elev_4.get_tracking_info(
                base_gps["lat"], base_gps["lon"], base_gps["alt"],
                drone_gps["lat"], drone_gps["lon"], drone_gps["alt"]
            )
            
            if tracking_info:
                if SHOW_TRACKING_INFO:
                    base_source = "Dynamic GPS" if base_gps["is_dynamic"] else "Static coordinates"
                    debug_print(f"Tracking Info ({base_source}) - Distance: {tracking_info['horizontal_distance']}m, "
                               f"Slant: {tracking_info['slant_range']}m, "
                               f"Alt Diff: {tracking_info['altitude_difference']}m")
                
                return (tracking_info['adjusted_azimuth'], 
                       tracking_info['adjusted_elevation'], 
                       tracking_info)
            else:
                return None, None, None
                
        except Exception as e:
            debug_print(f"Error calculating tracking angles: {e}", "ERROR")
            return None, None, None

def tracking_loop():
    """Main tracking loop with enhanced geopy calculations and logging"""
    debug_print("Starting enhanced tracking loop with geopy and logging")
    
    while True:
        try:
            azimuth, elevation, tracking_info = calculate_tracking_angles()
            
            if azimuth is not None and elevation is not None:
                debug_print(f"Calculated tracking angles: Az={azimuth}°, El={elevation}°")
                
                # Check if azimuth is reachable
                _, is_reachable = handle_servo_constraints(azimuth)
                
                if is_reachable:
                    # Only move if there's a significant change
                    current_delta_az = abs(azimuth - servo_azimuth_angle)
                    current_delta_el = abs(elevation - servo_elevation_angle)
                    
                    if current_delta_az > 0.5 or current_delta_el > 0.5:
                        move_antenna_to_target(azimuth, elevation, STEP_SIZE)
                    else:
                        debug_print("Target within movement threshold, no servo adjustment needed")
                else:
                    debug_print(f"Target azimuth {azimuth}° not reachable with servo constraints", "WARN")
                
                # Log data regardless of movement
                with drone_gps_lock, base_gps_lock:
                    log_tracking_data(
                        dict(drone_gps), 
                        dict(base_gps), 
                        tracking_info,
                        servo_azimuth_angle,
                        servo_elevation_angle
                    )
                    
            else:
                debug_print("Cannot calculate tracking angles, waiting...", "WARN")
                # Log empty data
                with drone_gps_lock, base_gps_lock:
                    log_tracking_data(
                        dict(drone_gps), 
                        dict(base_gps), 
                        None,
                        servo_azimuth_angle,
                        servo_elevation_angle
                    )
            
            time.sleep(1)  # Update rate
            
        except KeyboardInterrupt:
            debug_print("Tracking interrupted by user")
            break
        except Exception as e:
            debug_print(f"Error in tracking loop: {e}", "ERROR")
            time.sleep(5)

def cleanup():
    """Clean up resources"""
    debug_print("Cleaning up resources...")
    try:
        servo_azi_pwm.stop()
        servo_ele_pwm.stop()
        GPIO.cleanup()
        debug_print("GPIO cleanup completed")
    except Exception as e:
        debug_print(f"Error during GPIO cleanup: {e}", "ERROR")
    
    # Close log file
    if log_file_handle:
        try:
            log_file_handle.close()
            debug_print(f"Log file closed: {LOG_FILENAME}")
        except Exception as e:
            debug_print(f"Error closing log file: {e}", "ERROR")

def main():
    debug_print("Starting Enhanced Antenna Tracker System with servo constraints and logging")
    
    # Setup logging
    setup_logging()
    
    # Print base station info
    base_source = "Dynamic GPS" if mav_base else "Static coordinates"
    debug_print(f"Base station source: {base_source}")
    debug_print(f"Initial base coordinates: Lat={base_gps['lat']}, Lon={base_gps['lon']}, Alt={base_gps['alt']}m")
    
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
        else:
            debug_print("Using static base coordinates (no base GPS available)")
        
        # Wait for initial GPS data
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
