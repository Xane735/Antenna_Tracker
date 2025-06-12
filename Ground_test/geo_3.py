# Version 4: Dual GPS integration for Antenna Tracker
import threading
from pymavlink import mavutil
import time
import azi_elev_4
import RPi.GPIO as GPIO
import math

# === Constants ===
GCS_LAT = 13.0269709
GCS_LON = 77.5630563
GCS_ALT = 1  # meters

GEAR_RATIO = 2.0 # 2:1 gear ratio => 1 input degree = 2 output degree

STEP_SIZE = 1.0 # In degrees, Set the stepsize for the servos to move in

# Set Initial Position after PWM Start
servo_azimuth_angle = 90.0
servo_elevation_angle = 45.0

drone_gps = {"lat": None, "lon": None, "alt": None}
base_gps = {"lat": GCS_LAT, "lon": GCS_LON, "alt": GCS_ALT}

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
SERVO_AZI_PIN = 18
SERVO_ELE_PIN = 13
GPIO.setup(SERVO_AZI_PIN, GPIO.OUT)
GPIO.setup(SERVO_ELE_PIN, GPIO.OUT)

servo_azi_pwm = GPIO.PWM(SERVO_AZI_PIN, 50)
servo_ele_pwm = GPIO.PWM(SERVO_ELE_PIN, 50)
servo_azi_pwm.start(0)
servo_ele_pwm.start(0)

# === MAVLink Connections ===
mav_drone = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
mav_base = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

print("Waiting for heartbeats...")
mav_drone.wait_heartbeat()
mav_base.wait_heartbeat()
print("Connected to both systems")

def set_angle(azi_angle, ele_angle):
    # Convert to servo PWM-compatible angles (gear corrected)
    gear_corrected_azi = azi_angle / GEAR_RATIO
    gear_corrected_ele = ele_angle / GEAR_RATIO
    azi_duty = 2.5 + (gear_corrected_azi / 18)
    ele_duty = 2.5 + (gear_corrected_ele / 18)
    servo_azi_pwm.ChangeDutyCycle(azi_duty)
    servo_ele_pwm.ChangeDutyCycle(ele_duty)
    time.sleep(0.5)
    servo_azi_pwm.ChangeDutyCycle(0)
    servo_ele_pwm.ChangeDutyCycle(0)

# Setting Servo to an initial position
set_angle(servo_azimuth_angle / GEAR_RATIO, servo_elevation_angle / GEAR_RATIO)
time.sleep(1)

def update_drone_gps():
    while True:
        msg = mav_drone.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            drone_gps["lat"] = msg.lat / 1e7
            drone_gps["lon"] = msg.lon / 1e7
            drone_gps["alt"] = msg.alt / 1000
            print(f"[Drone] Lat: {drone_gps['lat']}, Lon: {drone_gps['lon']}, Alt: {drone_gps['alt']} m")

def update_base_gps():
    while True:
        msg = mav_base.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            base_gps["lat"] = msg.lat / 1e7
            base_gps["lon"] = msg.lon / 1e7
            base_gps["alt"] = msg.alt / 1000
            print(f"[Base] Lat: {base_gps['lat']}, Lon: {base_gps['lon']}, Alt: {base_gps['alt']} m")

# === Fixed move_antenna_stepwise ===
def move_antenna_to_target(target_az, target_el, step_size=1.0, delay=0.05, threshold=1.0):
    global servo_azimuth_angle, servo_elevation_angle

    # Normalize target values
    target_az = target_az % 360
    target_el = max(0, min(180, target_el))

    while True:
        # Compute deltas
        delta_az = (target_az - servo_azimuth_angle + 540) % 360 - 180  # shortest angular path
        delta_el = target_el - servo_elevation_angle

        # Break if within threshold
        if abs(delta_az) <= step_size and abs(delta_el) <= step_size:
            break

        # Compute step increments
        step_az = step_size * (1 if delta_az > 0 else -1) if abs(delta_az) > step_size else delta_az
        step_el = step_size * (1 if delta_el > 0 else -1) if abs(delta_el) > step_size else delta_el

        # Tentative new angles
        new_az = (servo_azimuth_angle + step_az) % 360
        new_el = max(0, min(180, servo_elevation_angle + step_el))

        # Adjust for servo limits
        adj_az, adj_el = azi_elev_4.adjust_angles_for_servo_limits(new_az, new_el)

        # Only move if change is significant
        if abs(adj_az - servo_azimuth_angle) > threshold or abs(adj_el - servo_elevation_angle) > threshold:
            servo_azimuth_angle = adj_az
            servo_elevation_angle = adj_el
            set_angle(servo_azimuth_angle, servo_elevation_angle)
            print(f"Moved to Azimuth: {servo_azimuth_angle:.2f}° | Elevation: {servo_elevation_angle:.2f}°")

        time.sleep(delay)

    # Final move to precise target if necessary
    final_az, final_el = azi_elev_4.adjust_angles_for_servo_limits(target_az, target_el)
    if abs(final_az - servo_azimuth_angle) > threshold or abs(final_el - servo_elevation_angle) > threshold:
        servo_azimuth_angle = final_az
        servo_elevation_angle = final_el
        set_angle(servo_azimuth_angle, servo_elevation_angle)
        print(f"Final Position -> Azimuth: {servo_azimuth_angle:.2f}° | Elevation: {servo_elevation_angle:.2f}°")


def main():
    
    try:
        threading.Thread(target=update_drone_gps, daemon=True).start()
        threading.Thread(target=update_base_gps, daemon=True).start()

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        servo_azi_pwm.stop()
        servo_ele_pwm.stop()
        GPIO.cleanup()

main()
