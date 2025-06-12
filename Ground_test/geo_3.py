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
def move_antenna_stepwise(target_az, target_el, step_size=1.0, delay=0.05):
    global servo_azimuth_angle, servo_elevation_angle

    target_az = target_az % 360
    target_el = max(0, min(180, target_el))

    while True:
        delta_az = (target_az - servo_azimuth_angle + 540) % 360 - 180
        delta_el = target_el - servo_elevation_angle

        if abs(delta_az) <= step_size and abs(delta_el) <= step_size:
            break

        step_az = step_size if delta_az > 0 else -step_size
        step_el = step_size if delta_el > 0 else -step_size

        # Clamp to not overshoot
        if abs(delta_az) < abs(step_az):
            step_az = delta_az
        if abs(delta_el) < abs(step_el):
            step_el = delta_el

        servo_azimuth_angle += step_az
        servo_elevation_angle += step_el

        set_angle(servo_azimuth_angle, servo_elevation_angle)
        print(f"Moved to Azimuth: {servo_azimuth_angle:.2f}° | Elevation: {servo_elevation_angle:.2f}°")

        time.sleep(delay)

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
