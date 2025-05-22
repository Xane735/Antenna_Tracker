# Version 3 of geo.py, currently working code
from pymavlink import mavutil
import time
import azi_elev_3
import RPi.GPIO as GPIO
from simple_pid import PID

# === Constants ===
GCS_LAT = 13.0269709
GCS_LON = 77.5630563
GCS_ALT = 1

# === PID Controllers ===
AZI_PID = PID(1.5, 0.01, 0.2)
AZI_PID.output_limits = (-10, 10)

ELE_PID = PID(1.0, 0.01, 0.2)
ELE_PID.output_limits = (-5, 5)

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
SERVO_AZI_PIN = 18 # GPIO18, Pin 12 , Azimuth servo
SERVO_ELE_PIN = 13 # GPIO13, Pin 33 , Elevation servo
GPIO.setup(SERVO_AZI_PIN, GPIO.OUT)
GPIO.setup(SERVO_ELE_PIN, GPIO.OUT)

servo_azi_pwm = GPIO.PWM(SERVO_AZI_PIN, 50)
servo_ele_pwm = GPIO.PWM(SERVO_ELE_PIN, 50)
servo_azi_pwm.start(0)
servo_ele_pwm.start(0)

# === Servo Tracking Angles ===
servo_azimuth_angle = 90.0  # Facing East
servo_elevation_angle = 45.0  # Mid elevation

class Servo:
    @staticmethod
    def set_angle(azi_angle, ele_angle):
        azi_duty = 2.5 + (azi_angle / 18)
        ele_duty = 2.5 + (ele_angle / 18)
        servo_azi_pwm.ChangeDutyCycle(azi_duty)
        servo_ele_pwm.ChangeDutyCycle(ele_duty)
        time.sleep(0.5)
        servo_azi_pwm.ChangeDutyCycle(0)
        servo_ele_pwm.ChangeDutyCycle(0)

# === MAVLink Connection ===
mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
print("Waiting for heartbeat...")
mav.wait_heartbeat()
print(f"Connected to system (system ID: {mav.target_system}, component ID: {mav.target_component})")

# === Message Interval Setting ===
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    500000, 0, 0, 0, 0, 0
)

def move_antenna(az_delta, el_delta, threshold=1.0):
    global servo_azimuth_angle, servo_elevation_angle

    # Calculate tentative new angles
    new_az = (servo_azimuth_angle + az_delta) % 360
    new_el = servo_elevation_angle + el_delta

    # Clamp elevation between 0 and 180 (servo range)
    new_el = max(0, min(180, new_el))

    # Adjust for servo limits
    adj_az, adj_el = azi_elev_3.adjust_angles_for_servo_limits(new_az, new_el)

    # Only update servo if change exceeds threshold (to reduce jitter)
    if abs(adj_az - servo_azimuth_angle) > threshold or abs(adj_el - servo_elevation_angle) > threshold:
        servo_azimuth_angle = adj_az
        servo_elevation_angle = adj_el
        Servo.set_angle(servo_azimuth_angle, servo_elevation_angle)
        print(f"Moved to Azimuth: {servo_azimuth_angle:.2f}° | Elevation: {servo_elevation_angle:.2f}°")
    else:
        # Change too small, skip update to reduce jitter
        pass

def GPS_stream():
    while True:
        msg = mav.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000

            print(f"Lat: {lat}, Lon: {lon}, Alt: {alt} m")

            azimuth = azi_elev_3.calculate_azimuth(GCS_LAT, GCS_LON, lat, lon)
            elevation = azi_elev_3.calculate_elevation(GCS_LAT, GCS_LON, lat, lon, GCS_ALT, alt)

            AZI_PID.setpoint = azimuth
            ELE_PID.setpoint = elevation

            azimuth_adjust = AZI_PID(servo_azimuth_angle)
            elevation_adjust = ELE_PID(servo_elevation_angle)

            move_antenna(azimuth_adjust, elevation_adjust)
            time.sleep(0.1)

def main():
    try:
        GPS_stream()
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        servo_azi_pwm.stop()
        servo_ele_pwm.stop()
        GPIO.cleanup()

main()
