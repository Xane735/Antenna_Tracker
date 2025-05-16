from pymavlink import mavutil
import math
import time
import azi_elev_2
import RPi.GPIO as GPIO
import subprocess

# Start MAVProxy in the background
'''print("Initializing Mavproxy")
subprocess.Popen([
    'mavproxy.py',
    '--master=udp:0.0.0.0:14550',     # Or '--master=udp:0.0.0.0:14550' depending on your source; For pixhawk: '--master=/dev/ttyAMA0'
    '--out=udp:192.168.1.193:14551',      # Or 'udp:127.0.0.1:14551'
    '--baudrate', '57600'        # Adjust baudrate based on your telemetry device
])

# Give MAVProxy a moment to start
time.sleep(5)
print("Mavproxy Running...")

# To know if Mavproxy is running: ps aux | grep mavproxy'''

# Latitude and Longitude of the GCS/Antenna Tracker
LAT_GCS = 13.026971
LON_GCS = 77.563056

# GPIO setup
GPIO.setmode(GPIO.BCM)

# Servo pins
SERVO_AZI_PIN = 18 # GPIO18, Pin 12 , Azimuth servo
SERVO_ELE_PIN = 13 # GPIO13, Pin 33 , Elevation servo

# Setup pins as output
GPIO.setup(SERVO_AZI_PIN, GPIO.OUT)
GPIO.setup(SERVO_ELE_PIN, GPIO.OUT)

# Create PWM instances at 50Hz
servo_azi = GPIO.PWM(SERVO_AZI_PIN, 50)
servo_ele = GPIO.PWM(SERVO_ELE_PIN, 50)

# Start PWM with neutral duty cycle (7.5%)
servo_azi.start(0)
servo_ele.start(0)

# Connect to MAVLink as Client
# Use 'tcp:localhost:5762' in the connection string to connect to SITL
# For Udp, 'udp:0.0.0.0:14551'

mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
print("Waiting for heartbeat...")
mav.wait_heartbeat()
print("Connected to system (system ID: %d, component ID: %d)" % (mav.target_system, mav.target_component))

# Set message interval
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    500000, 0, 0, 0, 0, 0
)

# Command functions
def loiter(mav):
    mav.mav.command_long_send(mav.target_system, mav.target_component, 176, 0, 1, 5, 0, 0, 0, 0, 0)
    msg = mav.recv_match(type='COMMAND_ACK', blocking=False)
    print(msg, "Set to Loiter Mode")

def auto(mav):
    mav.mav.command_long_send(mav.target_system, mav.target_component, 176, 0, 1, 3, 0, 0, 0, 0, 0)
    msg = mav.recv_match(type='COMMAND_ACK', blocking=False)
    print(msg, "Set to Auto Mode")

def arm(mav):
    mav.mav.command_long_send(mav.target_system, mav.target_component, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = mav.recv_match(type='COMMAND_ACK', blocking=False)
    print(msg, "Drone Armed")

def start_mission(mav):
    mav.mav.command_long_send(mav.target_system, mav.target_component, 300, 0, 0, 0, 0, 0, 0, 0, 0)
    test_msg = mav.recv_match(type='MISSION_CURRENT', blocking=False)
    print(test_msg, "Auto mission started!")

# Servo class
class Servo:
    @staticmethod
    def set_angle(servo_azi, ang_1, servo_ele, ang_2):
        duty_1 = 2.5 + (ang_1 / 18)
        duty_2 = 2.5 + (ang_2 / 18)
        servo_azi.ChangeDutyCycle(duty_1)
        servo_ele.ChangeDutyCycle(duty_2)
        time.sleep(0.5)
        servo_azi.ChangeDutyCycle(0)
        servo_ele.ChangeDutyCycle(0)

# GPS data handling
def GPS_stream(mav):
    while True:
        msg = mav.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000
            print(f"Lat: {lat}, Lon: {lon}, Alt: {alt} m, Satellites: {msg.satellites_visible}")
            azi = azi_elev_2.calculate_azimuth(LAT_GCS, LON_GCS, lat, lon)
            ele = azi_elev_2.calculate_elevation(LAT_GCS, LON_GCS, lat, lon, alt)
            Servo.set_angle(servo_azi, azi, servo_ele, ele)

# Entry point
def main(mav):
    # Optional flight mode control
    #loiter(mav)
    #arm(mav)
    #auto(mav)
    #start_mission(mav)
    GPS_stream(mav)

main(mav)
