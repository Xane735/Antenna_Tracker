import math
import time
from pymavlink import mavutil
from simple_pid import PID

# Constants
GCS_LAT = 13.0272307   # Replace with your actual GCS lat
GCS_LON = 77.5630984   # Replace with your actual GCS lon
GCS_ALT = 920       # Meters

# Connect to MAVLink
mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')

# PID Controllers (tune these values)
az_pid = PID(Kp=1.0, Ki=0.01, Kd=0.2, setpoint=0)
az_pid.output_limits = (-10, 10)  # Limits in degrees per update

el_pid = PID(Kp=1.0, Ki=0.01, Kd=0.2, setpoint=0)
el_pid.output_limits = (-5, 5)

# Helper functions
def calculate_azimuth(lat1, lon1, lat2, lon2):
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)
    d_lon = lon2 - lon1

    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    azimuth = math.degrees(math.atan2(y, x))
    azimuth = (azimuth + 360) % 360

    return wrap_angle_0_to_180(azimuth)

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

    return wrap_angle_0_to_180(elev_angle)
def get_uav_data():
    msg_gps = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    msg_att = mav.recv_match(type='ATTITUDE', blocking=True)

    lat = msg_gps.lat / 1e7
    lon = msg_gps.lon / 1e7
    alt = msg_gps.alt / 1000.0
    yaw = math.degrees(msg_att.yaw)
    yaw = (yaw + 360) % 360
    return lat, lon, alt, yaw

# Simulated actuator control (replace with GPIO PWM in real system)
servo_azimuth_angle = 0
servo_elevation_angle = 0

def move_antenna(az_delta, el_delta):
    global servo_azimuth_angle, servo_elevation_angle
    servo_azimuth_angle = (servo_azimuth_angle + az_delta) % 360
    servo_elevation_angle = max(0, min(90, servo_elevation_angle + el_delta))

    print(f"Servo Azimuth Angle: {round(servo_azimuth_angle, 2)}° | Elevation Angle: {round(servo_elevation_angle, 2)}°")

def wrap_angle_0_to_180(angle):
    angle = angle % 360
    if angle > 180:
        angle = 360 - angle
    return round(angle, 2)

# Main loop
def main():
    global servo_azimuth_angle, servo_elevation_angle
    print("PID-based Antenna Tracking Starting...")
    
    while True:
        try:
            lat, lon, alt, heading = get_uav_data()

            azimuth = calculate_azimuth(GCS_LAT, GCS_LON, lat, lon)
            elevation = calculate_elevation(GCS_LAT, GCS_LON, lat, lon, GCS_ALT, alt)

            # Set new setpoints for PID
            az_pid.setpoint = azimuth
            el_pid.setpoint = elevation

            # Get PID output (how much to rotate)
            azimuth_adjust = az_pid(servo_azimuth_angle)
            elevation_adjust = el_pid(servo_elevation_angle)

            move_antenna(azimuth_adjust, elevation_adjust)
            time.sleep(0.1)

        except Exception as e:
            print("Error:", e)
            time.sleep(1)

if __name__ == "__main__":
    main()
