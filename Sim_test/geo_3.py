from pymavlink import mavutil
import time
import azi_elev_4
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt

# === Constants ===
GCS_LAT = 13.0269709
GCS_LON = 77.5630563
GCS_ALT = 1

angle_log = []  # Global log list to store (timestamp, azimuth, elevation)

GEAR_RATIO = 2.0 # 2:1 gear ratio => 1 input degree = 2 output degree

STEP_SIZE = 1.0 # In degrees, Set the stepsize for the servos to move in

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
SERVO_AZI_PIN = 18  # GPIO18, Pin 12 , Azimuth servo
SERVO_ELE_PIN = 13  # GPIO13, Pin 33 , Elevation servo
GPIO.setup(SERVO_AZI_PIN, GPIO.OUT)
GPIO.setup(SERVO_ELE_PIN, GPIO.OUT)

servo_azi_pwm = GPIO.PWM(SERVO_AZI_PIN, 50)
servo_ele_pwm = GPIO.PWM(SERVO_ELE_PIN, 50)
servo_azi_pwm.start(0)
servo_ele_pwm.start(0)

# === Servo Tracking Angles ===
servo_azimuth_angle = 90.0  # Facing East
servo_elevation_angle = 45.0  # Mid elevation

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
            log_angles(servo_azimuth_angle, servo_elevation_angle)
            print(f"Moved to Azimuth: {servo_azimuth_angle:.2f}° | Elevation: {servo_elevation_angle:.2f}°")

        time.sleep(delay)

    # Final move to precise target if necessary
    final_az, final_el = azi_elev_4.adjust_angles_for_servo_limits(target_az, target_el)
    if abs(final_az - servo_azimuth_angle) > threshold or abs(final_el - servo_elevation_angle) > threshold:
        servo_azimuth_angle = final_az
        servo_elevation_angle = final_el
        set_angle(servo_azimuth_angle, servo_elevation_angle)
        print(f"Final Position -> Azimuth: {servo_azimuth_angle:.2f}° | Elevation: {servo_elevation_angle:.2f}°")

def plot_angle_log():
    if not angle_log:
        print("No data to plot.")
        return

    timestamps, azimuths, elevations = zip(*angle_log)

    # Convert timestamps to relative time (seconds from start)
    start_time = timestamps[0]
    time_series = [t - start_time for t in timestamps]

    plt.figure(figsize=(10, 5))
    plt.plot(time_series, azimuths, label='Azimuth (°)', color='blue')
    plt.plot(time_series, elevations, label='Elevation (°)', color='green')
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (degrees)")
    plt.title("Antenna Angle Tracking Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def log_angles(az, el):
    timestamp = time.time()
    angle_log.append((timestamp, az, el))

def GPS_stream():
    while True:
        msg = mav.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000

            print(f"Lat: {lat}, Lon: {lon}, Alt: {alt} m")

            azimuth = azi_elev_4.calculate_azimuth(GCS_LAT, GCS_LON, lat, lon)
            elevation = azi_elev_4.calculate_elevation(GCS_LAT, GCS_LON, lat, lon, GCS_ALT, alt)
            

            move_antenna_to_target(azimuth, elevation, step_size=3.0, delay=0.05)
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
