# Antenna Tracker System with Dual GPS - Refined Version
import threading
from pymavlink import mavutil
import time
import azi_elev_5 as tracker
import RPi.GPIO as GPIO
import math

# === Initial Setup ===
GEAR_RATIO = 2.0
STEP_SIZE = 1.0  # degrees

# Track logical antenna direction (0–360)
servo_logical_azimuth_angle = 90.0
servo_elevation_angle = 45.0

# Use external GPS inputs (not hardcoded)
drone_gps = {"lat": None, "lon": None, "alt": None}
base_gps = {"lat": None, "lon": None, "alt": None}
drone_gps_lock = threading.Lock()
base_gps_lock = threading.Lock()

DEBUG = True
VERBOSE_GPS = True

def debug(msg, level="INFO"):
    if DEBUG:
        print(f"[{time.strftime('%H:%M:%S')}] [{level}] {msg}")

# === GPIO Setup ===
try:
    GPIO.setmode(GPIO.BCM)
    SERVO_AZI_PIN = 18
    SERVO_ELE_PIN = 13
    GPIO.setup(SERVO_AZI_PIN, GPIO.OUT)
    GPIO.setup(SERVO_ELE_PIN, GPIO.OUT)
    pwm_azi = GPIO.PWM(SERVO_AZI_PIN, 50)
    pwm_ele = GPIO.PWM(SERVO_ELE_PIN, 50)
    pwm_azi.start(0)
    pwm_ele.start(0)
    debug("GPIO setup complete")
except Exception as e:
    debug(f"GPIO setup error: {e}", "ERROR")
    raise

# === MAVLink Connections ===
def connect_mavlink():
    try:
        mav_drone = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
        mav_base = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
        debug("MAVLink ports connected")

        mav_drone.wait_heartbeat(timeout=10)
        debug("Drone heartbeat received")
        mav_base.wait_heartbeat(timeout=10)
        debug("Base heartbeat received")
        return mav_drone, mav_base
    except Exception as e:
        debug(f"MAVLink connection failed: {e}", "ERROR")
        return None, None

mav_drone, mav_base = connect_mavlink()

# === Servo Control ===
def set_angle(logical_az, elevation):
    try:
        physical_az = logical_az / GEAR_RATIO
        physical_el = elevation / GEAR_RATIO

        duty_az = max(2.5, min(12.5, 2.5 + (physical_az * 10.0 / 180.0)))
        duty_el = max(2.5, min(12.5, 2.5 + (physical_el * 10.0 / 180.0)))

        pwm_azi.ChangeDutyCycle(duty_az)
        pwm_ele.ChangeDutyCycle(duty_el)
        debug(f"Set angles → Azimuth: {logical_az:.2f}°, Elevation: {elevation:.2f}°")

        time.sleep(0.5)
        pwm_azi.ChangeDutyCycle(0)
        pwm_ele.ChangeDutyCycle(0)
    except Exception as e:
        debug(f"Servo error: {e}", "ERROR")

# === GPS Threads ===
def update_gps(mav, gps_dict, lock, label):
    debug(f"Starting {label} GPS thread")
    while True:
        try:
            msg = mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if msg:
                with lock:
                    gps_dict['lat'] = msg.lat / 1e7
                    gps_dict['lon'] = msg.lon / 1e7
                    gps_dict['alt'] = msg.alt / 1000.0
                if VERBOSE_GPS:
                    debug(f"[{label} GPS] Lat: {gps_dict['lat']:.7f}, Lon: {gps_dict['lon']:.7f}, Alt: {gps_dict['alt']:.2f}m")
            else:
                debug(f"{label} GPS timeout", "WARN")
        except Exception as e:
            debug(f"{label} GPS error: {e}", "ERROR")
            time.sleep(1)

# === Movement Logic ===
def move_to(az_target, el_target, step=STEP_SIZE, delay=0.05):
    global servo_logical_azimuth_angle, servo_elevation_angle

    max_steps = 500
    for _ in range(max_steps):
        delta_az = (az_target - servo_logical_azimuth_angle + 540) % 360 - 180
        delta_el = el_target - servo_elevation_angle

        if abs(delta_az) <= 1.0 and abs(delta_el) <= 1.0:
            debug("Target reached")
            break

        step_az = step * (1 if delta_az > 0 else -1) if abs(delta_az) > step else delta_az
        step_el = step * (1 if delta_el > 0 else -1) if abs(delta_el) > step else delta_el

        servo_logical_azimuth_angle = (servo_logical_azimuth_angle + step_az) % 360
        servo_elevation_angle = max(0, min(180, servo_elevation_angle + step_el))

        az_adj, el_adj = tracker.adjust_angles_for_servo_limits(
            servo_logical_azimuth_angle, servo_elevation_angle)

        set_angle(az_adj, el_adj)
        time.sleep(delay)

# === Tracking ===
def calculate_angles():
    with drone_gps_lock, base_gps_lock:
        if not all([drone_gps["lat"], drone_gps["lon"], drone_gps["alt"]]):
            debug("Drone GPS unavailable", "WARN")
            return None, None

        return tracker.calculate_azimuth_elevation(
            base_gps["lat"], base_gps["lon"], base_gps["alt"],
            drone_gps["lat"], drone_gps["lon"], drone_gps["alt"]
        )

def tracking_loop():
    debug("Tracking started")
    while True:
        try:
            az, el = calculate_angles()
            if az is not None and el is not None:
                debug(f"Target Az={az:.2f}°, El={el:.2f}°")
                move_to(az, el)
            time.sleep(1)
        except KeyboardInterrupt:
            debug("Tracking interrupted")
            break
        except Exception as e:
            debug(f"Tracking error: {e}", "ERROR")
            time.sleep(5)

# === Main ===
def main():
    debug("Starting system...")
    set_angle(servo_logical_azimuth_angle, servo_elevation_angle)

    if mav_drone:
        threading.Thread(target=update_gps, args=(mav_drone, drone_gps, drone_gps_lock, "Drone"), daemon=True).start()
    if mav_base:
        threading.Thread(target=update_gps, args=(mav_base, base_gps, base_gps_lock, "Base"), daemon=True).start()

    time.sleep(5)
    tracking_loop()

def cleanup():
    debug("Cleaning up")
    pwm_azi.stop()
    pwm_ele.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        debug("Exited by user")
    finally:
        cleanup()
