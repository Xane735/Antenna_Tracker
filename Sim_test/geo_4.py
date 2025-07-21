
# lauki_sitl.py — Antenna Tracker using SITL simulation and real servos

import threading
import time
import math
import azi_elev_5 as tracker
import RPi.GPIO as GPIO
from pymavlink import mavutil
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np

# === Config ===
DEBUG = True
USE_SMOOTH_MOVEMENT = True
LOG_TO_CSV = True
GENERATE_RADAR_PLOT = True

GEAR_RATIO = 2.0
STEP_SIZE = 1.0
TRACKING_UPDATE_RATE = 1
BASE_LAT = 12.9716
BASE_LON = 77.5946
BASE_ALT = 920.0  # meters

servo_logical_azimuth_angle = 90.0
servo_elevation_angle = 45.0

drone_gps = {"lat": None, "lon": None, "alt": None}
drone_gps_lock = threading.Lock()

if LOG_TO_CSV:
    log_file = open("antenna_tracking_log.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "timestamp", "drone_lat", "drone_lon", "drone_alt",
        "base_lat", "base_lon", "base_alt",
        "azimuth", "elevation",
        "servo_azimuth_input", "servo_elevation_input",
        "horizontal_distance", "slant_range"
    ])

def debug(msg):
    if DEBUG:
        print(f"[{time.strftime('%H:%M:%S')}] {msg}")

def set_angle(logical_az, elevation):
    try:
        physical_az = logical_az / GEAR_RATIO
        physical_el = elevation / GEAR_RATIO

        duty_az = 2.5 + (physical_az * 10.0 / 180.0)
        duty_el = 2.5 + (physical_el * 10.0 / 180.0)
        duty_az = max(2.5, min(12.5, duty_az))
        duty_el = max(2.5, min(12.5, duty_el))

        pwm_azi.ChangeDutyCycle(duty_az)
        pwm_ele.ChangeDutyCycle(duty_el)
        time.sleep(0.5)
        pwm_azi.ChangeDutyCycle(0)
        pwm_ele.ChangeDutyCycle(0)
        debug(f"Set angle: Az={logical_az:.2f}, El={elevation:.2f}")
    except Exception as e:
        debug(f"Servo error: {e}")

def move_to(az_target, el_target, step=STEP_SIZE, delay=0.05):
    global servo_logical_azimuth_angle, servo_elevation_angle

    if not USE_SMOOTH_MOVEMENT:
        servo_logical_azimuth_angle = az_target
        servo_elevation_angle = el_target
        adj_az, adj_el = tracker.adjust_angles_for_servo_limits(az_target, el_target)
        set_angle(adj_az, adj_el)
        return

    for _ in range(500):
        delta_az = (az_target - servo_logical_azimuth_angle + 540) % 360 - 180
        delta_el = el_target - servo_elevation_angle

        if abs(delta_az) <= 1 and abs(delta_el) <= 1:
            break

        step_az = step if delta_az > 0 else -step if abs(delta_az) > step else delta_az
        step_el = step if delta_el > 0 else -step if abs(delta_el) > step else delta_el

        servo_logical_azimuth_angle = (servo_logical_azimuth_angle + step_az) % 360
        servo_elevation_angle = max(0, min(180, servo_elevation_angle + step_el))

        adj_az, adj_el = tracker.adjust_angles_for_servo_limits(
            servo_logical_azimuth_angle, servo_elevation_angle)

        set_angle(adj_az, adj_el)
        time.sleep(delay)

def log_to_csv(info):
    if not LOG_TO_CSV:
        return
    with drone_gps_lock:
        log_writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            drone_gps["lat"], drone_gps["lon"], drone_gps["alt"],
            BASE_LAT, BASE_LON, BASE_ALT,
            info["azimuth"], info["elevation"],
            round(info["adjusted_azimuth"] / GEAR_RATIO, 2),
            round(info["adjusted_elevation"] / GEAR_RATIO, 2),
            info["horizontal_distance"], info["slant_range"]
        ])
        log_file.flush()

def tracking_loop():
    while True:
        try:
            with drone_gps_lock:
                lat, lon, alt = drone_gps["lat"], drone_gps["lon"], drone_gps["alt"]
            if not all([lat, lon, alt]):
                debug("Drone GPS not ready")
                time.sleep(1)
                continue

            info = tracker.get_tracking_info(BASE_LAT, BASE_LON, BASE_ALT, lat, lon, alt)
            log_to_csv(info)
            move_to(info["adjusted_azimuth"], info["adjusted_elevation"])
            time.sleep(TRACKING_UPDATE_RATE)
        except KeyboardInterrupt:
            break
        except Exception as e:
            debug(f"Tracking error: {e}")
            time.sleep(2)

def update_drone_gps(mav):
    debug("Started drone GPS thread")
    while True:
        try:
            msg = mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if msg:
                with drone_gps_lock:
                    drone_gps["lat"] = msg.lat / 1e7
                    drone_gps["lon"] = msg.lon / 1e7
                    drone_gps["alt"] = msg.alt / 1000.0
        except Exception as e:
            debug(f"GPS error: {e}")
            time.sleep(1)

def generate_radar_plot():
    try:
        azimuths = []
        with open("antenna_tracking_log.csv") as f:
            reader = csv.DictReader(f)
            for row in reader:
                azimuths.append(float(row["azimuth"]))
        theta = np.deg2rad(azimuths)
        r = np.ones_like(theta)
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, polar=True)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.plot(theta, r, 'b.-')
        ax.set_title("Azimuth Radar Plot")
        fig.savefig("azimuth_radar_plot.png")
        debug("Radar plot saved to azimuth_radar_plot.png")
    except Exception as e:
        debug(f"Plot error: {e}")

def cleanup():
    pwm_azi.stop()
    pwm_ele.stop()
    GPIO.cleanup()
    if LOG_TO_CSV:
        log_file.close()
    if GENERATE_RADAR_PLOT:
        generate_radar_plot()
    debug("Cleanup done.")

def main():
    global pwm_azi, pwm_ele
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)
    pwm_azi = GPIO.PWM(18, 50)
    pwm_ele = GPIO.PWM(13, 50)
    pwm_azi.start(0)
    pwm_ele.start(0)
    set_angle(servo_logical_azimuth_angle, servo_elevation_angle)

    mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
    mav.wait_heartbeat()
    debug("Connected to SITL")

    threading.Thread(target=update_drone_gps, args=(mav,), daemon=True).start()
    time.sleep(5)
    tracking_loop()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        debug("Exiting")
    finally:
        cleanup()
