import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

SERVO1_PIN = 18  # Azimuth servo
SERVO2_PIN = 13  # Elevation servo

GPIO.setup(SERVO1_PIN, GPIO.OUT)
GPIO.setup(SERVO2_PIN, GPIO.OUT)

servo1 = GPIO.PWM(SERVO1_PIN, 50)
servo2 = GPIO.PWM(SERVO2_PIN, 50)

servo1.start(0)
servo2.start(0)

def set_angle(servo, angle):
    duty = 2.5 + (angle / 18)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

def sweep_azimuth_elevation():
    print("Simulating 360° azimuth and 180° elevation range...\n")

    # Step 1: 0° to 180° azimuth with normal elevation
    for az in range(0, 181, 30):
        el = 90  # Mid elevation
        print(f"Azimuth: {az}°, Elevation: {el}°")
        set_angle(servo1, az)
        set_angle(servo2, el)
        time.sleep(0.5)

    # Step 2: 180°–360° simulated by mirroring azimuth and flipping elevation
    for az in range(0, 181, 30):  # Again 0 to 180
        flipped_az = az
        flipped_el = 180 - 90  # Flip elevation around 90 (can vary if needed)
        print(f"Azimuth: {az+180}° (flipped to {flipped_az}°), Elevation flipped to: {flipped_el}°")
        set_angle(servo1, flipped_az)
        set_angle(servo2, flipped_el)
        time.sleep(0.5)

    # Step 3: Sweep elevation 0° to 180° at azimuth = 90°
    print("\nSweeping Elevation from 0° to 180° at fixed Azimuth = 90°")
    set_angle(servo1, 90)
    for el in range(0, 181, 30):
        print(f"Azimuth: 90°, Elevation: {el}°")
        set_angle(servo2, el)
        time.sleep(0.5)

try:
    sweep_azimuth_elevation()

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
