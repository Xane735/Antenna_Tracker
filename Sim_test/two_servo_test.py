import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)

# Servo pins
SERVO1_PIN = 18  # GPIO18, Pin 12 , Azimuth servo
SERVO2_PIN = 13  # GPIO13, Pin 33 , Elevation servo

# Setup pins as output
GPIO.setup(SERVO1_PIN, GPIO.OUT)
GPIO.setup(SERVO2_PIN, GPIO.OUT)

# Create PWM instances at 50Hz
servo1 = GPIO.PWM(SERVO1_PIN, 50)
servo2 = GPIO.PWM(SERVO2_PIN, 50)

# Start PWM with neutral duty cycle (7.5%)
servo1.start(0)
servo2.start(0)

def set_angle(servo, angle):
    duty = 2.5 + (angle / 18)  # Map 0–180° to 2.5–12.5% duty
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)  # Stop signal to prevent jitter

try:
    while True:
        print("Moving both servos to 0°")
        set_angle(servo1, 0)
        set_angle(servo2, 0)
        time.sleep(1)

        print("Moving both servos to 90°")
        set_angle(servo1, 90)
        set_angle(servo2, 90)
        time.sleep(1)

        print("Moving both servos to 180°")
        set_angle(servo1, 180)
        set_angle(servo2, 180)
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping...")
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
