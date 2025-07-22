import RPi.GPIO as GPIO
import time

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
AZI_PIN = 18
ELE_PIN = 13

GPIO.setup(AZI_PIN, GPIO.OUT)
GPIO.setup(ELE_PIN, GPIO.OUT)

pwm_azi = GPIO.PWM(AZI_PIN, 50)  # 50 Hz for standard servo
pwm_ele = GPIO.PWM(ELE_PIN, 50)

pwm_azi.start(0)
pwm_ele.start(0)

# === Helper Function ===
def set_servo_angle(pwm, angle):
    # Clamp angle between 0–120° to match servo capability
    angle = max(0, min(120, angle))
    duty = 5 + (angle * 5 / 120.0)  # Scale to 5–10% duty for 0–120°
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)

try:
    print("Sweeping Azimuth and Elevation servos...")
    while True:
        for angle in range(0, 181, 30):
            print(f"Moving to {angle}°")
            set_servo_angle(pwm_azi, angle)
            set_servo_angle(pwm_ele, angle)
            time.sleep(0.5)
        for angle in range(180, -1, -30):
            print(f"Moving to {angle}°")
            set_servo_angle(pwm_azi, angle)
            set_servo_angle(pwm_ele, angle)
            time.sleep(0.5)
except KeyboardInterrupt:
    print("Exiting...")

finally:
    pwm_azi.stop()
    pwm_ele.stop()
    GPIO.cleanup()
