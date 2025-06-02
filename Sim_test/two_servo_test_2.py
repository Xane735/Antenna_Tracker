# This code is for the Bigger Tracker with defined gear ratios.
import RPi.GPIO as GPIO
import time

# === Servo pins ===
SERVO_AZI_PIN = 18  # GPIO18
SERVO_ELE_PIN = 13  # GPIO13

# === Gear Ratio ===
GEAR_RATIO = 2.0  # Servo moves 2° for every 1° antenna motion

# === GPIO setup ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_AZI_PIN, GPIO.OUT)
GPIO.setup(SERVO_ELE_PIN, GPIO.OUT)

# Set PWM frequency to 50Hz
servo_azi_pwm = GPIO.PWM(SERVO_AZI_PIN, 50)
servo_ele_pwm = GPIO.PWM(SERVO_ELE_PIN, 50)

servo_azi_pwm.start(0)
servo_ele_pwm.start(0)

def angle_to_duty(angle):
    return 2.5 + (angle / 18.0)  # Standard for SG90-type servos

def move_servo(azi_angle, ele_angle):
    servo_az = GEAR_RATIO * azi_angle
    servo_el = GEAR_RATIO * ele_angle

    # Clamp servo angle to safe range (0-180°)
    servo_az = max(0, min(180, servo_az))
    servo_el = max(0, min(180, servo_el))

    duty_az = angle_to_duty(servo_az)
    duty_el = angle_to_duty(servo_el)

    print(f"Commanding Antenna Azimuth: {azi_angle}°, Elevation: {ele_angle}°")
    print(f"Servo Target Azimuth: {servo_az}°, Elevation: {servo_el}°")

    servo_azi_pwm.ChangeDutyCycle(duty_az)
    servo_ele_pwm.ChangeDutyCycle(duty_el)

    time.sleep(0.5)

    # Stop signal to prevent jitter
    servo_azi_pwm.ChangeDutyCycle(0)
    servo_ele_pwm.ChangeDutyCycle(0)

try:
    # Sweep from 0 to 90 antenna degrees
    for angle in range(0, 91, 15):  # Test in 15° steps
        move_servo(angle, angle)
        time.sleep(1)

    # Return to 0
    move_servo(0, 0)

except KeyboardInterrupt:
    print("Interrupted.")

finally:
    servo_azi_pwm.stop()
    servo_ele_pwm.stop()
    GPIO.cleanup()
