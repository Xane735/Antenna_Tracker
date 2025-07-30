import RPi.GPIO as GPIO
import time

# === Config ===
GPIO.setmode(GPIO.BCM)
AZI_PIN = 18
ELE_PIN = 13

# Servo mechanical limits (degrees on the servo horn)
SERVO_MIN_DEG = 0
SERVO_MAX_DEG = 180   # your servo capability

# External gear ratio (physical_out : servo)
GEAR_RATIO = 2.0      # 2:1 -> physical angle = 2 * servo angle

# Derived physical limits (degrees at the output shaft after gearing)
PHYS_MIN_DEG = SERVO_MIN_DEG * GEAR_RATIO
PHYS_MAX_DEG = SERVO_MAX_DEG * GEAR_RATIO  # 240° for a 2:1 gear with a 120° servo

# Sweep settings (in PHYSICAL degrees)
PHYS_STEP = 30
PHYS_SWEEP_MAX = PHYS_MAX_DEG  # set to 180 if you want 0–180° physical sweep

# === GPIO Setup ===
GPIO.setup(AZI_PIN, GPIO.OUT)
GPIO.setup(ELE_PIN, GPIO.OUT)

pwm_azi = GPIO.PWM(AZI_PIN, 50)  # 50 Hz for standard servo
pwm_ele = GPIO.PWM(ELE_PIN, 50)

pwm_azi.start(0)
pwm_ele.start(0)

# === Helpers ===
def set_servo_angle(pwm, angle_servo_deg):
    """
    Move the SERVO to angle_servo_deg (servo-side degrees).
    Scales 0-120° to 5-10% duty cycle.
    """
    # Clamp to servo limits
    angle_servo_deg = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, angle_servo_deg))
    duty = 5 + (angle_servo_deg * 5 / float(SERVO_MAX_DEG))  # 5–10% duty over 0–120°
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)  # allow movement
    pwm.ChangeDutyCycle(0)  # stop pulses to reduce jitter

def set_physical_angle(pwm, angle_phys_deg):
    """
    Move the OUTPUT (post-gear) to angle_phys_deg (physical degrees).
    Converts to servo-side angle using the gear ratio.
    Returns the servo angle actually commanded.
    """
    # Clamp to physical limits (this also protects the servo clamp indirectly)
    angle_phys_deg = max(PHYS_MIN_DEG, min(PHYS_MAX_DEG, angle_phys_deg))

    # Convert physical to servo angle
    angle_servo_deg = angle_phys_deg / GEAR_RATIO

    # Command the servo
    set_servo_angle(pwm, angle_servo_deg)

    return angle_servo_deg

try:
    print(f"Sweeping Azimuth and Elevation servos with gear ratio {GEAR_RATIO}:1 (physical:servo)")
    print(f"Servo limits: {SERVO_MIN_DEG}–{SERVO_MAX_DEG}°, Physical limits: {PHYS_MIN_DEG}–{PHYS_MAX_DEG}°")

    # Up-sweep in PHYSICAL degrees
    while True:
        # Up sweep
        phys = PHYS_MIN_DEG
        while phys <= PHYS_SWEEP_MAX:
            servo_cmd_azi = set_physical_angle(pwm_azi, phys)
            servo_cmd_ele = set_physical_angle(pwm_ele, phys)
            print(f"Moving to {phys:.0f}° physical "
                  f"(servo AZI {servo_cmd_azi:.1f}°, ELE {servo_cmd_ele:.1f}°)")
            time.sleep(0.5)
            phys += PHYS_STEP

        # Down sweep
        phys = PHYS_SWEEP_MAX
        while phys >= PHYS_MIN_DEG:
            servo_cmd_azi = set_physical_angle(pwm_azi, phys)
            servo_cmd_ele = set_physical_angle(pwm_ele, phys)
            print(f"Moving to {phys:.0f}° physical "
                  f"(servo AZI {servo_cmd_azi:.1f}°, ELE {servo_cmd_ele:.1f}°)")
            time.sleep(0.5)
            phys -= PHYS_STEP

except KeyboardInterrupt:
    print("Exiting...")

finally:
    pwm_azi.stop()
    pwm_ele.stop()
    GPIO.cleanup()
