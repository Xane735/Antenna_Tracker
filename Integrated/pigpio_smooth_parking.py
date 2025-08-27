import pigpio
import constants as con
import time
import helper_fns as hf

# ============= pigpio & Smooth Parking =============
def setup_pigpio():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running (sudo pigpiod)")
    pi.set_mode(con.SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(con.SERVO_EL_PIN, pigpio.OUTPUT)
    print(f"[INFO] pigpio ready (AZ={con.SERVO_AZ_PIN}, EL={con.SERVO_EL_PIN})")
    return pi

def _bounded_us(x: float) -> float:
    return max(500.0, min(2500.0, x))

def _get_start_us(pi, pin: int, default_us: float = 1500.0) -> float:
    try:
        val = float(pi.get_servo_pulsewidth(pin))
        if 500.0 <= val <= 2500.0:
            return val
    except Exception:
        pass
    return default_us

def smooth_park(pi, target_servo_deg_az: float, target_servo_deg_el: float,
                duration_s: float = 1.5, rate_hz: float = 60.0):
    """Ramp both servos smoothly to target servo degrees over duration_s."""
    target_us_az = hf.servo_deg_to_us(max(0.0, min(con.SERVO_RANGE_DEG, target_servo_deg_az)))
    target_us_el = hf.servo_deg_to_us(max(0.0, min(con.SERVO_RANGE_DEG, target_servo_deg_el)))

    start_us_az = _get_start_us(pi, con.SERVO_AZ_PIN, default_us=1500.0)
    start_us_el = _get_start_us(pi, con.SERVO_EL_PIN, default_us=1500.0)

    steps = max(1, int(duration_s * rate_hz))
    for i in range(1, steps + 1):
        a = i / steps
        us_az = _bounded_us(start_us_az + (target_us_az - start_us_az) * a)
        us_el = _bounded_us(start_us_el + (target_us_el - start_us_el) * a)
        pi.set_servo_pulsewidth(con.SERVO_AZ_PIN, us_az)
        pi.set_servo_pulsewidth(con.SERVO_EL_PIN, us_el)
        time.sleep(1.0 / rate_hz)
