import pigpio
import time
import config
import servo_map as servo

# ===================== pigpio & parking =====================

def setup():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running (sudo pigpiod)")
    pi.set_mode(config.SERVO_AZ_PIN, pigpio.OUTPUT)
    pi.set_mode(config.SERVO_EL_PIN, pigpio.OUTPUT)
    print(f"[INFO] pigpio ready (AZ={config.SERVO_AZ_PIN}, EL={config.SERVO_EL_PIN})")
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
    """Ramp both servos smoothly to target servo degrees over duration_s, without the initial twitch."""
    # Clamp to valid servo range
    t_az_deg = max(0.0, min(config.SERVO_RANGE_DEG, float(target_servo_deg_az)))
    t_el_deg = max(0.0, min(config.SERVO_RANGE_DEG, float(target_servo_deg_el)))

    target_us_az = servo.deg_to_us_az(t_az_deg)
    target_us_el = servo.deg_to_us(t_el_deg)

    # IMPORTANT: if pigpio returns 0 (unknown), assume we're already at TARGET to avoid jumping to 1500 µs first.
    start_us_az = _get_start_us(pi, config.SERVO_AZ_PIN, default_us=target_us_az)
    start_us_el = _get_start_us(pi, config.SERVO_EL_PIN, default_us=target_us_el)

    # Small deadband to suppress one-tick flicker
    DEAD_US = 8.0
    d_az = abs(target_us_az - start_us_az)
    d_el = abs(target_us_el - start_us_el)

    # If we're already "there", just enable PWM at target and exit
    if d_az <= DEAD_US and d_el <= DEAD_US:
        pi.set_servo_pulsewidth(config.SERVO_AZ_PIN, target_us_az)
        pi.set_servo_pulsewidth(config.SERVO_EL_PIN, target_us_el)
        return

    # Prime outputs to the measured start (turn PWM on cleanly), then ramp
    pi.set_servo_pulsewidth(config.SERVO_AZ_PIN, start_us_az)
    pi.set_servo_pulsewidth(config.SERVO_EL_PIN, start_us_el)
    time.sleep(0.02)

    steps = max(1, int(float(duration_s) * float(rate_hz)))
    for i in range(1, steps + 1):
        a = i / steps
        us_az = _bounded_us(start_us_az + (target_us_az - start_us_az) * a)
        us_el = _bounded_us(start_us_el + (target_us_el - start_us_el) * a)
        pi.set_servo_pulsewidth(config.SERVO_AZ_PIN, us_az)
        pi.set_servo_pulsewidth(config.SERVO_EL_PIN, us_el)
        time.sleep(1.0 / rate_hz)
