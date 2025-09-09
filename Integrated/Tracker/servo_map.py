import config
from typing import Tuple
import gpio_parking as gpio

def physical_to_servo_deg(az_phys: float, el_phys: float) -> Tuple[float, float]:
    az_raw = az_phys / config.AZ_GEAR_RATIO
    el_raw = el_phys / config.EL_GEAR_RATIO
    az_servo = max(0.0, min(config.SERVO_RANGE_DEG, az_raw))
    el_servo = max(0.0, min(config.SERVO_RANGE_DEG, el_raw))
    return az_servo, el_servo

def deg_to_us(deg: float) -> float:
    # guard types for static checkers
    mn = float(config.PULSE_MIN_US)
    mx = float(config.PULSE_MAX_US)
    rng = float(config.SERVO_RANGE_DEG)
    d = max(0.0, min(rng, float(deg)))
    return mn + (d / rng) * (mx - mn)

# === Custom AZ mapping ===
# Fit through: (0°,900us), (180°,1400us), (360°,1950us)
# us(az_phys) = 900 + (az^2)/1296 + (95/36)*az
def az_phys_to_us(az_phys: float) -> float:
    az = max(0.0, min(360.0, float(az_phys)))
    us = 900.0 + (az * az) / 1296.0 + (95.0 / 36.0) * az
    return gpio._bounded_us(us)

def deg_to_us_az(servo_deg: float) -> float:
    # Convert servo degrees back to PHYSICAL az using gear ratio, then apply calibrated mapping
    d = max(0.0, min(config.SERVO_RANGE_DEG, float(servo_deg)))
    phys_az = d * float(config.AZ_GEAR_RATIO)
    return az_phys_to_us(phys_az)
