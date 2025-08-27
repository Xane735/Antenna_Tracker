MODE_DEFAULT = "ground"         # "sim" or "ground"
BASE_MODE_DEFAULT = "static"   # "dynamic" or "static"   (only used in ground mode)

# --- Endpoints ---
SIM_DRONE_ENDPOINT = "udp:0.0.0.0:14550"
SIM_DRONE_BAUD     = None
DRONE_ENDPOINT     = "/dev/ttyACM0"
DRONE_BAUD         = 57600
BASE_ENDPOINT      = "/dev/ttyACM1"
BASE_BAUD          = 57600

# MAVLink stream requests (~5 Hz)
MAV_MSG_INTERVAL_US_GPS    = 200_000
MAV_MSG_INTERVAL_US_GLOBAL = 200_000

# Gear spokes ratios
SPOKES_SMALL   = 12
SPOKES_BIG     = 24
AZ_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL    # default 2.0
EL_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL    # default 2.0

# Physical limits
AZ_PHYS_MIN = 0.0
AZ_PHYS_MAX = 350.0
EL_PHYS_MIN = 0.0
EL_PHYS_MAX = 180.0

# Servo mapping — 180° servo
PULSE_MIN_US    = 900.0
PULSE_MAX_US    = 2100.0
SERVO_RANGE_DEG = 180.0

# Calibration
AZIMUTH_ZERO_OFFSET_DEG   = 0.0
ELEVATION_ZERO_OFFSET_DEG = 90.0
AZIMUTH_INVERT   = True
ELEVATION_INVERT = False  # set True if your rig needs "up is up"

# pigpio GPIO pins (BCM numbering)
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 17  # per your hardware

# Loop timings
UPDATE_PERIOD_S = 0.10   # 10 Hz default (set higher if your servos prefer slower updates)
PRINT_PERIOD_S  = 1.0
LOG_TO_CSV      = True
LOG_RAW_GPS     = True   # per-message GPS capture (for precision analysis)

EL_MIN_WORLD_DEG = 0.0
EL_MAX_WORLD_DEG = 90.0

EL_US_MIN, EL_US_MAX = 900.0, 2100.0
US_PER_SERVO_DEG = (EL_US_MAX - EL_US_MIN) / 180.0   # 6.666... µs/deg

def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def world_el_to_us(world_el_deg: float) -> int:
    # clamp to 0..90 so you never drive below horizon or beyond straight up
    w = _clamp(world_el_deg, EL_MIN_WORLD_DEG, EL_MAX_WORLD_DEG)

    # mapping for your 2:1 gear and 1500 µs (servo 90°) = sky:
    # world_el: 0:horizon, + 90:sky
    # servo_deg = (world_el + 90)/2
    servo_deg = (w + 90.0) / 2.0

    us = 1500.0 + (servo_deg - 90.0) * US_PER_SERVO_DEG
    # final safety clamp to mechanical µs range
    if us < EL_US_MIN: us = EL_US_MIN
    if us > EL_US_MAX: us = EL_US_MAX
    return int(us)

# SIM base
base_static = {
    "lat": 13.0277429,
    "lon": 77.5631762,
    "alt": 931.13,          # metres ASL
}