MODE_DEFAULT        = "ground"         # "sim" or "ground"
BASE_MODE_DEFAULT   = "static"   # "dynamic" or "static"   (only used in ground mode)

# --- Endpoints ---
SIM_DRONE_ENDPOINT = "udp:0.0.0.0:14550"
SIM_DRONE_BAUD     = None
DRONE_ENDPOINT     = "/dev/ttyACM0"
DRONE_BAUD         = 57600
BASE_ENDPOINT      = "/dev/ttyUSB0"
BASE_BAUD          = 57600

# MAVLink stream requests (~ 5 Hz)
MAV_MSG_INTERVAL_US_GPS    = 200_000
MAV_MSG_INTERVAL_US_GLOBAL = 200_000

# Gear spokes ratios
AZ_SPOKES_SMALL   = 12
AZ_SPOKES_BIG     = 24
EL_SPOKES_SMALL   = 12
EL_SPOKES_BIG     = 24
AZ_GEAR_RATIO     = AZ_SPOKES_BIG / AZ_SPOKES_SMALL    # default 2.0
EL_GEAR_RATIO     = EL_SPOKES_BIG / EL_SPOKES_SMALL    # default 2.0

# Physical limits
AZ_PHYS_MIN = 0.0
AZ_PHYS_MAX = 350.0
EL_PHYS_MIN = 0.0
EL_PHYS_MAX = 180.0

# Servo mapping – 180° servo
PULSE_MIN_US    = 900.0
PULSE_MAX_US    = 2100.0
SERVO_RANGE_DEG = 180.0

# Calibration
AZIMUTH_ZERO_OFFSET_DEG   = 0.0
ELEVATION_ZERO_OFFSET_DEG = 90.0

AZIMUTH_INVERT   = True
ELEVATION_INVERT = False  # set True if your rig needs "up is up"

# pigpio GPIO pins
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 17 

# Loop timings
UPDATE_PERIOD_S = 0.10   # 10 Hz default (set higher if you want servo with slower updates)
PRINT_PERIOD_S  = 1.0

EL_MIN_WORLD_DEG = 0.0
EL_MAX_WORLD_DEG = 90.0

# SIM base (used only in SIM mode)
base_static = {
    "lat": 13.0281865,
    "lon": 77.5675790,
    "alt": 931.13,          # metres ASL
}

EL_US_MIN, EL_US_MAX = 900.0, 2100.0
US_PER_SERVO_DEG     = (EL_US_MAX - EL_US_MIN) / 180.0   # 6.666... us/deg