# ===================== Defaults =====================

MODE_DEFAULT = "ground"          # "sim" or "ground"
BASE_MODE_DEFAULT = "static"     # "static" (freeze after 10s) or "dynamic"

# --- Endpoints ---
SIM_DRONE_ENDPOINT = "udp:0.0.0.0:14550"
SIM_DRONE_BAUD     = None
DRONE_ENDPOINT     = "/dev/ttyUSB1"
DRONE_BAUD         = 57600
BASE_ENDPOINT      = "/dev/ttyACM1"
BASE_BAUD          = 57600

# MAVLink stream requests
MAV_MSG_INTERVAL_US_GPS    = 200_000   # microseconds - 5 Hz (typical GPS) 
MAV_MSG_INTERVAL_US_GLOBAL = 50_000    # microseconds - 20 Hz (raise to 50_000 for ~20 Hz if supported)

# Gear spokes ratios
SPOKES_SMALL   = 12
SPOKES_BIG     = 24
AZ_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL    
EL_GEAR_RATIO  = SPOKES_BIG / SPOKES_SMALL

# Physical limits
AZ_PHYS_MIN = 0.0   # deg
AZ_PHYS_MAX = 360.0 # deg
EL_PHYS_MIN = 0.0   # deg
EL_PHYS_MAX = 180.0 # deg

# Servo mapping — 180° servo
PULSE_MIN_US    = 900.0  # microseconds
PULSE_MAX_US    = 1950.0 # microseconds
SERVO_RANGE_DEG = 180.0  # degrees

# Calibration
AZIMUTH_ZERO_OFFSET_DEG   = 0.0     # deg
ELEVATION_ZERO_OFFSET_DEG = 0.0     # deg
AZIMUTH_INVERT   = True             
ELEVATION_INVERT = False

# pigpio GPIO pins
SERVO_AZ_PIN = 18
SERVO_EL_PIN = 17

# Loop timings
UPDATE_PERIOD_S = 0.05   # main loop period; try 0.02–0.05 for snappier updates
PRINT_PERIOD_S  = 1.0    # How crowded you want your CLI to look like
LOG_TO_CSV      = True
LOG_RAW_GPS     = False # Make sure to remove once everything works. Most useless feature I've added *smh smh*

# SIM base (used only in SIM mode)
base_static = {"lat": 13.0276802, "lon": 77.5629616, "alt": 924.36}

# --- Tracking dynamics (Raise the values to 300-360, but can make it jerky or jumpy) -> How fast the antenna rotates ---
AZ_MAX_DEG_PER_SEC   = 180.0
EL_MAX_DEG_PER_SEC   = 180.0

# --- Flip cooldown / stickiness ---
MIN_FLIP_DWELL_S            = 2     # block any re-flip for 2 s
MIN_AZ_DELTA_SINCE_FLIP_DEG = 20.0  # need to move away from seam this much before reconsidering
FLIP_EXTRA_MARGIN_DEG       = 12.0  # extra benefit required to flip to the other side

ALLOW_BACKSIDE_FLIP = True          # Master switch for the flip logic. To be disabled if the drone isnt going to fly beyond 180 degrees elevation

EL_DEADBAND_DEG = 1.5      # ignore tiny EL changes (GPS jitter)
EL_MAX_DEG_PER_SEC = 60.0  # slow, steady elevation; AZ can stay 180+

""" 
    Hysteresis prevents the tracker from rapidly flipping back and forth if the drone is hovering right at a point where both the front and back poses are equally "good."
    It makes the current position "stickier" by adding a penalty to the alternative.
    The tracker will only flip if the backside pose is at least 10 servo degrees cheaper in movement than staying in the front pose.
    If you see it oscillating or "hesitating" at the flip point, increase this value. If it seems reluctant to flip when it should, decrease it.

"""
# Flip behavior tuning
ONLY_FLIP_NEAR_EDGE  = True    # KEEP THIS TRUE or you will break the tracker :)
EDGE                 = 10.0    # How close to the edge will the tracker flip. 
FLIP_HYSTERESIS_DEG  = 14.0    # Was 10.0; lets the new side win sooner
MIN_EL_FOR_FLIP      = 6.0     # Edit this if the elvation is passing through the tracker 

# Cost weighting: make azimuth more important than elevation near the seam
"""
If you want to make flips happen more readily, you could slightly decrease EL_WEIGHT (e.g., to 0.25) to make elevation movements even "cheaper" in the cost calculation.
"""
AZ_WEIGHT = 1.0
EL_WEIGHT = 0.30                # 0.25–0.35 works well

# If your rig wants “mirror elevation” use mirror_el; else try keep_el. Play with this if the elevation seems off after a flip.
FLIP_STYLE = "keep_el"          # "mirror_el" or "keep_el"

""" Edit these paramters to add a small bias to the azimuth/elevation ONLY when a flip occurs."""
FLIP_AZ_CORR_DEG = 0.0          # add/subtract small az bias ONLY when flipped
FLIP_EL_CORR_DEG = 0.0          # add/subtract small el bias ONLY when flipped

PARK_HOME_AZ = 0.0              # deg (world)
PARK_HOME_EL = 90.0             # deg (world)
PARK_DURATION = 1.5        # seconds
PARK_RATE = 60.0          # Hz
PARK_FACE_DRONE_START = True     # start parking by facing the drone
PARK_FACE_DRONE_EXIT  = True    # end parking by facing the drone
