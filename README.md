# MAVLink GPS Antenna Tracker

A high-performance, Raspberry Pi-based antenna tracker for automatically pointing a directional antenna at a drone. It uses MAVLink GPS data from both the ground station (base) and the drone to calculate the required azimuth and elevation, driving two servos for precise positioning.


## Features
Dual MAVLink GPS Input: Utilizes GPS data from both the drone and a ground-based receiver for accurate relative positioning.

1. Multiple Operating Modes:

Ground Mode : For live operation with physical hardware.
             - Static Mode: Locks onto the base station's position after a fixed time.
             - Dynamic Mode: Uses real time live positioning of base station.

Sim Mode: For testing and simulation without hardware, uses hardcoded GPS coordinates for the base sation.

2. Flexible Base Station Modes:

Static: Averages the base station's position for a few seconds upon startup and then locks it, ideal for fixed setups.

Dynamic: Continuously uses the latest GPS data from the base, suitable for moving ground stations.

Advanced Overhead Tracking ("Backside Flip"): Decides whether to track the drone from the "front" (0-180° azimuth) or "back" (180-360° azimuth) by rotating 180°, ensuring continuous tracking even when the drone flies directly overhead and behind the tracker.

3. Smooth & Stable Motion:

*Hysteresis & Cooldowns*: Prevents rapid, oscillating movements when the drone is near a flip point.

*Rate Limiting*: Caps the maximum rotational speed of the servos to avoid sudden jumps.

*Smooth Parking*: Gently moves servos to a defined home or last-known position on startup and shutdown.

*Highly Configurable*: Easily adjust gear ratios, servo pulse widths, physical limits, calibration offsets, and tracking dynamics directly in the script.

*Data Logging*: Logs detailed tracking data (world angles, servo positions, GPS coordinates) and raw GPS messages to .csv files for analysis.

# Hardware Requirements
Raspberry Pi: Any model with GPIO pins should work (e.g., Pi 3B+, Pi 4).

Pan-Tilt Mechanism: A 2-axis mount for your antenna.

Servos: Two standard servos for azimuth and elevation control.

MAVLink Radios:

One telemetry radio (e.g., SiK radio) to receive the drone's MAVLink stream.

One GPS receiver with a MAVLink-capable output (e.g., a U-Blox GPS configured to output MAVLink) for the base station.

Power Supply: A stable power source for the Raspberry Pi and servos.

BEC: One supplying 7.2V and another 5 V

Logic Level Converter: 5V - 3V.

**Software & Installation**
This project runs on Python and requires the pigpio library for hardware control.

Install pigpio Daemon: The pigpio library requires a system daemon to be running.

Bash

# Update package lists
```
sudo apt-get update
```
# Install pigpio
```
sudo apt-get install pigpio
```
# Enable and start the daemon
```
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```
Clone the Repository:

Bash
```
git clone <your-repository-url>
cd <your-repository-name>
Set up a Python Environment: (Recommended)
```
Bash
```
python3 -m venv venv
source venv/bin/activate
```
Install Dependencies:

Bash
```
pip install pigpio pymavlink geopy
```
Configuration
All primary configuration is done within the top section of the geo_13.py script. You must review and adjust these settings to match your specific hardware.

Endpoints:

DRONE_ENDPOINT: The serial port for your drone's telemetry radio (e.g., /dev/ttyUSB0).

BASE_ENDPOINT: The serial port for your base station's GPS (e.g., /dev/ttyACM0).

Hardware Mapping:

SERVO_AZ_PIN, SERVO_EL_PIN: The GPIO pins your servos are connected to.

AZ_GEAR_RATIO, EL_GEAR_RATIO: The gear ratio between your servo and the final output (e.g., 2.0 if a 12-tooth servo gear drives a 24-tooth platform gear).

PULSE_MIN_US, PULSE_MAX_US: The minimum and maximum servo pulse widths in microseconds. Calibrate this to match your servo's 0° and 180° positions.

Calibration:

AZIMUTH_ZERO_OFFSET_DEG, ELEVATION_ZERO_OFFSET_DEG: Add a correction angle if your tracker's physical "zero" position is not perfectly aligned with true North.

AZIMUTH_INVERT, ELEVATION_INVERT: Set to True if a servo is moving in the wrong direction.

Flip Behavior Tuning:

These parameters control the advanced "backside flip" logic. The defaults are a good starting point.

FLIP_HYSTERESIS_DEG: Adds "stickiness" to the current tracking side to prevent rapid flipping if the drone is hovering at a seam.

EDGE: How close to the 0° or 180° servo limit the tracker must be before a flip is considered.

MIN_EL_FOR_FLIP: Prevents flipping if the drone is too low to the horizon.

**Usage**
Run the main script from the terminal. Use command-line arguments to override default behaviors.

Bash
```
python geo_13.py --mode [mode] --base-mode [base_mode]
```

Key Arguments:
```
--mode: ground for real hardware (default) or sim for simulation.

--base-mode: static (default) to lock the base position after 10 seconds, or dynamic to use a continuously moving base.

--static-window-sec: The number of seconds to average the base GPS in static mode.

--park-home-az, --park-home-el: The world angles (in degrees) to park the tracker at on shutdown.
```

Examples:
Standard Ground Operation:
(Uses a fixed base position after a 10-second calibration window)

Bash
```
python geo_13.py --mode ground --base-mode static
```
Operation with a Moving Base Station:
(For example, if the tracker is on a boat or car)

Bash
```
python geo_13.py --mode ground --base-mode dynamic
```