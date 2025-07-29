#!/usr/bin/env python3
"""
pan_range_test.py
-----------------
Standalone physical pan range test for a 2:1 geared servo mount.

What it does
- Computes your WORLD->PHYSICAL mapping using the same logic we used in geo_4.py
- Derives the 240° WORLD window (where the mount can track continuously)
- Sweeps across that window in WORLD azimuth, commanding the servo and measuring the
  PHYSICAL azimuth reached each step
- Prints PHYSICAL min / max / total span so you can verify your true usable travel

Run on your Raspberry Pi (with RPi.GPIO installed):
    python3 pan_range_test.py --offset 33 --invert 1

Common options:
    --offset       World az zero-offset in degrees (e.g., 33 for a window of ~87..327 when invert=1)
    --invert       1 to mirror azimuth (reverse direction), 0 otherwise
    --azi-pin      BCM pin for azimuth servo (default 18)
    --ele-pin      BCM pin for elevation servo (default 13); we keep EL fixed
    --gear         External gear ratio physical_out:servo (default 2.0)
    --servo-max    Servo mechanical max degrees (default 120.0)
    --duty-min     PWM duty % at servo=0° (default 5.0)
    --duty-max     PWM duty % at servo=servo-max (default 10.0)
    --step         WORLD step in degrees (default 12.0 → ~6° servo step with 2:1 gear)
    --dwell        Seconds to hold each position (default 0.6)
    --elevation    WORLD elevation to hold during test (default 0.0)
"""

import argparse
import time
import math

try:
    import RPi.GPIO as GPIO
except Exception as e:
    raise SystemExit("ERROR: RPi.GPIO not available. Run this on a Raspberry Pi. Details: %s" % e)

# -------------------- CLI --------------------
p = argparse.ArgumentParser()
p.add_argument("--offset", type=float, default=30.0, help="World az zero-offset (deg).")
p.add_argument("--invert", type=int, default=1, help="Invert azimuth mapping? 1=yes, 0=no.")
p.add_argument("--azi-pin", type=int, default=18, help="BCM pin for azimuth servo PWM.")
p.add_argument("--ele-pin", type=int, default=13, help="BCM pin for elevation servo PWM (held fixed).")
p.add_argument("--gear", type=float, default=2.0, help="External gear ratio physical_out:servo.")
p.add_argument("--servo-max", type=float, default=120.0, help="Servo max angle (deg).")
p.add_argument("--duty-min", type=float, default=5.0, help="Duty %% at servo=0°.")
p.add_argument("--duty-max", type=float, default=10.0, help="Duty %% at servo=servo-max°.")
p.add_argument("--step", type=float, default=12.0, help="WORLD azimuth step (deg).")
p.add_argument("--dwell", type=float, default=0.6, help="Seconds to hold each position.")
p.add_argument("--elevation", type=float, default=0.0, help="WORLD elevation to hold.")
args = p.parse_args()

AZIMUTH_ZERO_OFFSET_DEG = args.offset
AZIMUTH_INVERT = bool(args.invert)
SERVO_MAX_DEG = float(args.servo_max)
GEAR_RATIO = float(args.gear)
DUTY_MIN = float(args.duty_min)
DUTY_MAX = float(args.duty_max)
STEP_WORLD_DEG = float(args.step)
DWELL = float(args.dwell)
ELEVATION_WORLD = float(args.elevation)

# Derived physical window (0..PHYS_MAX)
PHYS_MIN = 0.0
PHYS_MAX = SERVO_MAX_DEG * GEAR_RATIO  # e.g., 120 * 2 = 240°

# -------------------- Mapping helpers (match geo_4.py logic) --------------------
def norm360(x: float) -> float:
    return (x + 360.0) % 360.0

def world_window(invert: bool, offset_deg: float):
    """
    Compute the WORLD azimuth window [start, end] (deg) that maps into 0..PHYS_MAX
    given the invert flag and zero-offset.
    - If invert=False:  window = [-offset, 240 - offset] (mod 360)
    - If invert=True:   window = [120 - offset, 360 - offset] (mod 360)
    """
    if invert:
        start = norm360(120.0 - offset_deg)
        end   = norm360(360.0 - offset_deg)
    else:
        start = norm360(-offset_deg)
        end   = norm360(240.0 - offset_deg)
    return start, end  # span is 240° modulo 360

def apply_calibration_world_to_physical(az_world: float, el_world: float):
    """
    World -> calibrated world (offset/invert) -> PHYSICAL (post-gear) with folding/clamp.
    Matches the behavior we used in geo_4.py so results are comparable.
    """
    az_w = norm360(az_world + AZIMUTH_ZERO_OFFSET_DEG)
    el_w = el_world  # elevation held fixed; we don't use its mapping in this pan test

    if AZIMUTH_INVERT:
        az_w = norm360(360.0 - az_w)

    az_phys = az_w
    # Try to fold into [PHYS_MIN, PHYS_MAX] by subtracting 360 once if helpful
    if az_phys > PHYS_MAX:
        cand = az_phys - 360.0
        if PHYS_MIN <= cand <= PHYS_MAX:
            az_phys = cand

    # Clamp if still out of range (shouldn't happen when we sweep inside the window)
    if az_phys < PHYS_MIN:
        az_phys = PHYS_MIN
    if az_phys > PHYS_MAX:
        az_phys = PHYS_MAX

    return az_phys, el_w

def phys_to_servo(az_phys: float):
    """PHYSICAL -> SERVO (deg)."""
    az_servo = max(0.0, min(SERVO_MAX_DEG, az_phys / GEAR_RATIO))
    return az_servo

def servo_to_duty(servo_deg: float):
    """Linear map: 0..SERVO_MAX_DEG  ->  DUTY_MIN..DUTY_MAX (%)"""
    if servo_deg < 0.0: servo_deg = 0.0
    if servo_deg > SERVO_MAX_DEG: servo_deg = SERVO_MAX_DEG
    span = SERVO_MAX_DEG if SERVO_MAX_DEG != 0 else 1.0
    duty = DUTY_MIN + (servo_deg / span) * (DUTY_MAX - DUTY_MIN)
    return duty

def generate_world_sweep(start: float, end: float, step_deg: float):
    """Return a list of WORLD az values from 'start' to 'end' (inclusive), handling wrap-around."""
    seq = []
    if start <= end:
        a = start
        while a <= end + 1e-6:
            seq.append(a)
            a += step_deg
        if seq[-1] < end - 1e-6:
            seq.append(end)
    else:
        a = start
        while a < 360.0 + 1e-6:
            seq.append(a)
            a += step_deg
        a = 0.0
        while a <= end + 1e-6:
            seq.append(a)
            a += step_deg
        if seq[-1] < end - 1e-6:
            seq.append(end)
    return seq

# -------------------- GPIO / PWM --------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

AZI_PIN = int(args.azi_pin)
ELE_PIN = int(args.ele_pin)

GPIO.setup(AZI_PIN, GPIO.OUT)
GPIO.setup(ELE_PIN, GPIO.OUT)

pwm_azi = GPIO.PWM(AZI_PIN, 50)  # 50 Hz
pwm_ele = GPIO.PWM(ELE_PIN, 50)  # we keep EL fixed but initialize for completeness

pwm_azi.start(0)
pwm_ele.start(0)

def drive_az_servo_to_world(az_world: float, el_world: float):
    """Compute mapping and drive ONLY the AZ servo for the given WORLD angles."""
    az_phys, _ = apply_calibration_world_to_physical(az_world, el_world)
    az_servo = phys_to_servo(az_phys)
    duty = servo_to_duty(az_servo)
    print(f"[CMD] world_az={az_world:.2f}° -> phys_az={az_phys:.2f}° -> servo_az={az_servo:.2f}° -> duty={duty:.2f}%")
    pwm_azi.ChangeDutyCycle(duty)
    # EL is left unchanged (held at 0 duty). If you want to hold a non-zero elevation,
    # you can compute and command it similarly.

def stop_pwm():
    pwm_azi.ChangeDutyCycle(0)
    pwm_ele.ChangeDutyCycle(0)

# -------------------- Test --------------------
def pan_range_test(step_world_deg=STEP_WORLD_DEG, dwell=DWELL, elevation_world_deg=ELEVATION_WORLD):
    start, end = world_window(AZIMUTH_INVERT, AZIMUTH_ZERO_OFFSET_DEG)
    print("\n=== PAN RANGE TEST ===")
    print(f"  Invert={AZIMUTH_INVERT}, Offset={AZIMUTH_ZERO_OFFSET_DEG}°")
    print(f"  WORLD window: {start:.1f}° → {end:.1f}° (span 240°)")
    print(f"  Gear={GEAR_RATIO}:1, ServoMax={SERVO_MAX_DEG}°, Duty={DUTY_MIN:.2f}%..{DUTY_MAX:.2f}%")
    print(f"  Step={step_world_deg}°, Dwell={dwell}s, Elevation={elevation_world_deg}°")
    print("====================================\n")

    seq_fwd = generate_world_sweep(start, end, step_world_deg)
    seq_bwd = list(reversed(seq_fwd))

    az_phys_min = None
    az_phys_max = None

    # Move to start
    drive_az_servo_to_world(seq_fwd[0], elevation_world_deg)
    time.sleep(dwell)
    stop_pwm()

    # Forward sweep
    for azw in seq_fwd:
        drive_az_servo_to_world(azw, elevation_world_deg)
        azp, _ = apply_calibration_world_to_physical(azw, elevation_world_deg)
        az_phys_min = azp if az_phys_min is None else min(az_phys_min, azp)
        az_phys_max = azp if az_phys_max is None else max(az_phys_max, azp)
        time.sleep(dwell)
        stop_pwm()

    # Backward sweep (optional; exercises reverse direction)
    for azw in seq_bwd:
        drive_az_servo_to_world(azw, elevation_world_deg)
        azp, _ = apply_calibration_world_to_physical(azw, elevation_world_deg)
        az_phys_min = min(az_phys_min, azp)
        az_phys_max = max(az_phys_max, azp)
        time.sleep(dwell)
        stop_pwm()

    total_span = (az_phys_max - az_phys_min) if (az_phys_min is not None) else 0.0
    print("\n[RESULT] Physical AZ min={:.2f}°, max={:.2f}°, span={:.2f}°".format(
        az_phys_min or 0.0, az_phys_max or 0.0, total_span))
    print("=========== TEST COMPLETE ===========\n")

# -------------------- Main --------------------
try:
    pan_range_test()
except KeyboardInterrupt:
    print("\nInterrupted by user.")
finally:
    try:
        stop_pwm()
        time.sleep(0.2)
        pwm_azi.stop()
        pwm_ele.stop()
        GPIO.cleanup()
    except Exception:
        pass
