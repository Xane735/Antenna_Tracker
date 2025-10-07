#Super-simple manual calibration:
#      - Controls:  a/d = AZ -/+,  w/s = EL +/-
#      - 'm' to commit zero so *current physical pose* becomes (desired_world_az, desired_world_el)
#      - 'q' to quit (KeyboardInterrupt)
#    Moves servos as you nudge; returns (curr_phys_az, curr_phys_el).
#    Sets AZIMUTH_ZERO_OFFSET_RT / ELEVATION_ZERO_OFFSET_RT globals.

from pynput import keyboard
from pynput.keyboard import Key

def calibrate_simple(
    pi,
    curr_phys_az: float,
    curr_phys_el: float,
    step_deg: float = 5.0,
    desired_world_az: float = 0.0,
    desired_world_el: float = 0.0
):
    
    global AZIMUTH_ZERO_OFFSET_RT, ELEVATION_ZERO_OFFSET_RT

    print("\n[CAL] Manual calibration")
    print(f"[CAL] Controls: a/d=AZ±{step_deg}°, w/s=EL±{step_deg}°, m=commit zero, q=quit")
    print(f"[CAL] Target world zero => az={desired_world_az:.1f}°, el={desired_world_el:.1f}°\n")

    def _drive_now():
        s_az, s_el = physical_to_servo_deg(curr_phys_az, curr_phys_el)
        us_az      = servo_deg_to_us_az(s_az)
        us_el      = servo_deg_to_us(s_el)
        pi.set_servo_pulsewidth(SERVO_AZ_PIN, us_az)
        pi.set_servo_pulsewidth(SERVO_EL_PIN, us_el)
        print(f"[CAL] PHYS az={curr_phys_az:6.2f}° el={curr_phys_el:5.2f}° | SERVO az={s_az:6.2f}° el={s_el:5.2f}°")

    # show current pose on entry
    _drive_now()

    while True:
        cmd = input("[CAL] (a/d/w/s, m=commit, q=quit) > ").strip().lower()
        if (cmd == 'a') or (cmd == Key.left):
            curr_phys_az = wrap360(curr_phys_az - step_deg)
            _drive_now()
        elif (cmd == 'd') or (cmd == Key.right):
            curr_phys_az = wrap360(curr_phys_az + step_deg)
            _drive_now()
        elif (cmd == 'w') or (cmd == Key.up):
            curr_phys_el = min(EL_PHYS_MAX, curr_phys_el + step_deg)
            _drive_now()
        elif (cmd == 's') or (cmd == Key.down):
            curr_phys_el = max(EL_PHYS_MIN, curr_phys_el - step_deg)
            _drive_now()
        elif cmd == 'm':
            # Compute runtime zero so that (desired_world_az/el) maps to current *physical* pose
            # Handle inversion flags the same way your main pipeline does.
            if 'AZIMUTH_INVERT' in globals() and AZIMUTH_INVERT:
                # curr_phys = 360 - (desired + offset)  (wrapped)
                t = wrap360(360.0 - curr_phys_az)
                AZIMUTH_ZERO_OFFSET_RT = wrap360(t - desired_world_az)
            else:
                AZIMUTH_ZERO_OFFSET_RT = wrap360(curr_phys_az - desired_world_az)

            if 'ELEVATION_INVERT' in globals() and ELEVATION_INVERT:
                # curr_phys = -(desired + offset)
                ELEVATION_ZERO_OFFSET_RT = -curr_phys_el - desired_world_el
            else:
                ELEVATION_ZERO_OFFSET_RT = curr_phys_el - desired_world_el

            print(f"[CAL] Runtime zero set: AZ_OFF={AZIMUTH_ZERO_OFFSET_RT:.2f}°, "
                  f"EL_OFF={ELEVATION_ZERO_OFFSET_RT:.2f}°")
            return curr_phys_az, curr_phys_el

        elif cmd == 'q':
            raise KeyboardInterrupt

        elif cmd:
            print("[CAL] Unknown key. Use a/d/w/s, 'm' to commit, 'q' to quit.")
