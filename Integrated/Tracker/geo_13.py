# geo_13.py — zero-filter tracker with static/dynamic base modes

import argparse
import time
from datetime import datetime

import azi_elev_5 as tracker
import servo_map as servo
import config
import flip_logic as flip
import mavlink_io as mavlink
import gpio_parking as gpio
import logger as log
import utilities as util

# ===================== Main =====================

def main():
    ap = argparse.ArgumentParser(description="Antenna tracker with static/dynamic base, zero-ref + smooth parking ")

    ap.add_argument("--mode", choices=["sim","ground"], default= config.MODE_DEFAULT,help="SITL 'sim' or hardware 'ground'")
    ap.add_argument("--base-mode", choices=["static","dynamic"], default= config.BASE_MODE_DEFAULT,help="Base position: static=freeze after window; dynamic=always latest")
    ap.add_argument("--static-window-sec", type=float, default=10.0,help="Seconds to sample base before freezing (static mode only)")
    ap.add_argument("--print-period", type=float, default= config.PRINT_PERIOD_S,help="Seconds between console prints")
    ap.add_argument("--update-period", type=float, default= config.UPDATE_PERIOD_S,help="Main loop period seconds (servo update rate)")

    args = ap.parse_args()

    # Apply optional overrides
    
    print("=== geo_13 (sim/ground) — static/dynamic base, zero-ref, smooth parking — NO FILTERING ===")
    print(f"[CFG] Mode: {args.mode} | BaseMode: {args.base_mode} | Gear AZ {config.AZ_GEAR_RATIO}:1, EL {config.EL_GEAR_RATIO}:1 | Servo 180 deg @ {config.PULSE_MIN_US}-{config.PULSE_MAX_US}us")
    print(f"[CFG] update_period={args.update_period:.2f}s, print_period={args.print_period:.1f}s")
    print(f"[CFG] parking: home=({config.PARK_HOME_AZ:.1f}, {config.PARK_HOME_EL:.1f}) face_drone(start={config.PARK_FACE_DRONE_START}, exit={config.PARK_FACE_DRONE_EXIT}) duration={config.PARK_DURATION:.2f}s @ {config.PARK_RATE:.0f} Hz")

    pi = gpio.setup()
    log.open(prefix="Tracker")

    # --- Initial parking happens BEFORE MAV readers start ---
    print("[INFO] Parking to home based on fixed zero (no learned zero).")

    # Park to world (0°, home-EL); this is your logical zero
    phys_az0, phys_el0 = util.world_to_physical(config.PARK_HOME_AZ, config.PARK_HOME_EL)
    s_az0, s_el0 = servo.physical_to_servo_deg(phys_az0, phys_el0)
    gpio.smooth_park(pi, s_az0, s_el0, duration_s= config.PARK_DURATION, rate_hz=config.PARK_RATE)
    
    time.sleep(0.1)
    
    last_servo_az = float(s_az0)
    last_servo_el = float(s_el0)

    curr_phys_az = phys_az0
    curr_phys_el = phys_el0

    # Start readers
    if args.mode == "sim":
        mav_drone = mavlink.connect_mav(config.SIM_DRONE_ENDPOINT, config.SIM_DRONE_BAUD, True)
        mavlink.start_reader(mav_drone, "DRONE")
        print(f"[CFG] SIM base @ {config.base_static['lat']}, {config.base_static['lon']}, {config.base_static['alt']}m")
    else:
        mav_drone = mavlink.connect_mav(config.DRONE_ENDPOINT, config.DRONE_BAUD, True)
        mav_base  = mavlink.connect_mav(config.BASE_ENDPOINT,  config.BASE_BAUD,  False)
        mavlink.start_reader(mav_drone, "DRONE")
        mavlink.start_reader(mav_base,  "BASE")

    # ----- Base mode handling (ground only) -----
    base_fixed = None  # (lat, lon, alt) when static
    if args.mode == "ground":
        if args.base_mode == "static":
            print(f"[INFO] Base mode=static → collecting {args.static_window_sec:.1f}s, then freezing to average.")
            t_end = time.time() + float(args.static_window_sec)
            lats, lons, alts = [], [], []
            while time.time() < t_end:
                b_try = mavlink.get_latest_base()
                if b_try:
                    lats.append(b_try.lat); lons.append(b_try.lon); alts.append(b_try.alt)
                time.sleep(0.05)
            if lats:
                base_fixed = (sum(lats)/len(lats), sum(lons)/len(lons), sum(alts)/len(alts))
                print(f"[INFO] Base frozen avg: lat={base_fixed[0]:.7f}, lon={base_fixed[1]:.7f}, alt={base_fixed[2]:.2f}")
            else:
                print("[WARN] No base GPS during static window; will freeze on first base sample in the main loop.")
        else:
            print("[INFO] Base mode=dynamic → always use latest base GPS.")

    # ===== Zero-reference setup =====
    print("[INFO] Point the tracker at 90 degree to the right of the drone and stabilize GPS.")
    time.sleep(4.0)

    next_print = time.time()
    
    try:
        while True:
            # --- DRONE (latest) ---
            d = mavlink.get_latest_drone()
            if not d:
                time.sleep(0.01); continue

            # --- BASE (by mode) ---
            if args.mode == "sim":
                b_lat, b_lon, b_alt = config.base_static["lat"], config.base_static["lon"], config.base_static["alt"]
                base_mode_str = "static(SIM)"
                base_fix = base_sats = None
                base_locked = True
            else:
                if args.base_mode == "static":
                    if base_fixed is None:
                        b_now = mavlink.get_latest_base()
                        if not b_now:
                            time.sleep(0.01); continue
                        base_fixed = (b_now.lat, b_now.lon, b_now.alt)
                        print(f"[INFO] Base frozen late to lat={base_fixed[0]:.7f}, lon={base_fixed[1]:.7f}, alt={base_fixed[2]:.2f}")
                    b_lat, b_lon, b_alt = base_fixed
                    base_mode_str = "static"
                    base_fix = base_sats = None
                    base_locked = True
                else:
                    b = mavlink.get_latest_base()
                    if not b:
                        time.sleep(0.01); continue
                    b_lat, b_lon, b_alt = b.lat, b.lon, b.alt
                    base_mode_str = "dynamic"
                    base_fix = getattr(b, "fix_type", None)
                    base_sats = getattr(b, "sats", None)
                    base_locked = False

            # --- Angles (NO filtering) ---
            info = tracker.get_tracking_info(b_lat, b_lon, b_alt, d.lat, d.lon, d.alt)
            if not info:
                time.sleep(args.update_period); continue

            abs_az = info['azimuth']
            abs_el = info['elevation']
            world_az = abs_az
            world_el = abs_el

            # --- Calibration → pick normal vs backside by servo-distance ---
            cal_az, cal_el = util.apply_calibration(world_az, world_el)
            tgt_phys_az, tgt_phys_el, used_flip = flip.pick_target(cal_az, cal_el, last_servo_az, last_servo_el)

            # --- Per-tick max step (deg/tick) using the current update period ---
            az_step_max = float(config.AZ_MAX_DEG_PER_SEC) * float(args.update_period)
            el_step_max = float(config.EL_MAX_DEG_PER_SEC) * float(args.update_period)

            # --- Shortest-path deltas in PHYSICAL space ---
            d_az = util.shortest_delta_deg(tgt_phys_az, curr_phys_az)   # in (−180, +180]
            d_el = tgt_phys_el - curr_phys_el                      # EL doesn't wrap

            if abs(d_el) < config.EL_DEADBAND_DEG:
                d_el = 0.0
            # --- Clamp step to keep it snappy but safe (prevents 180° wrap jumps) ---
            if d_az >  az_step_max: d_az =  az_step_max
            if d_az < -az_step_max: d_az = -az_step_max
            if d_el >  el_step_max: d_el =  el_step_max
            if d_el < -el_step_max: d_el = -el_step_max

            # --- Advance the physical state; keep az wrapped and el clamped ---
            curr_phys_az = util.wrap360(curr_phys_az + d_az)
            curr_phys_el = max(config.EL_PHYS_MIN, min(config.EL_PHYS_MAX, curr_phys_el + d_el))

            # --- Now map CURRENT physical state → servo → pulse ---
            s_az, s_el = servo.physical_to_servo_deg(curr_phys_az, curr_phys_el)
            us_az      = servo.deg_to_us_az(s_az)
            us_el      = servo.deg_to_us(s_el)

            # --- Drive servos ---
            pi.set_servo_pulsewidth(config.SERVO_AZ_PIN, us_az)
            pi.set_servo_pulsewidth(config.SERVO_EL_PIN, us_el)
            last_servo_az, last_servo_el = s_az, s_el

            # Console output (paced)
            if time.time() >= next_print:
                print(f"[{datetime.now():%H:%M:%S}] Base={base_mode_str} "
                    f"WORLD {world_az:6.2f}/{world_el:5.2f} | "
                    f"PHYS {curr_phys_az:6.2f}/{curr_phys_el:5.2f} | "
                    f"SERVO {s_az:6.2f}/{s_el:5.2f} | us {us_az:5.0f}/{us_el:5.0f}"
                    f"{' | FLIP' if used_flip else ''}")
                next_print += float(args.print_period)

            log.row(
                datetime.now().isoformat(timespec='seconds'),
                base_mode_str, 1 if base_locked else 0,
                round(world_az,3), round(world_el,3),
                round(cal_az,3), round(cal_el,3),
                round(curr_phys_az,3), round(curr_phys_el,3),
                round(s_az,3), round(s_el,3),
                round(us_az,1), round(us_el,1),
                round(d.lat,7), round(d.lon,7), round(d.alt,2),
                round(b_lat,7), round(b_lon,7), round(b_alt,2),
                0.0, 0.0,
                "" if base_fix is None else base_fix,
                "" if base_sats is None else base_sats,
                1 if used_flip else 0    
            )

            time.sleep(args.update_period)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
    finally:
        print("[INFO] Smooth shutdown: parking…")
        try:
            if config.PARK_FACE_DRONE_EXIT:
                # Keep current AZ; ramp EL → home
                cal_azH, cal_elH = util.apply_calibration(0.0, config.PARK_HOME_EL)
                phys_azH, phys_elH = util.world_to_physical(cal_azH, cal_elH)
                s_azH = last_servo_az
                s_elH = servo.physical_to_servo_deg(phys_azH, phys_elH)[1]
                gpio.smooth_park(pi, s_azH, s_elH, duration_s=config.PARK_DURATION, rate_hz= config.PARK_RATE)
            else:
                cal_azH, cal_elH = util.apply_calibration(config.PARK_HOME_AZ, config.PARK_HOME_EL)
                phys_azH, phys_elH = util.world_to_physical(cal_azH, cal_elH)
                s_azH, s_elH = servo.physical_to_servo_deg(phys_azH, phys_elH)
                gpio.smooth_park(pi, s_azH, s_elH, duration_s= config.PARK_DURATION, rate_hz= config.PARK_RATE)

            time.sleep(0.2)
            pi.set_servo_pulsewidth(config.SERVO_AZ_PIN, 0)
            pi.set_servo_pulsewidth(config.SERVO_EL_PIN, 0)
            pi.stop()
        except Exception as e:
            print(f"[WARN] pigpio cleanup: {e}")
        log.close()

if __name__ == "__main__":
    main()
