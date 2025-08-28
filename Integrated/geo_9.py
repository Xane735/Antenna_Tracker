# Code to be tested. Hopefully is the final code
import argparse
import time,datetime
from typing import Optional
import azi_elev_5 as tracker

import constants as con

import helper_fns 
import GPS as gps
import pigpio_smooth_parking as psp
import mavlink
import csv_logging as csvlog
import base_filters_stabilizer as bfs

# ============= Main =============

def main():
    ap = argparse.ArgumentParser(description="Unified antenna tracker with static/dynamic base modes + smooth parking")
    ap.add_argument("--mode", choices=["sim","ground"], default=con.MODE_DEFAULT,
                    help="Run mode: SITL 'sim' or hardware 'ground' (default: ground)")

    ap.add_argument("--base-mode", choices=["dynamic","static"], default=con.BASE_MODE_DEFAULT,
                    help="Base position mode for 'ground': dynamic (filtered live) or static (auto-lock)")

    ap.add_argument("--static-window-sec", type=float, default=10.0,
                    help="Seconds of stable base needed before locking (static mode)")

    ap.add_argument("--static-sd-thresh-m", type=float, default=0.9,
                    help="Stddev threshold in meters to consider base stable (static mode)")

    ap.add_argument("--dynamic-window-sec", type=float, default=6.0,
                    help="Window for rolling median in dynamic mode")

    ap.add_argument("--alpha", type=float, default=1,
                    help="Exponential smoothing factor for world az/el (0..1)")

    ap.add_argument("--print-period", type=float, default=con.PRINT_PERIOD_S,
                    help="Seconds between console prints")

    ap.add_argument("--update-period", type=float, default=con.UPDATE_PERIOD_S,
                    help="Main loop period seconds (servo update rate)")

    # Parking & overrides
    ap.add_argument("--park-home-az", type=float, default=0.0,
                    help="Home azimuth (deg) for parking (default 0)")
    ap.add_argument("--park-home-el", type=float, default=90.0,
                    help="Home elevation (deg) for parking (default 90)")
    ap.add_argument("--park-face-drone-start", action="store_true",
                    help="On startup, park facing current drone azimuth (EL=home)")
    ap.add_argument("--park-face-drone-exit", action="store_true",
                    help="On shutdown, park facing current drone azimuth (EL=home)")
    ap.add_argument("--park-duration", type=float, default=1.5,
                    help="Seconds for smooth parking at init/cleanup")
    ap.add_argument("--park-rate-hz", type=float, default=60.0,
                    help="Update rate for smooth parking")
    ap.add_argument("--az-gear-ratio", dest="az_gear_ratio", type=float, default=None,
                    help="Override AZ gear ratio (default 2.0)")
    ap.add_argument("--el-gear-ratio", dest="el_gear_ratio", type=float, default=None,
                    help="Override EL gear ratio (default 2.0)")
    ap.add_argument("--servo-min-us", dest="servo_min_us", type=float, default=None,
                    help="Override servo min pulse (us), e.g., 900")
    ap.add_argument("--servo-max-us", dest="servo_max_us", type=float, default=None,
                    help="Override servo max pulse (us), e.g., 1200")

    args = ap.parse_args()

    # Apply optional overrides
    global AZ_GEAR_RATIO, EL_GEAR_RATIO, PULSE_MIN_US, PULSE_MAX_US
    if args.az_gear_ratio is not None:
        AZ_GEAR_RATIO = float(args.az_gear_ratio)
    if args.el_gear_ratio is not None:
        EL_GEAR_RATIO = float(args.el_gear_ratio)
    if args.servo_min_us  is not None:
        PULSE_MIN_US = float(args.servo_min_us)
    if args.servo_max_us  is not None:
        PULSE_MAX_US = float(args.servo_max_us)

    print("=== geo_9 (sim/ground) with base dynamic/static, zero-ref + smoothing + smooth parking ===")
    print(f"[CFG] Mode: {args.mode} | BaseMode: {args.base_mode} | "
          f"Gear AZ {con.AZ_GEAR_RATIO}:1, EL {con.EL_GEAR_RATIO}:1 | "
          f"Servo 180° @ {con.PULSE_MIN_US}-{con.PULSE_MAX_US}us")
    print(f"[CFG] update_period={args.update_period:.2f}s, print_period={args.print_period:.1f}s, alpha={args.alpha}")
    print(f"[CFG] parking: home=({args.park_home_az:.1f}°, {args.park_home_el:.1f}°) "
          f"face_drone(start={args.park_face_drone_start}, exit={args.park_face_drone_exit}) "
          f"duration={args.park_duration:.2f}s @ {args.park_rate_hz:.0f} Hz")

    pi = psp.setup_pigpio()
    csvlog.log_open(prefix="Tracker")

    # Prepare buffers and readers
    drone_buf = gps.GpsBuffer("drone", maxlen=300)
    base_buf  = gps.GpsBuffer("base",  maxlen=300)

    if args.mode == "sim":
        mav_drone = mavlink.connect_mav(con.SIM_DRONE_ENDPOINT, con.SIM_DRONE_BAUD, True)
        mavlink.start_reader(mav_drone, drone_buf, on_raw=csvlog.log_raw if con.LOG_RAW_GPS else None)
        print(f"[CFG] SIM base @ {con.base_static['lat']}, {con.base_static['lon']}, {con.base_static['alt']}m")
    else:
        mav_drone = mavlink.connect_mav(con.DRONE_ENDPOINT, con.DRONE_BAUD, True)
        mav_base  = mavlink.connect_mav(con.BASE_ENDPOINT,  con.BASE_BAUD,  False)
        mavlink.start_reader(mav_drone, drone_buf, on_raw=csvlog.log_raw if con.LOG_RAW_GPS else None)
        mavlink.start_reader(mav_base,  base_buf,  on_raw=csvlog.log_raw if con.LOG_RAW_GPS else None)

    # ===== Zero-reference setup =====
    zero_world_az = None
    zero_world_el = None
    print("[INFO] Point the tracker at the drone and stabilize GPS.")
    print("[INFO] Waiting 10 seconds for zero reference…")
    time.sleep(20.0)

    # We need one snapshot of both drone and base for zeroing
    def get_zero_snapshot():
        t_end = time.time() + 5.0
        while time.time() < t_end:
            d = drone_buf.latest()
            if args.mode == "sim":
                b = gps.GpsSample(time.time(), con.base_static["lat"], con.base_static["lon"], con.base_static["alt"])
            else:
                b = base_buf.latest()
            if d and b:
                return d, b
            time.sleep(0.2)
        return None, None

    d0, b0 = get_zero_snapshot()
    if d0 and b0:
        info0 = tracker.get_tracking_info(b0.lat, b0.lon, b0.alt, d0.lat, d0.lon, d0.alt)
        if info0:
            zero_world_az = info0.get('adjusted_azimuth', info0['azimuth'])
            zero_world_el = info0.get('adjusted_elevation', info0['elevation'])
            print(f"[INFO] Zero ref set: AZ={zero_world_az:.2f}°, EL={zero_world_el:.2f}°")
        else:
            print("[WARN] tracker.get_tracking_info failed for zeroing.")
    else:
        print("[WARN] Could not obtain initial GPS snapshot for zeroing. Proceeding with zero=(0,0).")

    # Smoothly park to initial position
    print("[INFO] Smoothly parking the tracker to initial position…")
    if args.park_face_drone_start and d0 and b0:
        info_init = tracker.get_tracking_info(b0.lat, b0.lon, b0.alt, d0.lat, d0.lon, d0.alt)
        if info_init:
            abs_az0 = info_init.get("adjusted_azimuth", info_init["azimuth"])
            # EL = home (e.g., horizon)
            world_az0, world_el0 = helper_fns.norm360(abs_az0), args.park_home_el
            cal_az0, cal_el0 = helper_fns.apply_calibration(world_az0, world_el0)
            phys_az0, phys_el0 = helper_fns.world_to_physical(cal_az0, cal_el0)
            s_az0, s_el0 = helper_fns.physical_to_servo_deg(phys_az0, phys_el0)
            psp.smooth_park(pi, s_az0, s_el0, duration_s=args.park_duration, rate_hz=args.park_rate_hz)
    else:
        cal_az0, cal_el0 = helper_fns.apply_calibration(args.park_home_az, args.park_home_el)
        phys_az0, phys_el0 = helper_fns.world_to_physical(cal_az0, cal_el0)
        s_az0, s_el0 = helper_fns.physical_to_servo_deg(phys_az0, phys_el0)
        psp.smooth_park(pi, s_az0, s_el0, duration_s=args.park_duration, rate_hz=args.park_rate_hz)

    # ===== Base mode management (GROUND only) =====
    base_state: Optional[bfs.BaseState] = None
    if args.mode == "ground":
        if args.base_mode == "static":
            print(f"[INFO] Static base mode: waiting up to {args.static_window_sec:.1f}s for stabilization (sd<{args.static_sd_thresh_m:.2f}m)…")
            bs = bfs.stabilize_base(base_buf, window_sec=args.static_window_sec, sd_thresh_m=args.static_sd_thresh_m)
            if bs:
                base_state = bs
                print(f"[INFO] Base locked: lat={bs.lat:.7f} lon={bs.lon:.7f} alt={bs.alt:.2f}  (sd≈{bs.sd_lat_m:.2f}/{bs.sd_lon_m:.2f} m)")
            else:
                print("[WARN] Base did not stabilize in time; falling back to dynamic filtered base.")
                args.base_mode = "dynamic"

    # Prepare smoothing variables and print pacing
    smoothed_az = None
    smoothed_el = None
    alpha = max(0.0, min(1.0, args.alpha))
    next_print = time.time()
    last_servo_az = 0.0
    last_servo_el = 0.0

    try:
        while True:
            # snapshot drone
            d = drone_buf.latest()
            if not d:
                time.sleep(0.05); continue

            # compute base according to mode
            if args.mode == "sim":
                b_lat, b_lon, b_alt = con.base_static["lat"], con.base_static["lon"], con.base_static["alt"]
                base_locked = True
                base_sd_lat = base_sd_lon = 0.0
                base_fix = base_sats = None
                base_mode_str = "static(SIM)"
            else:
                if base_state and base_state.locked:
                    b_lat, b_lon, b_alt = base_state.lat, base_state.lon, base_state.alt
                    base_locked = True
                    base_sd_lat, base_sd_lon = base_state.sd_lat_m, base_state.sd_lon_m
                    base_fix, base_sats = base_state.fix_type, base_state.sats
                    base_mode_str = "static"
                else:
                    bs = bfs.dynamic_base_filtered(base_buf, window_sec=args.dynamic_window_sec)
                    if not bs:
                        time.sleep(0.05); continue
                    b_lat, b_lon, b_alt = bs.lat, bs.lon, bs.alt
                    base_locked = False
                    base_sd_lat, base_sd_lon = bs.sd_lat_m, bs.sd_lon_m
                    base_fix, base_sats = bs.fix_type, bs.sats
                    base_mode_str = "dynamic"

            # Compute angles
            info = tracker.get_tracking_info(b_lat, b_lon, b_alt, d.lat, d.lon, d.alt)
            if not info:
                time.sleep(args.update_period); continue

            abs_az = info.get('adjusted_azimuth', info['azimuth'])
            abs_el = info.get('adjusted_elevation', info['elevation'])
            world_az = helper_fns.norm360(abs_az - (zero_world_az or 0.0))
            world_el = abs_el - (zero_world_el or 0.0)

            # Low-pass smoothing of world az/el
            if smoothed_az is None:
                smoothed_az, smoothed_el = world_az, world_el
            else:
                smoothed_az = (1-alpha)*smoothed_az + alpha*world_az
                smoothed_el = (1-alpha)*smoothed_el + alpha*world_el

            # Calibration → clamp → gearing → pulse
            cal_az, cal_el   = helper_fns.apply_calibration(smoothed_az, smoothed_el)
            phys_az, phys_el = helper_fns.world_to_physical(cal_az, cal_el)
            s_az, s_el       = helper_fns.physical_to_servo_deg(phys_az, phys_el)
            us_az            = helper_fns.servo_deg_to_us(s_az)
            us_el            = helper_fns.servo_deg_to_us(s_el)

            # Drive servos
            pi.set_servo_pulsewidth(con.SERVO_AZ_PIN, us_az)
            pi.set_servo_pulsewidth(con.SERVO_EL_PIN, us_el)
            last_servo_az, last_servo_el = s_az, s_el

            # Console output (paced)
            if time.time() >= next_print:
                print(f"[{datetime.datetime.now():%H:%M:%S}] Base={base_mode_str}{'Locked' if base_locked else ''} "
                      f"sd≈{base_sd_lat:.2f}/{base_sd_lon:.2f}m | "
                      f"WORLD {smoothed_az:6.2f}/{smoothed_el:5.2f}° | "
                      f"SERVO {s_az:6.2f}/{s_el:5.2f}° | µs {us_az:5.0f}/{us_el:5.0f}")
                next_print = time.time() + max(0.2, args.print_period)

            # CSV log (padded with diagnostics)
            csvlog.log_row(
                datetime.datetime.now().isoformat(timespec='seconds'),
                base_mode_str, 1 if base_locked else 0,
                round(smoothed_az,3), round(smoothed_el,3),
                round(cal_az,3), round(cal_el,3),
                round(phys_az,3), round(phys_el,3),
                round(s_az,3), round(s_el,3),
                round(us_az,1), round(us_el,1),
                round(d.lat,7), round(d.lon,7), round(d.alt,2),
                round(b_lat,7), round(b_lon,7), round(b_alt,2),
                round(base_sd_lat,3), round(base_sd_lon,3),
                "" if base_fix is None else base_fix,
                "" if base_sats is None else base_sats
            )
            time.sleep(args.update_period)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
    finally:
        print("[INFO] Smooth shutdown: parking…")
        try:
            if args.park_face_drone_exit:
                # Keep current AZ (last_servo_az), ramp EL → home
                cal_azH, cal_elH = helper_fns.apply_calibration(0.0, args.park_home_el)
                phys_azH, phys_elH = helper_fns.world_to_physical(cal_azH, cal_elH)
                s_azH = last_servo_az
                s_elH = helper_fns.physical_to_servo_deg(phys_azH, phys_elH)[1]
                psp.smooth_park(pi, s_azH, s_elH, duration_s=args.park_duration, rate_hz=args.park_rate_hz)
            else:
                cal_azH, cal_elH = helper_fns.apply_calibration(args.park_home_az, args.park_home_el)
                phys_azH, phys_elH = helper_fns.world_to_physical(cal_azH, cal_elH)
                s_azH, s_elH = helper_fns.physical_to_servo_deg(phys_azH, phys_elH)
                psp.smooth_park(pi, s_azH, s_elH, duration_s=args.park_duration, rate_hz=args.park_rate_hz)

            time.sleep(0.2)
            pi.set_servo_pulsewidth(con.SERVO_AZ_PIN, 0)
            pi.set_servo_pulsewidth(con.SERVO_EL_PIN, 0)
            pi.stop()
        except Exception as e:
            print(f"[WARN] pigpio cleanup: {e}")
        csvlog.log_close()

if __name__ == "__main__":
    main()
