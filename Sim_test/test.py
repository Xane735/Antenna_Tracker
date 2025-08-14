def main():
    ap = argparse.ArgumentParser(description = "Antenna Tracker with Static/Dynamic modes and Smooth Parking Algorithm")
    ap.add_argument("--mode", choices=["sim","grnd"], default = MODE_DEFAULT,
                    help ="Run mode: SITL-sim or Field Test-grnd (default: ground mode)")
    ap.add_argument("--base-mode", choices=["dynamic","static"], default = BASE_MODE_DEFAULT,
                    help = "Base Position mode for 'Ground Mode: dynamic or static)")
    ap.add_argument("--static-window", type=float, default = 10.0,
                    help ="Seconds of Base GPS hold needed before locking (static mode)")
    ap.add_argument("--alpha", type = float, default =0.2,
                    help="Exponential smoothing factor")
    ap.add_argument("--console-print", type=float, default = PRINT_PERIOD_S,
                    help = "Seconds between console logs on terminal")
    ap.add_argument("--update-period", type=float, default = UPDATE_PERIOD_S,
                    help = "Main loop period seconds (servo update rate)")
    ap.add_argument("--park-home-azi", type=float, default = 0.0,
                    help="Home azimuth (deg) for parking (default= 0.0 deg)")
    ap.add_argument("--park-home-elv", type=float, default = 90.0,
                    help="Home elevation (deg) for parking (default = 90.0 deg)")
    ap.add_argument("--park-duration", type=float, default =1.5,
                    help="Seconds for smooth parking at initialization/cleanup")
    ap.add_argument("--servo-min-us", dest="servo_min_us", type=float , default = 900,
                    help = "Override servo minimum pulse (us), (default = 900)")
    ap.add_argument("--servo-max-us", dest="servo_max_us", type =float, default = 2100,
                    help = "Override servo maximum pulse (us), (default = 2100)")
    
    args = ap.parse_args()

    global PULSE_MAX_US , PULSE_MIN_US
    if args.servo_max_us is not 2100:
        PULSE_MAX_US = float(arg.servo_max_us)
    if args.servo_min_us is not 900:
        PULSE_MIN_US = float(arg.servo_min_us)
    
    print("Starting Antenna Tracker....")
    print(f"[CFG] Mode: {args.mode} BaseMode: {args.base_mode} Pulse Range: {PULSE_MIN_US}-{PULSE_MAX_US} us")
    print(f"[CFG] Parking: home=({args.park_home_az},{args.park_home_el} deg)")
    print(f"[CFG] Parking duration {args.park_duration}")

    pi = setup_pigpio()
    log_open(prefix = "Tracker")

    drone_buf = GpsBuffer("drone", maxlen = 300)
    base_buf  = GpsBuffer("base",  maxlen = 300)

    if args.mode == "sim":
        mav_drone = connect_mav(SIM_DRONE_ENDPOINT, SIM_DRONE_BAUD, True)
        start_reader(mav_drone, drone_buf, on_raw = log_raw if LOG_RAW_GPS else None)
        print(f"[CFG] SIM base {base_static['lat']}, {base_static['lon']}, {base_static['alt']}m")
    else:
        mav_drone = connect_mav(DRONE_ENDPOINT, DRONE_BAUD, True)
        mav_base  = connect_mav(BASE_ENDPOINT,  BASE_BAUD,  False)
        start_reader(mav_drone, drone_buf, on_raw=log_raw if LOG_RAW_GPS else None)
        start_reader(mav_base,  base_buf,  on_raw=log_raw if LOG_RAW_GPS else None)
    
    zero_world_az = None
    zero_world_el = None
    print("[INFO] Point the tracker at the drone...")
    time.sleep(5.0)
    print("[INFO] Waiting 5 seconds for zero reference...")
    time.sleep(5.0)

    def get_zero_snapshot():
        t_end = time.time() + 5.0
        while time.time() < t_end:
            d = drone_buf.latest()  # Instance of Drones latest GPS stream from the buffer
            if args.mode == "sim":
                b = GpsSample(time.time(), base_static["lat"], base_static["lon"], base_static["alt"])
            else:
                b = base_buf.latest()  # Instance of Bases latest GPS stream from the buffer
            if d and b:
                return d, b
            time.sleep(0.2)
        return None, None
    
    drone_0, base_0 = get_zero_snapshot()
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
