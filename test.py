def track_antenna_to_target(raw_target_az, raw_target_el, step_size=2.0, delay=0.05, THRESH_HOLD=0.5):
    global servo_azimuth_angle, servo_elevation_angle

    # Adjust azimuth and elevation for servo-compatible behavior
    if raw_target_az > 180:
        target_az = raw_target_az - 180
        target_el = 180 - raw_target_el  # Flip elevation to look from behind
    else:
        target_az = raw_target_az
        target_el = raw_target_el

    # Clamp target angles within 0–180 servo range
    target_az = max(0, min(180, target_az))
    target_el = max(0, min(180, target_el))

    # Stepwise movement
    while True:
        # Calculate smallest azimuth difference (in range -180 to +180)
        delta_az = (target_az - servo_azimuth_angle + 540) % 360 - 180
        delta_el = target_el - servo_elevation_angle

        if abs(delta_az) <= step_size and abs(delta_el) <= step_size:
            break

        step_az = step_size * (1 if delta_az > 0 else -1) if abs(delta_az) > step_size else delta_az
        step_el = step_size * (1 if delta_el > 0 else -1) if abs(delta_el) > step_size else delta_el

        # Calculate new servo positions
        new_az = servo_azimuth_angle + step_az
        new_el = servo_elevation_angle + step_el

        # Final clamping
        new_az = max(0, min(180, new_az))
        new_el = max(0, min(180, new_el))

        # Only update if needed
        if abs(new_az - servo_azimuth_angle) > THRESH_HOLD or abs(new_el - servo_elevation_angle) > THRESH_HOLD:
            servo_azimuth_angle = new_az
            servo_elevation_angle = new_el
            Servo.set_angle(servo_azimuth_angle, servo_elevation_angle)
            print(f"Moved to Azimuth: {servo_azimuth_angle:.2f}° | Elevation: {servo_elevation_angle:.2f}°")

        time.sleep(delay)


#Usage:
track_antenna_to_target(raw_target_az=250, raw_target_el=30)
