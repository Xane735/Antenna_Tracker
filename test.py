# Compute original azimuth and elevation
azimuth = azi_elev_3.calculate_azimuth(GCS_LAT, GCS_LON, lat, lon)
elevation = azi_elev_3.calculate_elevation(GCS_LAT, GCS_LON, lat, lon, GCS_ALT, alt)

# Backtracking logic if drone is behind tracker
if azimuth > 180:
    azimuth = azimuth - 180          # Flip azimuth to track from behind
    elevation = 180 - elevation      # Invert elevation angle

# Convert elevation because 0° = down
servo_elevation_angle = 180 - elevation

# Clamp both angles
azimuth = max(0, min(180, azimuth))
servo_elevation_angle = max(0, min(180, servo_elevation_angle))

# Now you can move the servo using the corrected angles
move_antenna_stepwise(azimuth, servo_elevation_angle, step_size=3.0, delay=0.05)


def move_antenna_stepwise(target_az, target_el, step_size=1.0, delay=0.05):
    global servo_azimuth_angle, servo_elevation_angle

    # Normalize azimuth to [0, 360)
    target_az = target_az % 360

    # If target azimuth is behind the antenna (>180°), flip angles for servo limits
    if target_az > 180:
        target_az = target_az - 180
        target_el = 180 - target_el  # invert elevation

    # Clamp final azimuth and elevation to servo range (0-180)
    target_az = max(0, min(180, target_az))
    target_el = max(0, min(180, target_el))

    while True:
        delta_az = (target_az - servo_azimuth_angle + 540) % 360 - 180
        delta_el = target_el - servo_elevation_angle

        if abs(delta_az) <= step_size and abs(delta_el) <= step_size:
            break

        step_az = step_size * (1 if delta_az > 0 else -1) if abs(delta_az) > step_size else delta_az
        step_el = step_size * (1 if delta_el > 0 else -1) if abs(delta_el) > step_size else delta_el

        move_antenna(step_az, step_el)
        time.sleep(delay)
