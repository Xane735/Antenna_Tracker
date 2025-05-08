from pymavlink import mavutil

def GPS_stream():
    # Connect to MAVLink
    mav = mavutil.mavlink_connection('udp:0.0.0.0:14550')
    print("Waiting for GPS and attitude messages...")

    while True:
        msg = mav.recv_match(blocking=True)
        
        if not msg:
            continue

        # Handle GPS
        if msg.get_type() == 'GPS_RAW_INT':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            sat = msg.satellites_visible
            lat_lon_alt_uav(msg)  # Your existing function

        # Handle attitude (pitch/yaw)
        elif msg.get_type() == 'ATTITUDE':
            # Pitch, roll, yaw in radians
            pitch = msg.pitch
            yaw = msg.yaw

            # Convert to degrees
            pitch_deg = pitch * (180.0 / 3.14159265)
            yaw_deg = yaw * (180.0 / 3.14159265)

            # Call function to move servos
            move_servos(yaw_deg, pitch_deg)

# Dummy servo movement function (you’ll implement actual PWM logic)
def move_servos(yaw, pitch):
    print(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°")
