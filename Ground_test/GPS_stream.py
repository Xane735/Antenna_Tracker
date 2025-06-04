from pymavlink import mavutil

# Connect to Pixhawk #2 via USB
usb = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
#usb = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
usb.wait_heartbeat()
print("Got heartbeat from Pixhawk #2")

# Connect to Pixhawk #1 via telemetry
telemetry = mavutil.mavlink_connection('/dev/serial0', baud=57600)
telemetry.wait_heartbeat()
print("Got heartbeat from Pixhawk #1")


while True:
    msg1 = telemetry.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    msg2 = usb.recv_match(type='GLOBAL_POSITION_INT', blocking=False)

    if msg1:
        print(f"[Telem GPS] Lat: {msg1.lat/1e7:.7f}, Lon: {msg1.lon/1e7:.7f}, Alt: {msg1.alt/1000:.2f} m")

    if msg2:
        print(f"[USB GPS]   Lat: {msg2.lat/1e7:.7f}, Lon: {msg2.lon/1e7:.7f}, Alt: {msg2.alt/1000:.2f} m")
