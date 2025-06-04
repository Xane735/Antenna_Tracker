from pymavlink import mavutil
import threading

'''
# Connect to Pixhawk #2 via USB
usb = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
usb.wait_heartbeat()
print("Got heartbeat from Pixhawk #2")

# Connect to Pixhawk #1 via telemetry
telemetry = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
telemetry.wait_heartbeat()
print("Got heartbeat from Pixhawk #1")


while True:
    msg1 = telemetry.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    msg2 = usb.recv_match(type='GLOBAL_POSITION_INT', blocking=False)

    if msg1:
        print(f"[Telem GPS] Lat: {msg1.lat/1e7:.7f}, Lon: {msg1.lon/1e7:.7f}, Alt: {msg1.alt/1000:.2f} m")

    if msg2:
        print(f"[USB GPS]   Lat: {msg2.lat/1e7:.7f}, Lon: {msg2.lon/1e7:.7f}, Alt: {msg2.alt/1000:.2f} m")
'''

def read_gps(name, port, baud):
    print(f"[{name}] Connecting on {port} at {baud}...")
    mav = mavutil.mavlink_connection(port, baud=baud)
    
    print(f"[{name}] Waiting for heartbeat...")
    mav.wait_heartbeat(timeout=10)
    print(f"[{name}] Heartbeat received!")

    while True:
        msg = mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1e3
            print(f"[{name}] Lat: {lat}, Lon: {lon}, Alt: {alt} m")
        else:
            print(f"[{name}] No GPS data received.")

# Configure ports
gps_sources = [
    ("USB_Pixhawk", "/dev/ttyACM0", 57600),     # or /dev/ttyAMA0 or /dev/serial0
    ("Telemetry", "/dev/ttyUSB0", 57600)         # telemetry port
]

# Start each GPS reader in its own thread
threads = []
for name, port, baud in gps_sources:
    t = threading.Thread(target=read_gps, args=(name, port, baud), daemon=True)
    t.start()
    threads.append(t)

# Keep the main thread alive
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Exiting...")
