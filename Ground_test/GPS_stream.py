from pymavlink import mavutil
import threading

# Function to read GPS from Pixhawk
def read_pixhawk_gps():
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
    master.wait_heartbeat()
    print("[Pixhawk] Heartbeat received")

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            print(f"[Pixhawk] Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {alt:.2f} m")

# Function to read GPS from telemetry (another MAVLink stream)
def read_telemetry_gps():
    telem = mavutil.mavlink_connection('udp:0.0.0.0:14551')
    telem.wait_heartbeat()
    print("[Telem] Heartbeat received")

    while True:
        msg = telem.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            print(f"[Telem]   Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {alt:.2f} m")

# Run both readers in parallel
if __name__ == "__main__":
    t1 = threading.Thread(target=read_pixhawk_gps)
    t2 = threading.Thread(target=read_telemetry_gps)

    t1.start()
    t2.start()

    t1.join()
    t2.join()
