from pymavlink import mavutil
import csv
import time

# Connect to MAVProxy over UDP (make sure SITL is sending to this IP/port)
connection = mavutil.mavlink_connection('udp:0.0.0.0:14550')
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print(f"Heartbeat received from system {connection.target_system}")

# Open a CSV file to log GPS data
with open('gps_log.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'Lat', 'Lon', 'Alt'])

    print("Logging GPS data... Press Ctrl+C to stop.")

    try:
        while True:
            msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)

            if msg:
                # Convert from integers to degrees/meters
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0  # millimeters to meters
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

                print(f"{timestamp} - Lat: {lat}, Lon: {lon}, Alt: {alt}")
                writer.writerow([timestamp, lat, lon, alt])
    except KeyboardInterrupt:
        print("Stopped logging.")
