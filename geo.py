from pymavlink import mavutil
import math
import time
import automate

# Connect as a client (listen for broadcast packets)
# Use 'tcp:localhost:5762' in the connection string to connect to SITL

mav = mavutil.mavlink_connection('tcp:localhost:5762')

print("Waiting for heartbeat...")
mav.wait_heartbeat()
print("Connected to system (system ID: %d, component ID: %d)" % (mav.target_system, mav.target_component))
mav.mav.command_long_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    500000,  # in microseconds (2Hz)
    0, 0, 0, 0, 0
)


def lat_lon_alt_uav(mav):
    # Wait for a GLOBAL_POSITION_INT message from the drone
    msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    # Extract altitude from the message (in millimetres, converting it to metres)
    alt = (msg.alt)*1000

    # Extract latitude and longitude (in 1E7 format, so divide by 10^7 to get decimal degrees)
    lati = (msg.lat) / 10000000
    long = (msg.lon) / 10000000

    # Calculate intermediate values for radius of curvature (WGS-84 ellipsoid parameters)
    # Earth's equatorial radius (a) = 6378.137 km
    # Earth's polar radius (b) = 6356.753 km

    # Calculate squared terms for radius along latitude
    na = (6378.137 * 6378.137 * math.cos(lati))
    nb = (6356.753 * 6356.753 * math.sin(lati))

    # Calculate corresponding cos and sin terms
    da = (6378.137 * math.cos(lati))
    db = (6356.753 * math.sin(lati))

    # Print the estimated new latitude and adjusted heading
    print(lati, long, alt)

def GPS_stream():

    # Listen on all interfaces on port 14550
    mav = mavutil.mavlink_connection('udp:0.0.0.0:14550')
    print("Waiting for GPS messages...")
    while True:
        msg = mav.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            print(f"Lat: {msg.lat/1e7}, Lon: {msg.lon/1e7}, Alt: {msg.alt/1000} m, Satellites: {msg.satellites_visible}")
            


while True:
    automate.loiter(mav)
    time.sleep(3)
    automate.arm(mav)
    time.sleep(3)
    automate.auto(mav)
    time.sleep(3)
    automate.start_mission(mav)
    time.sleep(3)
    lat_lon_alt_uav(mav)