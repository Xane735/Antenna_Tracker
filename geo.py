from pymavlink import mavutil
import math
import time
import azi_elev

lat_gcs = 13.026971 # Set GCS Latitude
lon_gcs = 77.563056 # Set GCS Longitude

# Connect as a client (listen for broadcast packets)
# Use 'tcp:localhost:5762' in the connection string to connect to SITL

mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')

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

def loiter(mav):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                                     176, 0, 1, 5, 0, 0, 0, 0, 0)
    msg = mav.recv_match(type='COMMAND_ACK', blocking=False)
    print(msg)
    print("Set to Loiter Mode")

def auto(mav):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                                     176, 0, 1, 3, 0, 0, 0, 0, 0)

    msg = mav.recv_match(type='COMMAND_ACK', blocking=False)
    print(msg)
    print("Set to Auto Mode")

def arm(mav):
    # Arm the drone
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                                         400, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = mav.recv_match(type='COMMAND_ACK', blocking=False)
    print(msg)

def start_mission(mav):
    # Start the mission by sending MAV_CMD_MISSION_START
    start_msg = mav.mav.command_long_send(
                            0, 0,
                            300,
                            0,  # Confirmation
                            0, 0, 0, 0, 0, 0, 0  # Parameters for the command (not used in this case)
                    )       

    # Send the start mission command
    mav.mav.send(start_msg)

    test_msg = mav.recv_match(type='MISSION_CURRENT', blocking=False)
    print(test_msg)
    print(start_msg)
    print("Auto mission started!")


def lat_lon_alt_uav(msg):

    # Extract altitude from the message (in millimetres, converting it to metres)
    alt = (msg.alt)*1000

    # Extract latitude and longitude (in 1E7 format, so divide by 10^7 to get decimal degrees)
    lat = (msg.lat) / 10000000
    lon = (msg.lon) / 10000000

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
    print(lat, lon, alt)

def main(mav):
    #print("Initializing Mission")
    #loiter(mav)
    #time.sleep(3)
    #arm(mav)
    #time.sleep(3)
    #auto(mav)
    #time.sleep(3)
    #start_mission(mav)
    #time.sleep(3)
    GPS_stream(mav)

def GPS_stream():

    while True:
        msg = mav.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg: 
            print(f"Lat: {msg.lat/1e7}, Lon: {msg.lon/1e7}, Alt: {msg.alt/1000} m, Satellites: {msg.satellites_visible}")
            azi_elev.calculate_azimuth(lat_gcs, lon_gcs, msg.lat, msg.lon)
            azi_elev.calculate_elevation(lat_gcs, lon_gcs, msg.lat, msg.lon, msg.alt)

main(mav)
