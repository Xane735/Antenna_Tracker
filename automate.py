from pymavlink import mavutil
import math
import time

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

def loiter(mav):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                                     176, 0, 1, 5, 0, 0, 0, 0, 0)
    msg = mav.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    print("Set to Loiter Mode")

def auto(mav):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                                     176, 0, 1, 3, 0, 0, 0, 0, 0)

    msg = mav.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    print("Set to Auto Mode")

def arm(mav):
    # Arm the drone
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                                         400, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = mav.recv_match(type='COMMAND_ACK', blocking=True)
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

    test_msg = mav.recv_match(type='MISSION_CURRENT', blocking=True)
    print(test_msg)
    print(start_msg)
    print("Auto mission started!")

print("Starting Mission")
loiter(mav)
arm(mav)
auto(mav)
start_mission(mav)
