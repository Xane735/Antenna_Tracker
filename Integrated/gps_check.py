# To be tested lmao
import threading
import time
from pymavlink import mavutil
import sys

# Global lists for ports and baud rates
Port1 = "/dev/ttyACM0"
Port2 = "/dev/ttyUSB0"

baud_rate = 57600

def acknowledge(master, timeout_sec, port, pixhawk):
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout_sec)
    if ack is None or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"[ERROR] [{pixhawk}] Failed to set message interval for GPS_RAW_INT on {port}. Exiting thread.\n")
    return


def gps_stream(port, baud_rate, pixhawk):
    msg_id = 24  # Message ID for GPS_RAW_INT
    rate_hz = 5
    timeout_sec = 1.0

    try:
        print(f"[INFO] [{pixhawk}] Attempting connection on {port} at {baud_rate} baud...")
        master = mavutil.mavlink_connection(port, baud=baud_rate)
        master.wait_heartbeat(timeout=timeout_sec)
        print(f"[INFO] [{pixhawk}]Heartbeat received from system {master.target_system}, component {master.target_component} on {port}")

        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, int(1e6 / rate_hz), 0, 0, 0, 0, 0)
        acknowledge(master, timeout_sec, port, pixhawk)
        print(f"[INFO] [{pixhawk}]Requested GPS_RAW_INT at {rate_hz} Hz on {port}")
        print(f"[INFO] [{pixhawk}]Listening for GPS data on {port}...\n")

        t0 = time.time()
        Sucessful_connection = False
        count = 0
        while time.time() - t0 < 10:  # Run for 10 seconds
            msg = master.recv_match(type='GPS_RAW_INT', blocking = True, timeout = timeout_sec)
            
            if msg is None:
                print(f"[WARN] [{pixhawk}] No GPS_RAW_INT message received from {port} in {timeout_sec} seconds. Retrying...")
                return

            elif (msg.lat is None) or (msg.lon is None):
                print(f"[WARN] [{pixhawk}] Invalid GPS data received on {port}. Retrying... in 5 seconds")
                return

            elif (msg.lat == 0) and (msg.lon == 0):
                print(f"[WARN] [{pixhawk}] No valid GPS fix on {port}. Fix type: {msg.fix_type}. Retrying... in 5 seconds")
                time.sleep(5)
                t0 = time.time()
                count += 1
                if count >= 3:
                    print(f"[ERROR] [{pixhawk}] Multiple invalid GPS data received from {port}. Exiting thread.\n")
                    return
                continue

            else:
                print(f"[{port}] [{pixhawk}] GPS_RAW_INT: fix={msg.fix_type}, sats={msg.satellites_visible}, lat={msg.lat / 1e7:.7f}, lon={msg.lon / 1e7:.7f}, alt={msg.alt / 1000:.2f} m")
                Sucessful_connection = True
                break
            
        if Sucessful_connection == True:
            print(f"[INFO] [{pixhawk}] Successful GPS data retrieval from {port}. Entering main Loop...\n")
            master.close()

        else:
            print(f"[ERROR] [{pixhawk}] Failed to retrieve valid GPS data from {port} after multiple attempts. Exiting thread.\n")
            return
        
    except Exception as e:
        print(f"[ERROR] An error occurred for {port}: {e}")
    finally:
        print(f"[INFO] Thread for {port} at {baud_rate} is stopping.")

def gps_check():
    t1 = threading.Thread(target=gps_stream, args=(Port1, baud_rate, "Base_GPS") , daemon=True)
    t2 = threading.Thread(target=gps_stream, args=(Port2, baud_rate, "Drone_GPS"), daemon=True)

    t1.start()
    t2.start()
    
    try:
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("[INFO] Shutting down...")
        sys.exit(0)

"""
if __name__ == "__main__":
    gps_check()
   
"""
