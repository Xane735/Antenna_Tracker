import threading
import time
from pymavlink import mavutil
import sys

# Global lists for ports and baud rates
Port1 = "/dev/ttyACM0"
Port2 = "/dev/ttyUSB0"
#Port3 = "/dev/ttyUSB0"
baud_rate = 57600

def gps_stream(port, baud_rate, pixhawk):
    msg_id = 24  # Message ID for GPS_RAW_INT
    rate_hz = 5
    timeout_sec = 5

    try:
        print(f"[INFO] [{pixhawk}] Attempting connection on {port} at {baud_rate}Hz...")
        master = mavutil.mavlink_connection(port, baud=baud_rate)
        master.wait_heartbeat(timeout=timeout_sec)
        print(f"[INFO] [{pixhawk}]Heartbeat received from system {master.target_system}, component {master.target_component} on {port}")
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, int(1e6 / rate_hz), 0, 0, 0, 0, 0)
        print(f"[INFO] [{pixhawk}]Requested GPS_RAW_INT at {rate_hz} Hz on {port}")
        print(f"[INFO] [{pixhawk}]Listening for GPS data on {port}...\n")

        while True:
            msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=timeout_sec)
            
            if msg is None:
                print(f"[WARN] [{pixhawk}] No GPS_RAW_INT message received from {port} in {timeout_sec} seconds. Retrying...")
                continue

            print(f"[{port}] [{pixhawk}] GPS_RAW_INT: fix={msg.fix_type}, sats={msg.satellites_visible}, lat={msg.lat / 1e7:.7f}, lon={msg.lon / 1e7:.7f}, alt={msg.alt / 1000:.2f} m")

    except Exception as e:
        print(f"[ERROR] An error occurred for {port}: {e}")
    finally:
        print(f"[INFO] Thread for {port} at {baud_rate} is stopping.")

def main():
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

if __name__ == "__main__":
    main()