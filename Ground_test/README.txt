1.  To check for pixhawk's port in raspi, run:
    ls /dev/tty*
And search for /dev/tty/ACM0
2. Run mavproxy server: 
     mavproxy.py --master=/dev/ttyACM0 --baudrate 57600 --out=udp:192.168.1.153:14550
Note: change the port name depending upon what you find in the previous command. (ttyUSB0/ttyACM0)

3. To enable serial ports on raspi run:
    sudo raspi-config
    Interfacing Options >>> serial
    Login Shell over serial ---> No, Enable Serial Hardware ---> Yes
    Reboot the system

Telem will be connected to the Antenna Tracker which will get data. It will use the data and also forward it to the GCS.

3. To send to multiple endpoints:
    --out=udp:<GCS1_IP>:14550 --out=udp:<GCS2_IP>:14550
