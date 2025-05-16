Instructions to Install Mavproxy onto Raspian OS:

    sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
    python3 -m pip install PyYAML mavproxy --user
    echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

If you get a “permission denied” error message when connecting to serial devices, the user permissions may need to be changed:

    sudo usermod -a -G dialout <username>

Install python virtual environment:
sudo apt-get install python3-venv

Create a virtual environment (name it anything like mavproxy-env):
python3 -m venv mavproxy-env

Activate virtual environment:
source mavproxy-env/bin/activate

You should see your terminal change:
(mavproxy-env) user@your-pi:~$

Now install the packages:
pip install PyYAML mavproxy

Run mavproxy from inside the environment:
mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600

To exit the virtual environment:
deactivate

Enable access to global packaged (if required):
python3 -m venv --system-site-packages mavproxy-env

To find all ports connected:
ls /dev/ttyUSB\*
Auto detect port:
dmesg | grep tty

To update an existing installation with the current release:

    python3 -m pip install mavproxy pymavlink --user --upgrade

To detect the Port connected to the pixhawk:
mavproxy.py --master=/dev/ttyUSB0

To set baudrate of the port communication:
mavproxy.py --master=/dev/ttyUSB0

-----------------------------> GPS STREAM SET UP <-------------------------------------

1. Run SITL/ connect to the pixhawk on your GCS:

2. Hit Ctrl+F --> Select Mavlink --> Under First drop down select UDP client --> Input the IP address of the device and the port (14550) --> Hit connect

3. On raspi run:
   mavproxy.py --master=udp:0.0.0.0:14550 --out=udp:192.168.1.193:14551

4. View GPS coordinates:
   status
   You will see messages like:
   GPS: GPS lock: 3D fix
   GPS HDOP: 1.2 sats: 10
   Lat: 47.397742 Lon: 8.545594 Alt: 556.5
   For Raw output of the GPS:
   module load message
   watch GPS_RAW_INT

5. Open another new terminal (Make sure to activate the environment). Run the python script.

------------------------------> PID Tunning <------------------------------------------------

1. Tune Kp:
    Increase Kp gradually (e.g., 0.5 → 1.0 → 2.0).
    Watch the antenna's movement: it should become more responsive.
    Stop increasing if it starts oscillating.
2. Add Damping with Kd:
    Add a small Kd (e.g., 0.1 → 0.2).
    This smooths out fast changes and reduces overshoot.
    If it becomes too slow again, reduce Kd.
    Fix small long-term errors with Ki:
3. Add a small Ki (e.g., 0.01).
    Only increase if your tracker settles slightly off-target and stays there.
    Too much Ki causes instability over time.
