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
    ls /dev/ttyUSB*
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
    sim_vehicle.py -v ArduCopter -f quad --console --map --out=udp:<raspberry_pi_ip>:14550

2. Run mavproxy on raspi:
    mavproxy.py --master=udp:0.0.0.0:14550

3. View GPS coordinates:
    status
You will see messages like:
    GPS: GPS lock: 3D fix
    GPS HDOP: 1.2  sats: 10
    Lat: 47.397742  Lon: 8.545594  Alt: 556.5
For Raw output of the GPS:
    module load message
    watch GPS_RAW_INT
4. Run your python script to log the GPS Stream


