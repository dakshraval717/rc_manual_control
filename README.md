# manual_control_rc

To setup the rc car: 
1. install the rslidar_sdk and setup the ip address: 192.168.1.102:255.255.255.0
2. git clone the vehicle_steering_velocity_ctr
3. install rosserial_python: sudo apt install ros-noetic-rosserial-python
4. install joy package: sudo apt install ros-noetic-joy
5. setup permission for the usb: sudo usermod -a -G dialout $USER
6. log out and log back in (reboot)
7. laucnh the steering vehicle 

Enjoy your ride!
