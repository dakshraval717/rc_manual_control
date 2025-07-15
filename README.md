# manual_control_rc

To setup the rc car: 
1. install the rslidar_sdk and setup the ip address: 192.168.1.102:255.255.255.0
2. git clone the vehicle_steering_velocity_ctr
3. upload the arduino code into ESP32 (it's located in the include directory)
4. install rosserial_python: sudo apt install ros-noetic-rosserial-python
5. install joy package: sudo apt install ros-noetic-joy
6. setup permission for the usb: sudo usermod -a -G dialout $USER
7. log out and log back in (reboot)
8. laucnh the steering vehicle 

Enjoy your ride!
