#!/usr/bin/env bash

echo "Establishing SSH connection to Car 1..."
sshpass -p 'node$admin' ssh jetson-nano@192.168.50.161

roslaunch vehicle_steering_velocity_ctr vehicle vehicle_manual_run.launch
roslaunch rslidar_