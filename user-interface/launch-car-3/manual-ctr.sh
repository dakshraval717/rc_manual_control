#!/usr/bin/env bash

echo "Establishing SSH connection to Car 1..."
sshpass -p 'node$admin' ssh jetson-nano@192.168.50.161 'bash -lc "
cd ~/catkin_ws;
source /opt/ros/noetic/setup.bash;
source ~/catkin_ws/devel/setup.bash;
export ROS_MASTER_URI=http://192.168.50.119:11311;
export ROS_IP=192.168.50.161;
roslaunch --screen /home/jetson-nano/catkin_ws/src/vehicle_steering_velocity_ctr/launch/vehicle_manual_run.launch
"'