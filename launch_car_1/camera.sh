#!/usr/bin/env bash

# need to fix once camera working
echo "Establishing Camera connection to Car 1..."
sshpass -p 'node$admin' ssh jetson-nano@192.168.50.161 \
'bash -lc "cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch --screen /home/jetson-nano/catkin_ws/src/rslidar_sdk/launch/start.launch"'