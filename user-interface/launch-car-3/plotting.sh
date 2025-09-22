#!/usr/bin/env bash
echo "Launching real-time vehicle data interface for Car 1..."

source /opt/ros/noetic/setup.bash

rosrun plotjuggler plotjuggler &

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.119:11311
export ROS_IP=192.168.50.119

echo "Plotting interface launched for Car 1"