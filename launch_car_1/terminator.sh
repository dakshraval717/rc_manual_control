echo "Launching Car 1 Control Interface..."

# Launch each script in separate terminator windows
terminator --new-tab --title="LiDAR" --command="bash -c 'bash ~/catkin_ws/src/real-time-ui/launch-car-1/lidar.sh; exec bash'" &
sleep 1
terminator --new-tab --title="Manual_Control" --command="bash -c 'bash ~/catkin_ws/src/real-time-ui/launch-car-1/manual-ctr.sh; exec bash'" &
sleep 1  
terminator --new-tab --title="Camera" --command="bash -c '/usr/local/zed/tools/ZED_Explorer; exec bash'" &
sleep 1
terminator --new-tab --title="Plotting" --command="bash -c 'bash ~/catkin_ws/src/real-time-ui/launch-car-1/plotting.sh; exec bash'" &
sleep 1
terminator --new-tab --title="RViz" --command="bash -c 'rviz -d ~/catkin_ws/src/real-time-ui/launch-car-1/rviz-config.rviz '"

echo "All terminals launched!"
