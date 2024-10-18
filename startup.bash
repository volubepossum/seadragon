#!/bin/bash

echo "Running startup script"
echo "Hello from startup.sh"

# Source ROS2 workspace and run another script in the same subprocess
source /opt/ros/rolling/setup.bash
source ~/ros_ws/install/setup.bash
ros2 run camera_ros camera_node --ros-args -p camera:=0 -p format:='YUYV' -p width:=1920 -p height:=1080

