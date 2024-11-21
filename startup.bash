#!/bin/bash

echo "Running startup script"

# Source ROS2 workspace and run another script in the same subprocess
source /opt/ros/rolling/setup.bash

# source ~/camera_ws/install/setup.bash
# ros2 run camera_ros camera_node --ros-args -p camera:=0 -p format:='YUYV' -p width:=1920 -p height:=1080

# source ./imu_ws/install/setup.bash 
# ros2 launch tm_imu imu.launch.py

source ./i2c_pwm_board/install/setup.bash
ros2 run i2c_pwm_board node