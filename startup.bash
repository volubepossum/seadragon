#!/bin/bash

echo "Running startup script"

# Source ROS2 workspace and run another script in the same subprocess
source /opt/ros/rolling/setup.bash
source ros2_ws/install/setup.bash

ros2 run motor_controller motor_controller_node &
ros2 run i2c_pwm_board node &

ros2 run teleop teleop_node