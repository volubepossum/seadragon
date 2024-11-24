#!/bin/bash

echo "Running startup script"

# Source ROS2 workspace and run another script in the same subprocess
source /opt/ros/rolling/setup.bash
source ros2_ws/install/setup.bash

ros2 run teleop teleop_node
