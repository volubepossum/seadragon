mkfifo camera_pipe # Create a named pipe for the camera

if ! command -v apt &> /dev/null # Check if apt is installed
then
    echo "apt could not be found. Please install apt or edit the script."
    exit 1
fi

if [ "$EUID" -ne 0 ]; then # Check if the script is run as root
    echo "Trying to get root access to install apt packages..."
fi

# Install python3, python3-venv, and ROS Noetic
sudo apt update
sudo apt install -y python3 python3-venv python3-pip
sudo apt install -y ros-jazzy-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup ROS environment
source /opt/ros/jazzy/setup.bash

# Create a Python virtual environment and activate it
python3 -m venv .venv
source .venv/bin/activate

# Install necessary Python packages
pip install numpy opencv-python

# Install rospy and other ROS Python packages
pip install rospy rospkg catkin_pkg

# Ensure ROS environment variables are set in the virtual environment
echo "source /opt/ros/noetic/setup.bash" >> .venv/bin/activate