#!/bin/bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# mkfifo camera_pipe # Create a named pipe for the camera

if ! command -v apt &> /dev/null # Check if apt is installed
then
    echo "apt could not be found. Please install apt or edit the script."
    exit 1
fi

if [ "$EUID" -ne 0 ]; then # Check if the script is run as root
    echo "Trying to get root access to install apt packages..."
fi

sudo usermod -aG video $USER # Add the user to the video group
sudo usermod -aG audio $USER # Add the user to the audio group

# Install python3, python3-venv, and ROS Noetic
sudo apt update
sudo apt install -y python3 python3-venv python3-pip libi2c-dev i2c-tools


# Create a Python virtual environment and activate it
python3 -m venv .venv
source .venv/bin/activate

# Install necessary Python packages
pip install numpy opencv-python colcon-common-extensions empy==3.3.4 lark



locale  # check for UTF-8

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

# add GPG key for ros distro
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# add repository to sources.list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install dev tools
sudo apt update && sudo apt install ros-dev-tools -y

# install ROS
sudo apt upgrade -y
sudo apt install ros-rolling-desktop -y

# source ROS
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/rolling/setup.bash

# Install ROS dependencies
sudo apt install -y ros-rolling-rqt

mkdir -p ~/camera_ws/
cd ~/camera_ws/
git clone https://github.com/christianrauch/camera_ros.git src/camera_ros
rosdep install --from-paths src --ignore-src --skip-keys=libcamera
colcon build
chmod +x ./install/setup.bash
source ./install/setup.bash
cd $DIR

cd imu_ws
colcon build
cd $DIR

git clone --recursive https://github.com/vertueux/i2c_pwm_board.git
cd i2c_pwm_board/scripts
chmod +x install_dependencies.sh
./install_dependencies.sh
cd ..
colcon build
chmod +x ./install/setup.bash
source install/setup.bash
cd $DIR

exit 0 # Exit the script cause no need for mediamtx anymore
# Download the file from the link
wget -O /tmp/mediamtx.tar.gz "https://github.com/bluenviron/mediamtx/releases/download/v1.9.2/mediamtx_v1.9.2_linux_arm64v8.tar.gz"


# Create a directory to extract the downloaded file
mkdir /tmp/mediamtx

# Extract the downloaded file
tar -xzf /tmp/mediamtx.tar.gz -C /tmp/mediamtx

# Make the extracted file executable
chmod +x /tmp/mediamtx/mediamtx

# Move the executable to a directory in PATH
sudo mv /tmp/mediamtx/mediamtx ./

# Clean up by deleting the downloaded and extracted files
rm -rf /tmp/mediamtx.tar.gz /tmp/mediamtx
