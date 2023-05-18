#!/bin/bash

# Tutorial from: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html#id2

# Setup print colors for nice bash script formatting: https://stackoverflow.com/questions/5947742/how-to-change-the-output-color-of-echo-in-linux
RED='\033[0;31m'
NC='\033[0m' # No Color

# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify settings
echo -e "${RED}VERIFY UTF-8 SETTINGS:${NC}"
locale

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update

sudo apt upgrade

# Select appropriate version
sudo apt install ros-foxy-desktop python3-argcomplete # Desktop Install
#sudo apt install ros-foxy-ros-base python3-argcomplete # ROS-Base Install
#sudo apt install ros-dev-tools # Dev Install

# Environment setup
# source /opt/ros/foxy/setup.sh

echo -e "${RED}INSTALLATION COMPLETE, SOURCE INSTRUCTIONS:${NC}"
echo "Add a command to ~/.bashrc by completing these steps:"
echo "     1. Run the following command: nano ~/.bashrc"
echo "     2. At the bottom, add the line: source /opt/ros/foxy/setup.sh"
echo "     3. Save and exit ~/.bashrc"
echo "     4. Run the following command: source ~/.bashrc"
