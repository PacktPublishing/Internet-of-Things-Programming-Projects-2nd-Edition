#!/bin/bash

# Ensure the script is executed with superuser privileges
if [ "$(id -u)" != "0" ]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

echo "Beginning setup..."

# Update and upgrade the system
echo "Updating and upgrading your system. This might take a while."
apt-get update && apt-get upgrade -y

# Install necessary system utilities
echo "Installing necessary system utilities..."
apt-get install -y python3-pip python3-smbus i2c-tools git libi2c-dev

# Enable I2C and UART
echo "Enabling I2C and UART interfaces..."
raspi-config nonint do_i2c 0
raspi-config nonint do_serial 0

# Install Adafruit Blinka for CircuitPython libraries support
echo "Installing Adafruit Blinka and other Python libraries for hardware interfacing..."
pip3 install --upgrade pip
pip3 install Adafruit-Blinka adafruit-circuitpython-vl53l0x pyserial RPi.GPIO

# Install ROS 2 Humble Hawksbill
echo "Setting up ROS 2 Humble Hawksbill environment..."
# Add the ROS 2 apt repository
apt update && apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble Hawksbill
apt update
apt install -y ros-humble-desktop

# Initialize rosdep
rosdep init
rosdep update

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional ROS 2 utilities
apt install -y python3-colcon-common-extensions python3-rosdep

# Update pip and setuptools
pip3 install -U pip setuptools

echo "Setup complete! Please restart."
