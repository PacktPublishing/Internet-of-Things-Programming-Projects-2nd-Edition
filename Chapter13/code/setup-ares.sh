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
apt-get install -y python3-pip python3-smbus i2c-tools

# Enable I2C
echo "Enabling I2C interface..."
modprobe i2c-dev
echo "i2c-dev" | tee /etc/modules-load.d/i2c-dev.conf


# Setup udev rule for /dev/serial0 permissions
echo "Setting up udev rule for persistent /dev/serial0 permissions..."
echo 'SUBSYSTEM=="tty", ATTRS{device}=="/dev/serial0", MODE="0666"' > /etc/udev/rules.d/99-serial.rules

# Reload udev rules and trigger them
udevadm control --reload-rules
udevadm trigger


# Install Adafruit Blinka for CircuitPython libraries support
echo "Installing Adafruit Blinka and other Python libraries..."
pip3 install --upgrade pip
pip3 install Adafruit-Blinka adafruit-circuitpython-vl53l0x pyserial RPi.GPIO

# Ensure the correct version of paho-mqtt is installed
echo "Installing paho-mqtt version 1.5.1..."
pip3 install paho-mqtt==1.5.1


# Install ROS Humble Hawksbill
echo "Installing ROS Humble Hawksbill and required packages..."
apt update && apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros2.list
apt update
apt install -y ros-humble-ros-base

# Install Colcon build system
echo "Installing Colcon build system..."
apt install -y python3-colcon-common-extensions

# Install rosdep
echo "Installing rosdep..."
apt install -y python3-rosdep

# Initialize rosdep
echo "Initializing rosdep..."
rosdep init
echo "Fixing rosdep permissions..."
rosdep fix-permissions

# Drop privileges for rosdep update
# Note: The script assumes it's being run as root; this step temporarily drops root privileges for 'rosdep update'
sudo -u $SUDO_USER rosdep update


# Setup complete
echo "Setup complete! Please reboot your system."

echo "Use SSH to login to this Raspberry Pi. The IP address is:"
hostname -I
