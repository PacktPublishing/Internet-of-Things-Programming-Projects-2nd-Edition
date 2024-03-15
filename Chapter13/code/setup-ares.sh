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

# Give permission to /dev/serial0 for all users
echo "Granting permission to /dev/serial0 for all users..."
chmod a+rw /dev/serial0

# I2C and UART are typically enabled by default on Ubuntu 22.04 for Raspberry Pi,
# but you might need to manually enable them in /boot/firmware/usercfg.txt if not.
# Uncomment and edit the following lines as needed:
#echo "dtparam=i2c_arm=on" >> /boot/firmware/usercfg.txt
#echo "dtparam=spi=on" >> /boot/firmware/usercfg.txt
#echo "enable_uart=1" >> /boot/firmware/usercfg.txt

# Install Adafruit Blinka for CircuitPython libraries support
echo "Installing Adafruit Blinka and other Python libraries..."
pip3 install --upgrade pip
pip3 install Adafruit-Blinka adafruit-circuitpython-vl53l0x pyserial RPi.GPIO

# Ensure the correct version of paho-mqtt is installed
echo "Installing paho-mqtt version 1.5.1..."
pip3 install paho-mqtt==1.5.1

# Continue with the installation of ROS, colcon, and other components...


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


# Environment setup
echo "Adding ROS 2 environment setup to your bashrc..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# Source the environment for this script session to use ROS commands
source /opt/ros/humble/setup.bash

# Setup complete
echo "Setup complete! Please reboot your system."

echo "Use SSH to login to this Raspberry Pi. The IP address is:"
hostname -I
