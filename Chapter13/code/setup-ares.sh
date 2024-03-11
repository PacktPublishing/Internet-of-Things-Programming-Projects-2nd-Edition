#!/bin/bash

# Ensure the script is executed with superuser privileges
if [ "$(id -u)" != "0" ]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

# Update the system
echo "Updating and upgrading your system. This might take a while."
sudo apt-get update && sudo apt-get upgrade -y

# Set Locale
echo "Setting up the locale..."
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS 2 repository to the system
echo "Adding the ROS 2 repository..."
apt update && apt install -y software-properties-common curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
echo "Installing ROS 2 Humble Hawksbill and development tools..."
apt update
apt install -y ros-humble-ros-base python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
echo "Initializing rosdep..."
rosdep init
rosdep update

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

# Update pip to its latest version
pip3 install --upgrade pip

# Install required Python libraries for serial communication and MQTT
echo "Installing pyserial and paho-mqtt for serial communication and MQTT functionality..."
pip3 install pyserial paho-mqtt

# Install I2C library for Python
echo "Installing SMBus for I2C communication..."
apt install -y python3-smbus

# Install CircuitPython libraries required for VL53L0X
echo "Installing CircuitPython libraries for VL53L0X..."
pip3 install adafruit-circuitpython-vl53l0x

# Enable I2C interface
echo "Enabling I2C interface..."
raspi-config nonint do_i2c 0

# Install raspi-config
echo "Installing raspi-config for Raspberry Pi configuration..."
apt install -y raspi-config

# Install ROS 2 demo nodes packages
echo "Installing ROS 2 demo nodes packages..."
apt install -y ros-humble-demo-nodes-py ros-humble-demo-nodes-cpp

echo "ROS 2 Humble Hawksbill installation, Python libraries for serial communication, MQTT, SMBus for I2C, CircuitPython libraries for VL53L0X, and raspi-config setup is complete."

# Modified instructions to run demo nodes without a GUI
echo "To run the ROS 2 demo nodes, use the following commands in your terminal:"
echo "Run the C++ talker node in the background: source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker > /dev/null 2>&1 &"
echo "Then, run the Python listener node: source /opt/ros/humble/setup.bash && ros2 run demo_nodes_py listener"
