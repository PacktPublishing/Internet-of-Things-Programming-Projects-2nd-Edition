#!/bin/bash

# Ensure the script is executed with superuser privileges
if [ "$(id -u)" != "0" ]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

echo "Starting setup..."

# Update and upgrade the system
echo "Updating and upgrading your system. This might take a while."
apt-get update && apt-get upgrade -y

# Install Python 3, PIP, and other essential packages
echo "Installing Python 3, PIP, and necessary packages..."
apt-get install -y python3-pip python3-smbus i2c-tools git

# Enable I2C interface
echo "Enabling the I2C interface..."
raspi-config nonint do_i2c 0

# Enable UART interface
echo "Enabling the UART interface and disabling the serial console..."
raspi-config nonint do_serial 0

# Install Adafruit Blinka (CircuitPython library support on Raspberry Pi)
echo "Installing Adafruit Blinka (CircuitPython support)..."
pip3 install --upgrade pip
pip3 install Adafruit-Blinka

# Install the Adafruit library for the VL53L0X sensor
echo "Installing the Adafruit library for VL53L0X..."
pip3 install adafruit-circuitpython-vl53l0x

# Install PySerial for UART communication
echo "Installing PySerial for UART communication..."
pip3 install pyserial

echo "Setup complete!"

# Reminder to reboot
echo "Please reboot your Raspberry Pi to ensure all changes take effect."
