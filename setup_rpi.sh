#!/bin/bash

# This script automates the setup of a Raspberry Pi with Ubuntu Desktop 22.04 LTS

# Update the system
sudo apt update
sudo apt upgrade -y

# Install essential tools
sudo apt install -y openssh-server curl git minicom python3-pip

# Setup SSH for remote access
sudo systemctl enable ssh
sudo systemctl start ssh

# Create a new user (change 'dev' to any desired username)
echo "Creating user 'dev'"
sudo useradd -m -s /bin/bash dev
echo "Setting password for 'dev'"
echo "dev:password" | sudo chpasswd  # Replace 'password' with a secure password
sudo usermod -aG sudo dev  # Add user to sudo group (if necessary)

# Configure Git (set up user information for Git commits)
echo "Configuring Git user"
sudo -u dev git config --global user.name "Dev User"
sudo -u dev git config --global user.email "dev@example.com"  # Replace with actual email

# Install Tailscale for remote network access
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list
sudo apt-get update
sudo apt-get install -y tailscale
sudo tailscale up

# # Connect to Wifi (if needed, replace <ESSID_NAME> and <ESSID_PASSWORD>)
# # Replace these with the correct SSID and password for your network
# echo "Connecting to WiFi..."
# sudo nmcli dev wifi connect <ESSID_NAME> password <ESSID_PASSWORD>

# Setup Modem (for 4G modem connection)
sudo apt-get install minicom
sudo minicom -D /dev/ttyUSB2
echo "Enter the following commands to configure the modem:"
echo "ATE1"
echo "AT+CUSBPIDSWITCH=9011,1,1"
echo "Press ctrl-A X to exit minicom"
sudo dhclient -v usb0

# Install ROS2 Humble
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro
sudo apt install -y python3-colcon-common-extensions

# Set up ROS2 workspace
mkdir ros2_ws
cd ros2_ws
colcon build --symlink-install

# # Install foxglove dependencies
# sudo apt install -y ros-humble-foxglove-bridge

# Set up ROS2 environment automatically on terminal start
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Install Git (if not already installed)
sudo apt install -y git

# Initialize git repository
git init

# # Generate SSH key and link to GitHub (document the steps)
# echo "Follow the instructions to generate SSH key and add to GitHub: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent"

# Install differential GNSS package
echo "Installing differential GNSS package"
cd ~/ros2_ws/src
git clone https://github.com/O-Robotics/ublox_dgnss.git

# Setup udev rule for GNSS device
echo "Setting up udev rule"
echo 'ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-ublox-gnss.rules

# Reload udev rules
echo "Reloading udev rules"
sudo udevadm control --reload-rules

# Verify udev rule (instructions for the user)
echo "To verify udev rule, run:"
echo "lsusb"
echo "Then, check the device permissions:"
echo "ls -l /dev/bus/usb/<bus_number>/<device_number>"
echo "Ensure it shows 'root' permission and 'plugdev' group."

# Build the workspace with the differential GNSS package
echo "Building the ROS2 workspace with differential GNSS package"
cd ~/ros2_ws
colcon build --packages-select ublox_dgnss

# Source the ROS2 workspace
source ~/ros2_ws/install/setup.bash


# Final steps: Done, restart terminal to apply changes
echo "Setup complete! Restart the terminal to apply environment settings."
