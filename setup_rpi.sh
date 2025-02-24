#!/bin/bash

# This script automates the setup of a Raspberry Pi with Ubuntu Desktop 22.04 LTS

# Update the system
sudo apt update
sudo apt upgrade -y

# Install essential tools
sudo apt install -y openssh-server curl git minicom python3-pip software-properties-common p7zip

# Setup SSH for remote access
sudo systemctl enable ssh
sudo systemctl start ssh

# ------------ dev ------------ 
# Create a new user (change 'dev' to any desired username)
echo "Creating user 'dev'"
sudo useradd -m -s /bin/bash dev
echo "Setting password for 'dev'"
echo "dev:password" | sudo chpasswd  # Replace 'password' with a secure password
sudo usermod -aG sudo dev  # Add user to sudo group (if necessary)

# ------------ Tailscale ------------ 
# Install Tailscale for remote network access
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list
sudo apt-get update
sudo apt-get install -y tailscale
sudo tailscale up

# ------------ CAN ------------ 
# Install CAN utilities
echo "Installing CAN utilities..."
sudo apt-get install -y can-utils

# Bring CAN interface down if it's already up
echo "Bringing can0 down..."
sudo ip link set can0 down

# Set up CAN interface
echo "Configuring CAN interface..."
sudo ip link set can0 up type can bitrate 1000000
sudo ipconfig can0 txqueuelen 65536

# Test CAN bus communication
echo "Testing CAN bus communication..."
dmesg | grep spi0
ipconfig can0

# Test sending and receiving CAN data
candump can0 &  # Start receiving data in the background
sleep 2
cansend can0 000#11.22.33.44
echo "CAN test completed. Check the candump output for received data."

# ------------ ROS 2 Installation ------------ 
# Check and set locale to UTF-8
echo "Checking and setting locale to UTF-8"
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # Verify locale settings

# Enable Ubuntu Universe repository
sudo add-apt-repository universe

# Add ROS 2 GPG key and repository
echo "Setting up ROS 2 repository"
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install base and dev tools
sudo apt install -y ros-humble-ros-base
sudo apt install -y ros-dev-tools
sudo apt install -y python3-rosdep

# Set up ROS 2 environment automatically on terminal start
echo "Setting up ROS 2 environment"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash


# ------------ ROS 2 Dependencies------------ 
# Install ROS2 Humble related packages
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro
sudo apt install -y python3-colcon-common-extensions

# Set up ROS2 workspace
mkdir ros2_ws
cd ros2_ws
colcon build --symlink-install

# ------------ foxglove ------------ 
# sudo apt install -y ros-humble-foxglove-bridge

# ------------ git ------------ 
# Install Git (if not already installed)
sudo apt install -y git

# # Configure Git (set up user information for Git commits)
# echo "Configuring Git user"
# sudo -u dev git config --global user.name "Dev User"
# sudo -u dev git config --global user.email "dev@example.com"  # Replace with actual email

# # Initialize git repository
# git init

# ------------ ssh key ------------ 
# # Generate SSH key and link to GitHub (document the steps)
# echo "Follow the instructions to generate SSH key and add to GitHub: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent"


# ------------ ROS 2 GNSS ------------ 
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

# ------------ The end ------------ 
# Final steps: Done, restart terminal to apply changes
echo "Setup complete! Restart the terminal to apply environment settings."
