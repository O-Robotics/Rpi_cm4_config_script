#!/bin/bash
set -e
trap 'echo "❌ Script interrupted at line $LINENO"; exit 1' ERR

# -------------------------------
# Configurable Parameters
# -------------------------------
DEV_USERNAME=dev
DEV_PASSWORD=password

print_info() { echo -e "\e[34m🤖 $1\e[0m"; }
print_success() { echo -e "\e[32m✔ $1 completed successfully\e[0m"; }
print_warning() { echo -e "\e[33m⚠ $1\e[0m"; }

# Step 1: Delete existing user if exists
if id "$DEV_USERNAME" &>/dev/null; then
    print_warning "User '$DEV_USERNAME' already exists. Deleting..."
    sudo deluser --remove-home "$DEV_USERNAME"
    print_success "User '$DEV_USERNAME' deleted"
fi

# Step 2: Create fresh dev user
print_info "Creating user '$DEV_USERNAME'"
sudo useradd -m -s /bin/bash "$DEV_USERNAME"
echo "$DEV_USERNAME:$DEV_PASSWORD" | sudo chpasswd
sudo usermod -aG sudo "$DEV_USERNAME"
print_success "User '$DEV_USERNAME' created with password '$DEV_PASSWORD'"

# Step 3: Create dev_setup.sh inside dev's home
print_info "Generating setup script at /home/$DEV_USERNAME/dev_setup.sh"

cat << EOF | sudo tee /home/$DEV_USERNAME/dev_setup.sh > /dev/null
#!/bin/bash
set -e
trap 'echo "❌ Dev setup interrupted at line $LINENO"; exit 1' ERR

print_info() { echo -e "\e[34m🤖 $1\e[0m"; }
print_success() { echo -e "\e[32m✔ $1 completed successfully\e[0m"; }

# Install essentials
print_info "🤖 Installing essential tools"
sudo apt update
sudo apt install -y curl git python3-pip software-properties-common can-utils net-tools locales gnupg

# Setup locale
print_info "🤖 Configuring locale"
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 Humble source
print_info "🤖 Setting up ROS 2 source (Ubuntu 22.04)"
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
echo "deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-rosdep python3-colcon-common-extensions

# Initialize rosdep
print_info "🤖 Initializing rosdep"
sudo rosdep init || echo "rosdep already initialized"
rosdep update

# Setup workspace
print_info "🤖 Setting up ROS 2 workspace"
WS_ROOT=~/Documents/ORobotics
WS_NAME=localization_ws
WS_PATH=\$WS_ROOT/\$WS_NAME
mkdir -p \$WS_PATH/src
cd \$WS_PATH/src

# Clone necessary packages
print_info "🤖 Cloning GNSS, IMU, and Sweeper description packages"
git clone https://github.com/O-Robotics/ublox_dgnss.git
git clone https://github.com/tilk/rtcm_msgs.git
git clone https://github.com/O-Robotics/wit_ros2_imu.git
git clone https://github.com/O-Robotics/AMR-Sweeper_description.git

# Install dependencies
print_info "🤖 Installing ROS package dependencies"
cd \$WS_PATH
rosdep install --from-paths src --ignore-src -r -y

# Build packages
print_info "🤖 Building ROS 2 packages step by step"
colcon build --packages-select rtcm_msgs
colcon build --packages-select ublox_ubx_interfaces ublox_ubx_msgs ublox_dgnss_node ublox_nav_sat_fix_hp_node ntrip_client_node
colcon build --packages-select ublox_dgnss
colcon build --packages-select wit_ros2_imu
colcon build --packages-select AMR-Sweeper_description

# Source on terminal start
print_info "🤖 Adding ROS 2 source to .bashrc"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source \$WS_PATH/install/setup.bash" >> ~/.bashrc

# Setup GNSS udev rule
print_info "🤖 Adding udev rule for GNSS"
echo 'ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ublox-gnss.rules

# Setup IMU udev rule
print_info "🤖 Adding udev rule for IMU (creates /dev/imu_usb)"
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="imu_usb"' | sudo tee /etc/udev/rules.d/imu_usb.rules

# Reload udev
print_info "🤖 Reloading udev rules"
sudo udevadm control --reload-rules
sudo udevadm trigger

print_success "🤖\n🟢 Dev environment setup complete!"
print_info "✅ You can verify: ls -l /dev/imu_usb"
EOF

# Step 4: Set permissions
sudo chown "$DEV_USERNAME:$DEV_USERNAME" "/home/$DEV_USERNAME/dev_setup.sh"
sudo chmod +x "/home/$DEV_USERNAME/dev_setup.sh"

print_success "dev_setup.sh created and made executable"

# Final instruction
echo -e "\n🟢 \e[1mNow switch to the '$DEV_USERNAME' user and run:\e[0m"
echo -e "    su - $DEV_USERNAME"
echo -e "    ./dev_setup.sh\n"
