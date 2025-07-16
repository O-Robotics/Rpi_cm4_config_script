#!/bin/bash
set -e
trap 'echo "❌ Script interrupted at line $LINENO"; exit 1' ERR

# -------------------------------
# Configurable Parameters
# -------------------------------
DEV_USERNAME=dev
DEV_PASSWORD=password
ROB_USERNAME=rob
ROB_PASSWORD=robot

print_info() { echo -e "\e[34m🤖 $1\e[0m"; }
print_success() { echo -e "\e[32m✔ $1 completed successfully\e[0m"; }
print_warning() { echo -e "\e[33m⚠ $1\e[0m"; }

create_user() {
    local username=$1
    local password=$2
    if id "$username" &>/dev/null; then
        print_warning "User '$username' already exists. Deleting..."
        sudo pkill -u "$username" || true
        sudo deluser --remove-home "$username"
        print_success "User '$username' deleted"
    fi
    print_info "Creating user '$username'"
    sudo useradd -m -s /bin/bash "$username"
    echo "$username:$password" | sudo chpasswd
    sudo usermod -aG sudo "$username"
    print_success "User '$username' created with password '$password'"
}

# Step 1: Create dev user
create_user "$DEV_USERNAME" "$DEV_PASSWORD"

# Step 2: Create rob user
create_user "$ROB_USERNAME" "$ROB_PASSWORD"

# Step 3: Generate dev_setup.sh in both dev and rob home
for USER in "$DEV_USERNAME" "$ROB_USERNAME"; do
    print_info "Generating setup script at /home/$USER/dev_setup.sh"
    cat << 'EOF' | sudo tee "/home/$USER/dev_setup.sh" > /dev/null
#!/bin/bash
set -e
trap 'echo "❌ Dev setup interrupted at line \$LINENO"; exit 1' ERR

print_info() { echo -e "\e[34m🤖 \$1\e[0m"; }
print_success() { echo -e "\e[32m✔ \$1\e[0m"; }

# Install essentials
print_info "Installing essential tools"
sudo apt update
sudo apt install -y curl git python3-pip software-properties-common can-utils net-tools locales gnupg

# Setup locale
print_info "Configuring locale"
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 Humble source
print_info "Setting up ROS 2 source (Ubuntu 22.04)"
sudo add-apt-repository universe -y
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
echo "deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-rosdep python3-colcon-common-extensions ros-humble-ros-environment

# Install gazebo & ROS2 Control dependencies
print_info "Installing Gazebo and ROS2 Control dependencies"
sudo apt install -y ros-humble-ros-ignition
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control \
    ros-humble-xacro ros-humble-joy ros-humble-twist-mux ros-humble-joint-state-broadcaster ros-humble-joint-state-publisher

# Initialize rosdep
print_info "Initializing rosdep"
sudo rosdep init || echo "rosdep already initialized"
rosdep update

# Setup workspace
print_info "Setting up ROS 2 workspace"
WS_ROOT=~/O_Robotics
WS_NAME=ros2_ws
WS_PATH=\$WS_ROOT/\$WS_NAME
mkdir -p \$WS_PATH/src
cd \$WS_PATH/src

source /opt/ros/humble/setup.bash

# Clone necessary packages
print_info "Cloning GNSS, IMU, Sweeper description, and ROS2 Control packages"
git clone https://github.com/O-Robotics/ublox_dgnss.git
git clone https://github.com/tilk/rtcm_msgs.git
git clone https://github.com/O-Robotics/wit_ros2_imu.git
git clone https://github.com/O-Robotics/amr_sweeper_description.git
git clone https://github.com/O-Robotics/ROS2_Control.git

# Install dependencies
print_info "Installing ROS package dependencies"
cd \$WS_PATH
rosdep install --from-paths src --ignore-src -r -y

# Build packages
print_info "Building ROS 2 packages step by step"
colcon build --packages-select rtcm_msgs
colcon build --packages-select ublox_ubx_interfaces ublox_ubx_msgs ublox_dgnss_node ublox_nav_sat_fix_hp_node ntrip_client_node
colcon build --packages-select ublox_dgnss
colcon build --packages-select wit_ros2_imu
colcon build --packages-select amr_sweeper_description
colcon build --packages-select ROS2_Control

# Source on terminal start
print_info "Adding ROS 2 source to .bashrc"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source \$WS_PATH/install/setup.bash" >> ~/.bashrc

# Setup GNSS udev rule
print_info "Adding udev rule for GNSS"
echo 'ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ublox-gnss.rules

# Setup IMU udev rule
print_info "Adding udev rule for IMU (creates /dev/imu_usb)"
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="imu_usb"' | sudo tee /etc/udev/rules.d/99-imu.rules

# Reload udev
print_info "Reloading udev rules"
sudo udevadm control --reload-rules
sudo udevadm trigger

print_success "🟢 Dev environment setup complete!"
print_info "✅ Example commands to run:"
echo "source install/setup.bash"
echo "ros2 launch amr_sweeper_bringup bringup.launch.py use_sim_time:=true"
echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped"
echo "For physical robot:"
echo "ros2 launch amr_sweeper_bringup bringup.launch.py"
EOF

    sudo chown "$USER:$USER" "/home/$USER/dev_setup.sh"
    sudo chmod +x "/home/$USER/dev_setup.sh"
    print_success "dev_setup.sh created and made executable for $USER"
done

# Final instruction
echo -e "\n🟢 \e[1mNow switch to the user and run:\e[0m"
echo -e "    su - $DEV_USERNAME"
echo -e "    ./dev_setup.sh"
echo -e "\nOR for production:\n"
echo -e "    su - $ROB_USERNAME"
echo -e "    ./dev_setup.sh\n"
