#!/bin/bash
set -e
trap 'echo "❌ Script interrupted at line $LINENO"; exit 1' ERR

# -------------------------------
# Configurable Parameters
# -------------------------------
DEV_USERNAME=dev
DEV_PASSWORD=password

print_info() { echo -e "\e[34m➡ $1\e[0m"; }
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

cat << 'EOF' | sudo tee /home/$DEV_USERNAME/dev_setup.sh > /dev/null
#!/bin/bash
set -e
trap 'echo "❌ Dev setup interrupted at line $LINENO"; exit 1' ERR

print_info() { echo -e "\e[34m➡ $1\e[0m"; }
print_success() { echo -e "\e[32m✔ $1 completed successfully\e[0m"; }

# Install essentials
print_info "Installing essential tools"
sudo apt update
sudo apt install -y curl git python3-pip software-properties-common p7zip-full can-utils net-tools locales

# Setup locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository and key (Humble assumed)
print_info "Setting up ROS 2 source (Ubuntu 22.04 only)"
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-rosdep python3-colcon-common-extensions

sudo rosdep init || echo "rosdep already initialized"
rosdep update

# Setup workspace
print_info "Setting up ros2_ws"
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/O-Robotics/ublox_dgnss.git
cd ~/ros2_ws
colcon build --symlink-install

# Add sourcing to .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Setup GNSS udev rules
print_info "Adding udev rule for GNSS"
echo 'ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-ublox-gnss.rules
sudo udevadm control --reload-rules

print_success "Dev environment setup complete! Run 'source ~/.bashrc' or restart terminal."
EOF

# Step 4: Set ownership and permission
sudo chown "$DEV_USERNAME:$DEV_USERNAME" "/home/$DEV_USERNAME/dev_setup.sh"
sudo chmod +x "/home/$DEV_USERNAME/dev_setup.sh"

print_success "dev_setup.sh created and made executable"

# Final instruction
echo -e "\n🟢 \e[1mNow switch to the '$DEV_USERNAME' user and run:\e[0m"
echo -e "    su - $DEV_USERNAME"
echo -e "    chmod +x dev_setup.sh && ./dev_setup.sh\n"
