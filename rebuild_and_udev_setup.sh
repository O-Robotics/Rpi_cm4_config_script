#!/bin/bash

# === CONFIGURATION ===
WS_PATH=~/ORobotics/localization_ws

# === FUNCTIONS ===
print_info() {
    echo -e "\033[1;34m[INFO]\033[0m $1"
}

print_success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"
}

udev_rule_exists() {
    local rule_file=$1
    local match_str=$2
    if [ -f "$rule_file" ] && grep -q "$match_str" "$rule_file"; then
        return 0  # exists
    else
        return 1  # not exists
    fi
}

# === START ===
print_info "Sourcing ROS 2 Humble environment"
source /opt/ros/humble/setup.bash

print_info "Switching to workspace: $WS_PATH"
cd $WS_PATH

# === BUILD ROS PACKAGES STEP BY STEP ===
print_info "Building: rtcm_msgs"
colcon build --packages-select rtcm_msgs

print_info "Building: ublox_ubx_interfaces ublox_ubx_msgs ublox_dgnss_node ublox_nav_sat_fix_hp_node ntrip_cli"
colcon build --packages-select ublox_ubx_interfaces ublox_ubx_msgs ublox_dgnss_node ublox_nav_sat_fix_hp_node ntrip_cli

print_info "Building: ublox_dgnss"
colcon build --packages-select ublox_dgnss

print_info "Building: wit_ros2_imu"
colcon build --packages-select wit_ros2_imu

print_info "Building: AMR-Sweeper_description"
colcon build --packages-select AMR-Sweeper_description

print_success "All specified ROS 2 packages built successfully!"

# === SETUP UDEV RULES ===
GNSS_RULE_FILE="/etc/udev/rules.d/99-ublox-gnss.rules"
IMU_RULE_FILE="/etc/udev/rules.d/99-imu.rules"

print_info "Checking GNSS udev rule"
if udev_rule_exists "$GNSS_RULE_FILE" '1546.*01a9'; then
    print_info "GNSS udev rule already exists, skipping"
else
    print_info "Adding udev rule for GNSS"
    echo 'ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666"' | sudo tee "$GNSS_RULE_FILE"
fi

print_info "Checking IMU udev rule"
if udev_rule_exists "$IMU_RULE_FILE" '1a86.*7523'; then
    print_info "IMU udev rule already exists, skipping"
else
    print_info "Adding udev rule for IMU (creates /dev/imu_usb)"
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="imu_usb"' | sudo tee "$IMU_RULE_FILE"
fi

# === RELOAD UDEV RULES ===
print_info "Reloading udev rules"
sudo udevadm control --reload-rules
sudo udevadm trigger

print_success "udev rules checked, updated, and reloaded!"
print_info "You can verify: ls -l /dev/imu_usb"
