#!/bin/bash
set -e
# Install and enable a systemd service for Zenoh ROS2DDS bridge on RPi/Jetson.
# Usage: sudo bash install_service.sh [username] [config_path]
# Defaults: username=dev, config_path=$HOME/zenoh_ros2dds/bridge-rpi.json5 (resolved for the specified user)

SERVICE_NAME=zenoh-ros2dds.service
ZENOH_VER="1.5.1"
TARGET_USER="${1:-dev}"

# Resolve HOME for target user
USER_HOME=$(getent passwd "$TARGET_USER" | cut -d: -f6)
if [ -z "$USER_HOME" ] || [ ! -d "$USER_HOME" ]; then
  echo "User home not found for $TARGET_USER"
  exit 1
fi

CONFIG_PATH_INPUT="${2:-$USER_HOME/zenoh_ros2dds/bridge-rpi.json5}"
CONFIG_PATH=$(realpath -m "$CONFIG_PATH_INPUT" 2>/dev/null || echo "$CONFIG_PATH_INPUT")

# Detect arch
uname_m=$(uname -m)
case "$uname_m" in
  aarch64|arm64)
    ARCH="aarch64-unknown-linux-gnu"
    ;;
  armv7l|armv6l)
    ARCH="armv7-unknown-linux-gnueabihf"
    ;;
  *)
    echo "Unsupported architecture: $uname_m"
    exit 1
    ;;
 esac

BRIDGE_DIR="$USER_HOME/zenoh_ros2dds/zenoh-bridge-ros2dds-${ZENOH_VER}-${ARCH}-standalone"
BRIDGE_BIN="$BRIDGE_DIR/ros2dds"

if [ ! -x "$BRIDGE_BIN" ]; then
  echo "Bridge binary not found at $BRIDGE_BIN. Run setup_ros2dds_zenoh.sh as $TARGET_USER first."
  exit 1
fi

UNIT_PATH="/etc/systemd/system/$SERVICE_NAME"

echo "Creating systemd unit at $UNIT_PATH ..."
cat <<EOF | sudo tee "$UNIT_PATH" > /dev/null
[Unit]
Description=Zenoh ROS2DDS Bridge
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$TARGET_USER
Environment=RUST_LOG=info
Restart=on-failure
RestartSec=2
ExecStart=$BRIDGE_BIN -c $CONFIG_PATH
WorkingDirectory=$USER_HOME/zenoh_ros2dds

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
echo "Service installed and enabled. Start now with: sudo systemctl start $SERVICE_NAME"
echo "Check status with: sudo systemctl status $SERVICE_NAME"
