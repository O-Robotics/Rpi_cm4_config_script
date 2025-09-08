#!/bin/bash
set -e

# Install Zenoh ROS2DDS bridge for ROS 2 Humble on ARM devices (RPi/Jetson)
# English-only, no emojis. Designed for endpoint devices publishing topics over the network.

ZENOH_VER="1.5.1"
INSTALL_DIR="$HOME/zenoh_ros2dds"

# Detect architecture: armhf/armv7 vs aarch64
uname_m=$(uname -m)
case "$uname_m" in
  aarch64|arm64)
    ARCH="aarch64-unknown-linux-gnu"
    ;;
  armv7l|armv6l)
    ARCH="armv7-unknown-linux-gnueabihf"
    ;;
  *)
    echo "Unsupported architecture: $uname_m. Set ARCH manually by editing this script."
    exit 1
    ;;
 esac

BRIDGE_ZIP="zenoh-bridge-ros2dds-${ZENOH_VER}-${ARCH}-standalone.zip"
BRIDGE_URL="https://github.com/eclipse-zenoh/zenoh-bridge-ros2dds/releases/download/v${ZENOH_VER}/${BRIDGE_ZIP}"

echo "[INFO] Installing dependencies..."
sudo apt update && sudo apt install -y curl unzip

echo "[INFO] Creating installation directory..."
mkdir -p "$INSTALL_DIR"
cd "$INSTALL_DIR"

echo "[INFO] Downloading Zenoh bridge binary: $BRIDGE_ZIP"
curl -fL -o "$BRIDGE_ZIP" "$BRIDGE_URL"
unzip -o "$BRIDGE_ZIP"
chmod +x zenoh-bridge-ros2dds*/ros2dds || true

BRIDGE_BIN_DIR="$INSTALL_DIR/zenoh-bridge-ros2dds-${ZENOH_VER}-${ARCH}-standalone"
if [ ! -d "$BRIDGE_BIN_DIR" ]; then
  echo "[ERROR] Expected directory not found: $BRIDGE_BIN_DIR"
  exit 1
fi

echo "[INFO] Adding to PATH via ~/.bashrc (if not present)..."
if ! grep -q "$BRIDGE_BIN_DIR" "$HOME/.bashrc" 2>/dev/null; then
  echo "export PATH=\"$BRIDGE_BIN_DIR:\$PATH\"" >> "$HOME/.bashrc"
fi

cat <<EOF
[INFO] Setup complete.
To use now in this shell:
  export PATH="$BRIDGE_BIN_DIR:$PATH"

Example run:
  ros2dds -c "$INSTALL_DIR/bridge-rpi.json5"
EOF