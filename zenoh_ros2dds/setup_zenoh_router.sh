#!/bin/bash
set -e
# Install zenoh router (zenohd) for Linux PC (x86_64) by default.
# Adjust ARCH if you need a different build.

ZENOH_VER="1.5.1"
INSTALL_DIR="$HOME/zenoh_router"

# Detect architecture (default to x86_64 for PC)
uname_m=$(uname -m)
case "$uname_m" in
  x86_64|amd64)
    ARCH="x86_64-unknown-linux-gnu"
    ;;
  aarch64|arm64)
    ARCH="aarch64-unknown-linux-gnu"
    ;;
  *)
    echo "Unsupported architecture: $uname_m. Set ARCH manually."
    exit 1
    ;;
 esac

ROUTER_TGZ="zenoh-${ZENOH_VER}-${ARCH}.tar.gz"
ROUTER_URL="https://github.com/eclipse-zenoh/zenoh/releases/download/v${ZENOH_VER}/${ROUTER_TGZ}"

echo "[INFO] Installing dependencies..."
sudo apt update && sudo apt install -y curl tar

echo "[INFO] Creating installation directory..."
mkdir -p "$INSTALL_DIR"
cd "$INSTALL_DIR"

echo "[INFO] Downloading zenohd: $ROUTER_TGZ"
curl -fL -o "$ROUTER_TGZ" "$ROUTER_URL"
tar -xzf "$ROUTER_TGZ"

# Find zenohd binary
ROUTER_BIN=$(find "$INSTALL_DIR" -type f -name zenohd -perm -111 | head -n1)
if [ -z "$ROUTER_BIN" ]; then
  echo "[ERROR] zenohd binary not found after extraction."
  exit 1
fi

# Add to PATH
if ! grep -q "$(dirname "$ROUTER_BIN")" "$HOME/.bashrc" 2>/dev/null; then
  echo "export PATH=\"$(dirname "$ROUTER_BIN"):\$PATH\"" >> "$HOME/.bashrc"
fi

echo "[INFO] zenohd installed at: $ROUTER_BIN"
echo "Run: zenohd -l tcp/0.0.0.0:7447"