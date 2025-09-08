#!/bin/bash
set -e
# Unified launcher for Zenoh + ROS2DDS on both endpoint (RPi/Jetson) and simulation PC.
# English-only, no emojis.
# Role detection can be overridden with ZENOH_ROLE=endpoint|simulation_pc
# Optional args:
#   --config <path>   : override config path (defaults depend on role)
#   --router <addr>   : override router endpoint, e.g., tcp/192.168.1.100:7447
#   --no-router       : on PC, do not start zenohd automatically

ROLE=""
CONFIG_OVERRIDE=""
ROUTER_ADDR_OVERRIDE=""
AUTO_START_ROUTER=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG_OVERRIDE="$2"; shift 2 ;;
    --router)
      ROUTER_ADDR_OVERRIDE="$2"; shift 2 ;;
    --no-router)
      AUTO_START_ROUTER=0; shift ;;
    *)
      echo "Unknown argument: $1"; exit 1 ;;
  esac
done

if [[ -n "$ZENOH_ROLE" ]]; then
  ROLE="$ZENOH_ROLE"
else
  if uname -m | grep -qi "arm\|aarch64"; then
    ROLE="endpoint"
  else
    ROLE="simulation_pc"
  fi
fi

echo "[INFO] Detected role: $ROLE"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USER_HOME="$HOME"

# Ensure configs exist in $HOME/zenoh_ros2dds for easy editing
mkdir -p "$USER_HOME/zenoh_ros2dds"
[[ -f "$USER_HOME/zenoh_ros2dds/bridge-rpi.json5" ]] || cp "$SCRIPT_DIR/bridge-rpi.json5" "$USER_HOME/zenoh_ros2dds/bridge-rpi.json5"
[[ -f "$USER_HOME/zenoh_ros2dds/bridge-pc.json5" ]] || cp "$SCRIPT_DIR/bridge-pc.json5" "$USER_HOME/zenoh_ros2dds/bridge-pc.json5"

update_config_router() {
  local cfg="$1"
  local ep="$2"
  # Replace the first endpoints array with the provided address (simple sed for JSON5 format)
  if [[ -n "$ep" ]]; then
    sed -i -E "s#(connect:\s*\{\s*endpoints:\s*\[)[^\]]*(\]\s*\})#\1\"$ep\"\2#" "$cfg"
  fi
}

if [[ "$ROLE" == "simulation_pc" ]]; then
  # Start zenoh router if requested
  if [[ "$AUTO_START_ROUTER" -eq 1 ]]; then
    if ! command -v zenohd >/dev/null 2>&1; then
      echo "[WARN] zenohd not found in PATH. Install using zenoh_ros2dds/setup_zenoh_router.sh"
    else
      if ! pgrep -x "zenohd" >/dev/null 2>&1; then
        echo "[INFO] Starting zenohd on tcp/0.0.0.0:7447"
        nohup zenohd -l tcp/0.0.0.0:7447 >/tmp/zenohd.log 2>&1 &
        sleep 1
      else
        echo "[INFO] zenohd already running"
      fi
    fi
  fi
  CFG="${CONFIG_OVERRIDE:-$USER_HOME/zenoh_ros2dds/bridge-pc.json5}"
  update_config_router "$CFG" "${ROUTER_ADDR_OVERRIDE:-tcp/localhost:7447}"
  exec ros2dds -c "$CFG"
else
  # endpoint (RPi/Jetson)
  CFG="${CONFIG_OVERRIDE:-$USER_HOME/zenoh_ros2dds/bridge-rpi.json5}"
  if [[ -z "$ROUTER_ADDR_OVERRIDE" ]]; then
    echo "[INFO] Using router from config file (edit connect.endpoints). Use --router to override."
  else
    update_config_router "$CFG" "$ROUTER_ADDR_OVERRIDE"
  fi
  exec ros2dds -c "$CFG"
fi