#!/bin/bash

##############################################
#   User Configurable Parameters
##############################################
# ROS 2 version & global setup file
ROS_DISTRO="humble"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

# Workspace options (name → path)
declare -A WORKSPACES
WORKSPACES["rob_ws"]="${HOME}/ORobotics/ROS2_Control/rob_ws"
WORKSPACES["localization_ws"]="${HOME}/ORobotics/localization_ws"

##############################################
#   Script Logic
##############################################

# Ensure script is sourced, not executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced: use 'source ./select_workspace.sh'"
  return 1 2>/dev/null || exit 1
fi

# Check ROS global setup file
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}"
  return 1 2>/dev/null || exit 1
fi

# Source global ROS 2 environment
source "${ROS_SETUP}"

# Prompt user to choose workspace
echo "Select a workspace to activate:"
i=1
for ws_name in "${!WORKSPACES[@]}"; do
    echo "${i}) ${ws_name}"
    ((i++))
done

read -p "Enter number: " choice

# Get workspace name by index
ws_name=$(echo "${!WORKSPACES[@]}" | tr ' ' '\n' | sed -n "${choice}p")

if [[ -z "${ws_name}" ]]; then
    echo "Invalid choice."
    return 1 2>/dev/null || exit 1
fi

WS_PATH="${WORKSPACES[$ws_name]}"
echo "Switching to ${ws_name}..."

cd "$WS_PATH" || { echo "Workspace path not found: $WS_PATH"; return 1 2>/dev/null || exit 1; }

if [[ -f "install/setup.bash" ]]; then
    source install/setup.bash
    echo "Workspace activated: $WS_PATH"
    echo "You can now run ros2 commands."
else
    echo "⚠ Could not source setup.bash — maybe you need to build first?"
fi
