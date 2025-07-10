#!/bin/bash

# Ensure script is sourced, not executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "❌ This script must be sourced: use 'source ./select_workspace.sh'"
  return 1 2>/dev/null || exit 1
fi

# Source global ROS 2 environment
source /opt/ros/humble/setup.bash

# Prompt user to choose workspace
echo "Select a workspace to activate:"
echo "1) rob_ws"
echo "2) localization_ws"
read -p "Enter number (1 or 2): " choice

if [ "$choice" == "1" ]; then
    WS_PATH=~/ORobotics/ROS2_Control/rob_ws
    echo "Switching to rob_ws..."
elif [ "$choice" == "2" ]; then
    WS_PATH=~/ORobotics/localization_ws
    echo "Switching to localization_ws..."
else
    echo "Invalid choice. Exiting."
    return 1 2>/dev/null || exit 1
fi

cd "$WS_PATH" || { echo "Workspace path not found: $WS_PATH"; return 1 2>/dev/null || exit 1; }

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace activated: $WS_PATH"
    echo "You can now run ros2 commands."
else
    echo "❌ Could not source setup.bash — maybe you need to build first?"
fi
