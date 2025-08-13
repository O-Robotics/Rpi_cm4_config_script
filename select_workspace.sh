#!/bin/bash

##############################################
#   User Configurable Parameters
##############################################
# ROS 2 version & global setup file
ROS_DISTRO="humble"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

# Base search directory for workspaces
WORKSPACE_BASE="$(pwd)" # This will search all workspace that have /src, you can also change the path to your own

##############################################
#   Script Logic
##############################################

# Ensure script is sourced, not executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo " This script must be sourced: use 'source ./select_workspace.sh'"
  return 1 2>/dev/null || exit 1
fi

# Check ROS global setup file
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo " ROS setup file not found: ${ROS_SETUP}"
  return 1 2>/dev/null || exit 1
fi

# Source global ROS 2 environment
source "${ROS_SETUP}"

# Auto-detect workspaces: any directory with a 'src/' subfolder
declare -A WORKSPACES
while IFS= read -r -d '' dir; do
    ws_name=$(basename "$dir")
    WORKSPACES["$ws_name"]="$dir"
done < <(find "$WORKSPACE_BASE" -mindepth 1 -maxdepth 1 -type d -exec test -d "{}/src" \; -print0)

# Check if we found any workspaces
if [[ ${#WORKSPACES[@]} -eq 0 ]]; then
    echo "⚠ No ROS 2 workspaces found under $WORKSPACE_BASE"
    return 1 2>/dev/null || exit 1
fi

# Create ordered list of workspace names
workspace_names=("${!WORKSPACES[@]}")
IFS=$'\n' workspace_names=($(sort <<<"${workspace_names[*]}"))
unset IFS

# Prompt user
echo "Select a workspace to activate:"
for i in "${!workspace_names[@]}"; do
    echo "$((i+1))) ${workspace_names[$i]}"
done

read -p "Enter number: " choice
if ! [[ "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#workspace_names[@]} )); then
    echo " Invalid choice."
    return 1 2>/dev/null || exit 1
fi

# Get workspace name and path
ws_name="${workspace_names[$((choice-1))]}"
WS_PATH="${WORKSPACES[$ws_name]}"

echo "Switching to ${ws_name}..."
cd "$WS_PATH" || { echo " Workspace path not found: $WS_PATH"; return 1 2>/dev/null || exit 1; }

if [[ -f "install/setup.bash" ]]; then
    source install/setup.bash
    echo " Workspace activated: $WS_PATH"
    echo "You can now run ros2 commands."
else
    echo "⚠ Could not source setup.bash — maybe you need to build first?"
fi
