#!/bin/bash
# build_inc.sh - Incremental build for ws_base workspace with symlink install

set -e

# Source ROS2 — prefer workspace install, fall back to system ROS2.
# If your ROS2 is at a non-standard path, source it before running:
#   source ~/ros2_humble/install/setup.bash && bash build_inc.sh
if [[ -f "install/setup.bash" ]]; then
    source install/setup.bash
elif [[ -f "../install/setup.bash" ]]; then
    source ../install/setup.bash
elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
    source /opt/ros/humble/setup.bash
elif [[ -z "${ROS_DISTRO:-}" ]]; then
    echo ""
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble/ and not already sourced."
    echo "Source your ROS2 installation before running this script:"
    echo "  source <ros2_path>/install/setup.bash && bash build_inc.sh"
    echo ""
    exit 1
fi

# Incremental build — interfaces first so CMake can find them
echo "Incremental build for ws_base workspace..."
colcon build --symlink-install --packages-select action_ifaces msgs_ifaces services_ifaces

# Re-source so mission_control can find the interfaces
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

colcon build --symlink-install --packages-select mission_control

# Source the newly built setup.bash if it exists
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Sourced install/setup.bash."
else
    echo "Warning: install/setup.bash not found after build."
fi

echo ""
echo "Incremental build complete for ws_base workspace."
echo ""
echo "Modified packages have been rebuilt."
