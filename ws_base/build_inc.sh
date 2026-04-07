#!/bin/bash
# build_inc.sh - Incremental build for ws_base workspace with symlink install

set -e

# Source ROS2 base
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source common_ifaces for msgs_ifaces (it is NOT built in ws_base to avoid duplicate .so files)
COMMON_IFACES="$(cd "$(dirname "$0")/.." && pwd)/common_ifaces/install/setup.bash"
if [ ! -f "$COMMON_IFACES" ]; then
    echo "ERROR: common_ifaces not built."
    exit 1
fi
source "$COMMON_IFACES"

# Source existing ws_base install for incremental builds
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# Incremental build — action_ifaces and services_ifaces (msgs_ifaces is from common_ifaces)
echo "Incremental build for ws_base workspace..."
colcon build --symlink-install --packages-select action_ifaces services_ifaces

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
