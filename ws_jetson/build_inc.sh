#!/bin/bash
# build_inc.sh - Incremental build for ws_jetson workspace with symlink install

set -e

# Source ROS2 setup if available (check multiple locations)
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
elif [ -f "../install/setup.bash" ]; then
    source ../install/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source common interface packages (msgs_ifaces, action_ifaces, services_ifaces)
COMMON_IFACES="$(dirname "$0")/../common_ifaces/install/setup.bash"
if [ -f "$COMMON_IFACES" ]; then
    source "$COMMON_IFACES"
else
    echo "ERROR: common_ifaces not built. Run 'colcon build' in common_ifaces/ first."
    exit 1
fi

# Incremental build (does not clean old artefacts)
colcon build --symlink-install
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Sourced install/setup.bash."
else
    echo "Warning: install/setup.bash not found after build."
fi
echo "Incremental build complete for ws_jetson workspace."
