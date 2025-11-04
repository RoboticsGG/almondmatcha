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
else
    # Try to source from common ROS2 installation paths
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

# Incremental build (does not clean old artefacts)
colcon build --symlink-install

    # Source the newly built setup.bash if it exists
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo "Sourced install/setup.bash."
    else
        echo "Warning: install/setup.bash not found after build."
    fi
echo "Incremental build complete for ws_jetson workspace."
