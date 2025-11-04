#!/bin/bash
# build_inc.sh - Incremental build for ws_jetson workspace with symlink install

set -e

# Source ROS2 setup if available
if [ -f "../install/setup.bash" ]; then
    source ../install/setup.bash
elif [ -f "../common_ifaces/install/setup.bash" ]; then
    source ../common_ifaces/install/setup.bash
else
    echo "Warning: No setup.bash found. Building without sourcing ROS2 environment."
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
