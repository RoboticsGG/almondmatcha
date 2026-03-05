#!/bin/bash
# build_inc.sh - Incremental build for ws_rpi workspace with symlink install

set -e

cd "$(dirname "$0")"

# Source ROS2 base installation first
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: /opt/ros/humble/setup.bash not found"
    exit 1
fi

# Step 1: interfaces (in case any changed)
echo "Step 1/2: Building interface packages..."
colcon build --symlink-install --packages-select action_ifaces msgs_ifaces services_ifaces
source install/setup.bash

# Step 2: application packages
echo "Step 2/2: Building application packages..."
colcon build --symlink-install --packages-select \
    chassis_control \
    chassis_sensors \
    gnss_navigation \
    rover_monitoring \
    rover_bringup

source install/setup.bash

echo ""
echo "Incremental build complete for ws_rpi workspace."
echo "Run './launch_rover_tmux.sh' to launch rover nodes."
