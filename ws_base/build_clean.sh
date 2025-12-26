#!/bin/bash
# build_clean.sh - Clean and build the entire ws_base workspace with symlink install

set -e

# Remove old build, install, and log artefacts
echo "Cleaning previous build artefacts..."
rm -rf build/ install/ log/

# Clear stale AMENT_PREFIX_PATH to avoid warnings from colcon
unset AMENT_PREFIX_PATH

# Source ROS2 setup from standard location if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Build the workspace
echo "Building ws_base workspace..."
colcon build --symlink-install

# Source the newly built setup.bash if it exists
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Sourced install/setup.bash."
fi

echo ""
echo "======================================"
echo "Clean build complete for ws_base"
echo "======================================"
echo ""
echo "Packages built:"
echo "  - msgs_ifaces (ChassisCtrl, ChassisIMU, ChassisSensors, SpresenseGNSS, UbloxGNSS, RoverStatus)"
echo "  - action_ifaces (DesData)"
echo "  - services_ifaces (SpdLimit)"
echo "  - mission_control (mission_command_node, base_monitoring_node)"
echo ""
echo "To run base station nodes:"
echo "  export ROS_DOMAIN_ID=4  # For monitoring (receives from Domain 4)"
echo "  ros2 run mission_control base_monitoring_node"
echo ""
echo "  export ROS_DOMAIN_ID=5  # For commands (sends to Domain 5)"
echo "  ros2 run mission_control mission_command_node"
