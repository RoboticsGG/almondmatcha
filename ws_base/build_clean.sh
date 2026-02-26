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
echo "  - msgs_ifaces (ChassisCtrl, ChassisIMU, ChassisSensors, SpresenseGNSS, UbloxGNSS, TelemetryRelay)"
echo "  - action_ifaces (DesData)"
echo "  - services_ifaces (SpdLimit)"
echo "  - mission_control (mission_command_node, mission_monitoring_node_pc)"
echo ""
echo "To run base station nodes:"
echo "  # Mission monitoring (Domain 4 - telemetry display)"
echo "  export ROS_DOMAIN_ID=4"
echo "  ros2 run mission_control mission_monitoring_node_pc  # Subscribes to /tpc_telemetry_relay"
echo ""
echo "  # Mission command (Domain 5 - rover control)"
echo "  export ROS_DOMAIN_ID=5"
echo "  ros2 run mission_control mission_command_node  # Publishes mission commands"
echo ""
echo "Note: Cross-domain architecture reduces Domain 5 participant count for STM32 memory optimization"
