#!/bin/bash
# build_clean.sh - Clean and build the entire ws_rpi workspace with symlink install

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
echo "Building ws_rpi workspace..."
colcon build --symlink-install

# Source the newly built setup.bash if it exists
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Sourced install/setup.bash."
fi

echo ""
echo "======================================"
echo "Clean build complete for ws_rpi"
echo "======================================"
echo ""
echo "Packages built:"
echo "  - msgs_ifaces (ChassisCtrl, ChassisIMU, ChassisSensors, SpresenseGNSS, UbloxGNSS, TelemetryRelay)"
echo "  - action_ifaces (DesData)"
echo "  - services_ifaces (SpdLimit)"
echo "  - chassis_control"
echo "  - chassis_sensors"
echo "  - gnss_navigation (node_gnss_spresense, node_gnss_ublox)"
echo "  - rover_monitoring (mission_monitoring_node_rpi, node_rover_monitoring)"
echo "  - rover_bringup"
echo ""
echo "To launch rover nodes:"
echo "  ./launch_rover_tmux.sh"
echo ""
echo "Mission monitoring (publishes telemetry relay to base station):"
echo "  ros2 run rover_monitoring mission_monitoring_node_rpi"
