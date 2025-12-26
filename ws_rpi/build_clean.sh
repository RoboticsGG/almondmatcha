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
echo "  - msgs_ifaces (ChassisCtrl, ChassisIMU, ChassisSensors, SpresenseGNSS, UbloxGNSS, RoverStatus)"
echo "  - action_ifaces (DesData)"
echo "  - services_ifaces (SpdLimit)"
echo "  - pkg_chassis_control"
echo "  - pkg_chassis_sensors"
echo "  - pkg_gnss_navigation (node_gnss_spresense, node_gnss_ublox)"
echo "  - pkg_rover_monitoring (node_rover_monitoring, domain_relay.py)"
echo "  - rover_launch_system"
echo ""
echo "To launch rover nodes:"
echo "  ./launch_rover_tmux.sh"
echo ""
echo "To run domain relay (for Domain 4 monitoring):"
echo "  python3 install/pkg_rover_monitoring/lib/pkg_rover_monitoring/domain_relay.py"
