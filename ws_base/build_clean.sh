#!/bin/bash
# build_clean.sh - Clean and build the entire ws_base workspace with symlink install

set -e

# Remove old build, install, and log artefacts
echo "Cleaning previous build artefacts..."
rm -rf build/ install/ log/

# Clear stale AMENT_PREFIX_PATH to avoid warnings from colcon
unset AMENT_PREFIX_PATH

# Source ROS2 Humble — try standard debian path; skip if already sourced.
# If your ROS2 is installed at a non-standard path (e.g. built from source),
# source it BEFORE running this script:
#   source ~/ros2_humble/install/setup.bash && bash build_clean.sh
if [[ -n "${ROS_DISTRO:-}" ]]; then
    : # already sourced by caller
elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
    source /opt/ros/humble/setup.bash
else
    echo ""
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble/ and not already sourced."
    echo "Source your ROS2 installation before running this script:"
    echo "  source <ros2_path>/install/setup.bash && bash build_clean.sh"
    echo "  # e.g. for a source build: source ~/ros2_humble/install/setup.bash && bash build_clean.sh"
    echo ""
    exit 1
fi

# Step 1: Build interface packages first
echo "Step 1/2: Building interface packages..."
colcon build --symlink-install --packages-select action_ifaces msgs_ifaces services_ifaces

# Source so CMake can find interface packages in step 2
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# Step 2: Build application packages
echo "Step 2/2: Building application packages..."
colcon build --symlink-install --packages-select mission_control

# Re-source the final setup
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
