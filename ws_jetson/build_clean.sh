
#!/bin/bash
# build_clean.sh - Clean and build the entire ws_jetson workspace with symlink install

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

# Source common interface packages (msgs_ifaces, action_ifaces, services_ifaces)
COMMON_IFACES="$(dirname "$0")/../common_ifaces/install/setup.bash"
if [ -f "$COMMON_IFACES" ]; then
    source "$COMMON_IFACES"
else
    echo "ERROR: common_ifaces not built. Run 'colcon build' in common_ifaces/ first."
    exit 1
fi

# Build the workspace
colcon build --symlink-install
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Sourced install/setup.bash."
fi

echo "Clean build complete for ws_jetson workspace."
