
#!/bin/bash
# build_clean.sh - Clean and build the entire ws_jetson workspace with symlink install

set -e

# Remove old build, install, and log artefacts
echo "Cleaning previous build artefacts..."
rm -rf build/ install/ log/

# Clear stale AMENT_PREFIX_PATH to avoid warnings from colcon
unset AMENT_PREFIX_PATH

# Build the workspace (clean build, no need to source ROS2 first)
colcon build --symlink-install

# Source the newly built setup.bash if it exists
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Sourced install/setup.bash."
else
    echo "Warning: install/setup.bash not found after build."
fi

# Print completion message
echo "Clean build complete for ws_jetson workspace."
