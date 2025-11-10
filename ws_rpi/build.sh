#!/bin/bash
# ROS2 Workspace Build Script for ws_rpi
# This script handles the proper build order for all packages

# ------------------------------------------------------------------------------
# How to build ws_rpi (Raspberry Pi ROS 2 workspace)
#
# 1. Make sure you are in the ws_rpi directory.
# 2. Run this script:
#      ./build.sh
#    - To clean before building, run:
#      ./build.sh clean
# 3. The script will:
#    - Build interface packages first (action_ifaces, msgs_ifaces, services_ifaces)
#    - Source the install/setup.bash
#    - Build application packages (pkg_chassis_control, pkg_chassis_sensors, pkg_gnss_navigation, rover_launch_system)
# 4. After a successful build, source install/setup.bash before running nodes.
# ------------------------------------------------------------------------------

set -e  # Exit on error

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Building ws_rpi ROS2 Workspace${NC}"
echo -e "${GREEN}========================================${NC}"

# Navigate to workspace directory
cd "$(dirname "$0")"

# Option to clean build
if [ "$1" == "clean" ]; then
    echo -e "${YELLOW}Cleaning build, install, and log directories...${NC}"
    rm -rf build install log
    echo -e "${GREEN}Clean complete!${NC}"
fi

# Build interface packages first (they are dependencies for other packages)
echo -e "\n${YELLOW}Step 1/2: Building interface packages...${NC}"
colcon build --packages-select action_ifaces msgs_ifaces services_ifaces

# Source the setup file to make interfaces available
echo -e "\n${YELLOW}Sourcing install/setup.bash...${NC}"
source install/setup.bash

# Build application packages
echo -e "\n${YELLOW}Step 2/2: Building application packages...${NC}"
colcon build --packages-select \
    pkg_chassis_control \
    pkg_chassis_sensors \
    pkg_gnss_navigation \
    rover_launch_system

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}Build Complete!${NC}"
echo -e "${GREEN}========================================${NC}"

# List all packages
echo -e "\n${YELLOW}Built packages:${NC}"
source install/setup.bash
colcon list

echo -e "\n${YELLOW}To use this workspace, run:${NC}"
echo -e "  ${GREEN}source install/setup.bash${NC}"
