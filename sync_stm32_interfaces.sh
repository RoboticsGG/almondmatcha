#!/bin/bash
# sync_stm32_interfaces.sh
# Synchronizes message definitions from common_ifaces to STM32 workspaces
# Run this script before building STM32 firmware

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMMON_IFACES="$SCRIPT_DIR/common_ifaces/msgs_ifaces/msg"
STM32_CHASSIS="$SCRIPT_DIR/mros2-mbed-chassis-dynamics/mros2_add_msgs/mros2_msgs/msgs_ifaces/msg"
STM32_SENSORS="$SCRIPT_DIR/mros2-mbed-sensors-gnss/mros2_add_msgs/mros2_msgs/msgs_ifaces/msg"

echo "==================================="
echo "STM32 Interface Sync"
echo "==================================="
echo "Source: common_ifaces/"
echo ""

# Sync STM32 Chassis Dynamics
echo "[1/2] Syncing STM32 Chassis Dynamics messages..."
cp -v "$COMMON_IFACES/ChassisCtrl.msg" "$STM32_CHASSIS/"
cp -v "$COMMON_IFACES/ChassisIMU.msg" "$STM32_CHASSIS/"
echo "  ✓ ChassisCtrl.msg, ChassisIMU.msg copied"

# Sync STM32 Sensors GNSS
echo ""
echo "[2/2] Syncing STM32 Sensors GNSS messages..."
cp -v "$COMMON_IFACES/ChassisSensors.msg" "$STM32_SENSORS/"
echo "  ✓ ChassisSensors.msg copied"

echo ""
echo "==================================="
echo "✓ Sync complete!"
echo "==================================="
echo ""
echo "Next steps:"
echo "  1. Build STM32 Chassis: cd mros2-mbed-chassis-dynamics && ./build.bash"
echo "  2. Build STM32 Sensors: cd mros2-mbed-sensors-gnss && ./build.bash"
echo ""
