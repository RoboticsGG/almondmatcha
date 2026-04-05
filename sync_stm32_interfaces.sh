#!/bin/bash
# sync_stm32_interfaces.sh
# Synchronizes message definitions from common_ifaces to STM32 workspaces.
# Run this script before building STM32 firmware whenever a message definition changes.
#
# Two-stage sync:
#   Stage 1: .msg files  → mros2_add_msgs/  (human-readable source of truth)
#   Stage 2: .hpp files  → mros2/mros2_msgs/ (compiled by the STM32 build system)
#
# The build system includes mros2/mros2_msgs/ (via mros2/CMakeLists.txt BEFORE INTERFACE)
# and NOT mros2_add_msgs/. Stage 2 keeps them identical so there is no divergence.

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMMON_IFACES="$SCRIPT_DIR/common_ifaces/msgs_ifaces/msg"

CHASSIS_ADD="$SCRIPT_DIR/mros2-mbed-chassis-dynamics/mros2_add_msgs/mros2_msgs/msgs_ifaces/msg"
CHASSIS_MROS2="$SCRIPT_DIR/mros2-mbed-chassis-dynamics/mros2/mros2_msgs/msgs_ifaces/msg"

SENSORS_ADD="$SCRIPT_DIR/mros2-mbed-sensors-gnss/mros2_add_msgs/mros2_msgs/msgs_ifaces/msg"
SENSORS_MROS2="$SCRIPT_DIR/mros2-mbed-sensors-gnss/mros2/mros2_msgs/msgs_ifaces/msg"

echo "==================================="
echo "STM32 Interface Sync"
echo "==================================="
echo "Source: common_ifaces/"
echo ""

# ── Stage 1: .msg files → mros2_add_msgs/ ────────────────────────────────────
echo "[1/4] Syncing chassis-dynamics .msg files..."
cp -v "$COMMON_IFACES/ChassisCtrl.msg"    "$CHASSIS_ADD/"
cp -v "$COMMON_IFACES/ChassisIMU.msg"     "$CHASSIS_ADD/"

echo ""
echo "[2/4] Syncing sensors-gnss .msg files..."
cp -v "$COMMON_IFACES/ChassisSensors.msg" "$SENSORS_ADD/"

# ── Stage 2: .hpp files → mros2/mros2_msgs/ ──────────────────────────────────
# The .hpp files in mros2_add_msgs/ are the hand-authored C++ serialization headers
# that match the .msg definitions above.  They must be mirrored into mros2/mros2_msgs/
# because that directory is the one on the compiler include path.
echo ""
echo "[3/4] Mirroring chassis-dynamics .hpp files into mros2/mros2_msgs/..."
for hpp in "$CHASSIS_ADD"/*.hpp; do
    cp -v "$hpp" "$CHASSIS_MROS2/"
done

echo ""
echo "[4/4] Mirroring sensors-gnss .hpp files into mros2/mros2_msgs/..."
for hpp in "$SENSORS_ADD"/*.hpp; do
    cp -v "$hpp" "$SENSORS_MROS2/"
done

echo ""
echo "==================================="
echo "✓ Sync complete!"
echo "==================================="
echo ""
echo "Next steps:"
echo "  1. Build STM32 Chassis: cd mros2-mbed-chassis-dynamics && sudo ./build.bash all"
echo "  2. Build STM32 Sensors: cd mros2-mbed-sensors-gnss   && sudo ./build.bash all"
echo ""
