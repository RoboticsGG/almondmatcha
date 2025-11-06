# Domain Architecture Consolidation Summary

**Date:** 2025-11-01  
**Change Type:** Major Architectural Refactoring  
**Status:** ✅ Complete

---

## Overview

Successfully refactored the rover system from a **two-domain architecture (Domain 5 + Domain 6)** to a **single-domain architecture (Domain 5 only)**, while maintaining the Domain 2 bridge for ws_base communication.

### Motivation

The original multi-domain separation (5 and 6) was based on an incorrect understanding of mROS2/embeddedRTPS limitations. After analyzing the source code, we determined:

- **Previous Assumption (Wrong):** Each STM32 board needs its own domain ID
- **Actual Limitation:** Each STM32 board has a resource limit of `MAX_NUM_PARTICIPANTS=10` (not domain-specific)
- **Conclusion:** Domain separation was unnecessary and added complexity without benefit

---

## Architecture Before vs After

### Before: Two-Domain Architecture
```
Domain 2: ws_base ↔ ws_rpi bridge
  ├─ node_gnss_spresense
  ├─ node_gnss_mission_monitor
  └─ node_chassis_controller

Domain 5: Chassis Dynamics STM32
  ├─ STM32 mros2-mbed-chassis-dynamics
  ├─ node_chassis_imu (ROS2 subscriber)
  └─ node_domain_bridge (D5 ↔ D2 bridge)

Domain 6: Sensors STM32
  ├─ STM32 mros2-mbed-sensors-gnss
  └─ node_chassis_sensors (ROS2 subscriber)
```

### After: Single-Domain Architecture
```
Domain 2: ws_base ↔ ws_rpi communication
  ├─ node_gnss_spresense
  ├─ node_gnss_mission_monitor
  └─ node_chassis_controller

Domain 5: All STM32 boards + sensors
  ├─ STM32 mros2-mbed-chassis-dynamics
  ├─ STM32 mros2-mbed-sensors-gnss
  ├─ node_chassis_imu (ROS2 subscriber)
  └─ node_chassis_sensors (ROS2 subscriber)
```

---

## Changes Made

### 1. STM32 Sensors Board (mros2-mbed-sensors-gnss)

**File:** `mros2-mbed-sensors-gnss/workspace/sensors_node/app.cpp`
- **Line 162:** Changed `setenv("ROS_DOMAIN_ID", "6", 6)` → `setenv("ROS_DOMAIN_ID", "5", 5)`
- **Line 174:** Updated log message from "Domain 6" → "Domain 5"

**File:** `mros2-mbed-sensors-gnss/platform/rtps/config.h`
- **Line 48:** Changed `const uint8_t DOMAIN_ID = 6` → `const uint8_t DOMAIN_ID = 5`

### 2. ROS2 Node Chassis Controller (node_chassis_controller.cpp)

**File:** `ws_rpi/src/pkg_chassis_control/src/node_chassis_controller.cpp`

**Removed:**
- Domain 5 context, node, publisher, and executor
- Multi-threaded executor for multi-domain communication
- `initializeSubscriberDomain()` and `initializePublisherDomain()` methods
- `pub_rocon_d5_` publisher

**Simplified:**
- Now operates entirely in Domain 2
- Single publisher: `pub_rocon_d2_` (publishes to `tpc_chassis_ctrl_d2`)
- All subscribers and services in main node context
- Removed multi-domain bridge complexity

### 3. Domain Bridge Node Removal

**File:** `ws_rpi/src/pkg_chassis_control/src/node_domain_bridge.cpp`
- **Status:** ❌ **DELETED** (no longer needed)

**File:** `ws_rpi/src/pkg_chassis_control/CMakeLists.txt`
- Removed domain bridge build target and executable configuration

### 4. Launch Configuration Updates

**File:** `ws_rpi/src/rover_launch_system/launch/rover_startup.launch.py`

**Changes:**
- Merged `domain_5_group` and `domain_6_group` into single `domain_5_group`
- Removed `node_domain_bridge` launch configuration
- Both `node_chassis_imu` and `node_chassis_sensors` now in Domain 5 group
- Updated return statement to include only `domain_2_group` and `domain_5_group`

### 5. Tmux Launch Script Updates

**File:** `ws_rpi/launch_rover_tmux.sh`

**Changes:**
- Updated header: "3x2 Grid Layout (5 nodes)" (was 6 nodes)
- Pane 4: Changed from "Domain Bridge (D5→D2)" to "Chassis Sensors (Domain 5)"
- Pane 5: Changed from "Chassis Sensors (Domain 6)" to "Monitor (Reserved)"
- Removed Domain 6 export commands
- Updated all display messages for consistency

---

## Build Results

All packages rebuilt successfully:

```bash
✅ pkg_chassis_control   [1min 11s]
✅ pkg_chassis_sensors   [18.5s]
✅ rover_launch_system   [10.7s]
```

**Errors:** None  
**Warnings:** None (except colcon marker warning, non-critical)

---

## Testing Checklist

Before deploying to rover:

- [ ] Flash updated firmware to STM32 sensors board (mros2-mbed-sensors-gnss)
- [ ] Verify STM32 chassis board still on Domain 5
- [ ] Test `ros2 topic list` on Domain 5 shows both `/tpc_chassis_imu` and `/tpc_chassis_sensors`
- [ ] Test `node_chassis_imu` receives IMU data
- [ ] Test `node_chassis_sensors` receives encoder/power/GNSS data
- [ ] Test `node_chassis_controller` publishes to Domain 2 correctly
- [ ] Verify ws_base ↔ ws_rpi communication still works
- [ ] Test full rover launch with tmux script
- [ ] Monitor participant count on Domain 5 (should be well under 10 per board)

---

## Participant Count Analysis

### Domain 5 Participants (Per STM32 Board):

**STM32 Chassis Dynamics:**
- 1x Publisher: `tpc_chassis_imu`
- 1x Subscriber: `tpc_chassis_cmd`
- **Total:** 2 participants (well under MAX_NUM_PARTICIPANTS=10)

**STM32 Sensors:**
- 1x Publisher: `tpc_chassis_sensors`
- **Total:** 1 participant (well under MAX_NUM_PARTICIPANTS=10)

**ROS2 Nodes on Raspberry Pi:**
- node_chassis_imu (1 subscriber)
- node_chassis_sensors (1 subscriber)
- **Total:** 2 ROS2 participants

**Conclusion:** Combined total is 5 participants across Domain 5, with each STM32 board managing only 1-2 participants. This is well within the `MAX_NUM_PARTICIPANTS=10` limit per board.

---

## Benefits of Consolidation

1. **Reduced Complexity:** No need for domain bridge node
2. **Fewer Nodes:** 5 active nodes instead of 6
3. **Cleaner Architecture:** Logical separation between rover (Domain 5) and base (Domain 2)
4. **Easier Debugging:** Fewer moving parts to troubleshoot
5. **Better Performance:** Eliminates bridge relay overhead
6. **Correct Understanding:** Architecture now matches actual mROS2 limitations

---

## File Changes Summary

### Modified Files (7):
1. `mros2-mbed-sensors-gnss/workspace/sensors_node/app.cpp`
2. `mros2-mbed-sensors-gnss/platform/rtps/config.h`
3. `ws_rpi/src/pkg_chassis_control/src/node_chassis_controller.cpp`
4. `ws_rpi/src/pkg_chassis_control/CMakeLists.txt`
5. `ws_rpi/src/rover_launch_system/launch/rover_startup.launch.py`
6. `ws_rpi/launch_rover_tmux.sh`

### Deleted Files (1):
1. `ws_rpi/src/pkg_chassis_control/src/node_domain_bridge.cpp`

---

## Next Steps

1. Commit and push changes to repository
2. Flash updated STM32 firmware to sensors board
3. Deploy to rover and test system integration
4. Update README.md with new domain architecture
5. Update system diagrams/documentation

---

## Rollback Plan (If Needed)

If issues arise:
1. Revert git commits
2. Re-flash STM32 boards with Domain 6 firmware
3. Rebuild packages with old configuration
4. Restore `node_domain_bridge.cpp` from git history

---

**Refactored by:** GitHub Copilot  
**Reviewed by:** [Pending]  
**Deployed:** [Pending]
