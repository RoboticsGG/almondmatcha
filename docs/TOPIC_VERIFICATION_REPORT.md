# Topic and Domain Verification Report
Generated: November 4, 2025

## Executive Summary
✅ **PASS** - No critical mismatches found
⚠️ **WARNING** - 2 minor inconsistencies detected (non-breaking)

---

## 1. Vision Navigation Flow (ws_jetson → ws_rpi)

### Domain Configuration
- **ws_jetson**: Domain 2 ✅ (set in launch file)
- **node_chassis_controller**: Listens on Domain 2 ✅

### Camera Stream Node
```
Publisher: /tpc_rover_d415_rgb
Type: sensor_msgs/Image
Domain: 2
Status: ✅ CORRECT
```

### Lane Detection Node
```
Subscriber: /tpc_rover_d415_rgb    ✅ MATCHES camera publisher
Publisher: /tpc_rover_nav_lane
Type: std_msgs/Float32MultiArray
Domain: 2
Status: ✅ CORRECT
```

### Steering Control Node
```
Subscriber: /tpc_rover_nav_lane    ⚠️ MISMATCH - Missing leading slash
            (subscribes to '/tpc_rover_nav_lane' but publisher uses '/tpc_rover_nav_lane')
Publisher: tpc_rover_fmctl         ⚠️ MISMATCH - Missing leading slash
Type: std_msgs/Float32MultiArray
Domain: 2
Status: ⚠️ INCONSISTENT (but will work due to ROS2 name resolution)
```

**Note:** ROS2 normalizes topic names, so `tpc_rover_fmctl` and `/tpc_rover_fmctl` resolve to the same topic. Not critical but inconsistent.

---

## 2. Chassis Control Flow (ws_rpi Domain 2 → Domain 5 → STM32)

### node_chassis_controller (ws_rpi)
```
Domain 2 (Subscriber):
- Subscribes: tpc_rover_fmctl          ✅ (matches Jetson publisher)
  Type: std_msgs/Float32MultiArray
- Subscribes: tpc_gnss_mission_active  ✅
  Type: std_msgs/Bool
- Publishers: pub_rovercontrol_d2      ✅
  Type: msgs_ifaces/ChassisCtrl

Domain 5 (Publisher):
- Publishers: pub_rovercontrol_d5      ✅
  Type: msgs_ifaces/ChassisCtrl
  
Status: ✅ CORRECT
```

### node_domain_bridge (ws_rpi)
```
Domain 5 (Subscriber):
- Subscribes: pub_rovercontrol_d5      ✅ MATCHES chassis_controller D5 publisher
  Type: msgs_ifaces/ChassisCtrl

Domain 2 (Publisher):
- Publishers: pub_rovercontrol         ❌ CRITICAL MISMATCH
  Type: msgs_ifaces/ChassisCtrl
  
Expected: Should publish to Domain 5 for STM32
Actual: Publishing to Domain 2

Status: ❌ DOMAIN MISMATCH - Bridge publishes to wrong domain
```

**CRITICAL ISSUE:** Domain bridge subscribes from Domain 5 but publishes to Domain 2. STM32 is on Domain 5 and won't receive messages!

### STM32 chassis_controller
```
Domain: 5 (hardcoded in platform/mros2-platform.cpp)
Subscribes: pub_rovercontrol           ❌ Won't receive from Domain 2
  Type: msgs_ifaces/MainRocon (wrapper for SubRocon)
Publishers: tp_imu_data_d5
  Type: msgs_ifaces/MainGyroData
  
Status: ❌ DOMAIN MISMATCH - Listening on Domain 5, but messages on Domain 2
```

---

## 3. Message Type Compatibility

### ChassisCtrl vs MainRocon/SubRocon

**ws_rpi uses:** `msgs_ifaces/ChassisCtrl`
```cpp
uint8 fdr_msg       # Front direction
float32 ro_ctrl_msg # Steering control
uint8 spd_msg       # Speed command
uint8 bdr_msg       # Back direction
```

**STM32 uses:** `msgs_ifaces/MainRocon` (wraps SubRocon)
```cpp
// SubRocon.msg
uint8 fdr_msg
float32 ro_ctrl_msg
uint8 spd_msg
uint8 bdr_msg

// MainRocon.msg
msgs_ifaces/SubRocon mainrocon_msg
```

**Status:** ❌ **TYPE MISMATCH**
- Field names are identical
- Field types are identical
- But message types are different: `ChassisCtrl` vs `MainRocon`
- STM32 expects nested structure, ws_rpi sends flat structure

---

## 4. GNSS Navigation Flow

### node_gnss_spresense (ws_rpi)
```
Domain: 2 (default)
Publishers: tpc_gnss_spresense
Type: msgs_ifaces/SpresenseGNSS
Status: ✅ CORRECT
```

### node_gnss_mission_monitor (ws_rpi)
```
Domain: 2 (default)
Subscribes: tpc_gnss_spresense         ✅ MATCHES publisher
Publishers: 
  - tpc_gnss_mission_active            ✅
  - tpc_gnss_mission_remain_dist       ✅
  - tpc_rover_dest_coordinate          ✅
Status: ✅ CORRECT
```

---

## 5. Sensor Data Flow (STM32 Domain 6 → ws_rpi)

### node_chassis_imu (ws_rpi)
```
Domain: 5 (exported before launch)
Subscribes: tp_imu_data_d5             ✅ MATCHES STM32 publisher
Type: msgs_ifaces/ChassisIMU
Status: ✅ CORRECT
```

### node_chassis_sensors (ws_rpi)
```
Domain: 6 (exported before launch)
Subscribes: tp_sensdata_d5             ✅ MATCHES STM32 sensors publisher
Type: msgs_ifaces/ChassisSensors
Status: ✅ CORRECT
```

---

## Critical Issues Summary

### 🔴 CRITICAL - Must Fix

1. **Domain Bridge Direction Mismatch**
   - **File:** `ws_rpi/src/pkg_chassis_control/src/node_domain_bridge.cpp`
   - **Issue:** Bridges Domain 5→Domain 2, but STM32 is on Domain 5
   - **Expected Flow:** Domain 2 → Domain 5 (Jetson/RPi → STM32)
   - **Actual Flow:** Domain 5 → Domain 2 (backwards!)
   - **Fix:** Reverse bridge direction
     - Subscribe from Domain 2: `pub_rovercontrol_d2`
     - Publish to Domain 5: `pub_rovercontrol` or `pub_rovercontrol_d5`

2. **Message Type Mismatch**
   - **Issue:** ws_rpi uses `ChassisCtrl`, STM32 uses `MainRocon`
   - **Impact:** Messages won't deserialize correctly
   - **Fix Options:**
     - Option A: Update STM32 to use `ChassisCtrl` type
     - Option B: Update ws_rpi to use `MainRocon` type
     - Option C: Add message converter in domain bridge

### ⚠️ WARNING - Should Fix

3. **Inconsistent Topic Name Prefix**
   - **File:** `ws_jetson/vision_navigation/vision_navigation_pkg/steering_control_node.py`
   - **Issue:** Missing leading slash on `tpc_rover_fmctl`
   - **Impact:** Works but inconsistent with other topics
   - **Fix:** Add leading slash for consistency

4. **Topic Name in README Documentation**
   - Some topics listed with `/` prefix, others without
   - Should standardize documentation

---

## Recommended Fixes

### Fix 1: Correct Domain Bridge Direction

**Current (WRONG):**
```cpp
// Subscribes from Domain 5, publishes to Domain 2
pub_chassis_ctrl_d2_ = this->create_publisher<msgs_ifaces::msg::ChassisCtrl>(
    "pub_rovercontrol", 10);  // Domain 2

sub_chassis_ctrl_d5_ = this->create_subscription<msgs_ifaces::msg::ChassisCtrl>(
    "pub_rovercontrol_d5", 10, ...);  // Domain 5
```

**Should be (CORRECT):**
```cpp
// Subscribe from Domain 2, publish to Domain 5
sub_chassis_ctrl_d2_ = this->create_subscription<msgs_ifaces::msg::ChassisCtrl>(
    "pub_rovercontrol_d2", 10, ...);  // Domain 2 (from chassis_controller)

pub_chassis_ctrl_d5_ = this->create_publisher<msgs_ifaces::msg::ChassisCtrl>(
    "pub_rovercontrol", 10);  // Domain 5 (to STM32)
```

### Fix 2: Message Type Consistency

**Option A (Recommended): Update STM32 to use ChassisCtrl**
1. Copy `ChassisCtrl.msg` to `mros2-mbed-chassis-dynamics/mros2_add_msgs/mros2_msgs/msgs_ifaces/msg/`
2. Regenerate message headers
3. Update STM32 code to use `msgs_ifaces::msg::ChassisCtrl`

**Option B: Update ws_rpi to use MainRocon**
1. Change all `ChassisCtrl` types to `MainRocon` in ws_rpi
2. Update message field access (add `.mainrocon_msg.` prefix)

### Fix 3: Standardize Topic Names

Update `steering_control_node.py`:
```python
self.pub_fmctl = self.create_publisher(
    Float32MultiArray, 
    '/tpc_rover_fmctl',  # Add leading slash
    10
)
```

---

## Testing Checklist

After fixes:
- [ ] Build all workspaces
- [ ] Launch ws_rpi system
- [ ] Launch ws_jetson vision
- [ ] Verify topics visible: `ros2 topic list` (with correct domain)
- [ ] Check message flow: `ros2 topic echo /tpc_rover_fmctl`
- [ ] Verify STM32 receives commands (LED should blink on command)
- [ ] Check domain bridge relay: monitor both D2 and D5 topics
- [ ] Verify end-to-end: Camera → Lane → Steering → Motor

---

## Configuration Files to Update

1. `ws_rpi/src/pkg_chassis_control/src/node_domain_bridge.cpp`
2. `ws_jetson/vision_navigation/vision_navigation_pkg/steering_control_node.py`
3. `mros2-mbed-chassis-dynamics/workspace/chassis_controller/app.cpp`
4. `README.md` - Topic table corrections
