# Refactoring Changelog - Message Types & Topic Names

**Date**: 2025-11-04  
**Type**: Message Type Unification & Topic Naming Standardization  
**Scope**: STM32 Chassis Controller + STM32 Sensors + ws_rpi Chassis Control Nodes

## Overview

This refactoring addresses two maintainability issues:

1. **Message Type Inconsistency**: Replaced deprecated nested structures (`MainRocon/SubRocon`, `MainGyroData/SubGyroData`, `MainSensData/SubSensData`) with flat refactored structures (`ChassisCtrl`, `ChassisIMU`, `ChassisSensors`)
2. **Topic Naming Standardization**: Renamed all topics to `tpc_<subsystem>_<data_type>` convention for consistency

## Variable Naming Review

**Current naming conventions:**
- ✅ **Kept existing names**: `fdr_msg`, `ro_ctrl_msg`, `spd_msg`, `bdr_msg` (ChassisCtrl)
- ✅ **Kept existing names**: `accel_x`, `gyro_x`, etc. (ChassisIMU)  
- ✅ **Kept existing names**: `mt_lf_encode_msg`, `sys_current_msg`, etc. (ChassisSensors)

**Rationale**: While some abbreviations exist (`fdr`, `bdr`, `mt_lf`), the comments clearly explain meaning. Changing would require extensive codebase refactoring. Consistency maintained within each message type.

## Message Type Changes

### Before: Nested Structures (Deprecated)

#### MainRocon/SubRocon (Chassis Control)
```cpp
msgs_ifaces::msg::MainRocon
  └── mainrocon_msg (SubRocon)
      ├── fdr_msg
      ├── ro_ctrl_msg
      ├── spd_msg
      └── bdr_msg
```

#### MainGyroData/SubGyroData (IMU)
```cpp
msgs_ifaces::msg::MainGyroData
  └── maingyrodata_msg (SubGyroData)
      ├── accel_x, accel_y, accel_z
      └── gyro_x, gyro_y, gyro_z
```

#### MainSensData/SubSensData (Sensors)
```cpp
msgs_ifaces::msg::MainSensData
  └── mainsensdata_msg (SubSensData)
      ├── mt_lf_encode_msg
      ├── mt_rt_encode_msg
      ├── sys_current_msg
      └── sys_volt_msg
```

### After: Flat Structures (Refactored)

#### ChassisCtrl
```cpp
msgs_ifaces::msg::ChassisCtrl
  ├── fdr_msg       (uint8)    # Front direction: 1=right, 2=straight, 3=left
  ├── ro_ctrl_msg   (float32)  # Steering control (0.0 to 1.0)
  ├── spd_msg       (uint8)    # Speed command (0-255)
  └── bdr_msg       (uint8)    # Back direction: 0=stop, 1=forward, 2=backward
```

#### ChassisIMU
```cpp
msgs_ifaces::msg::ChassisIMU
  ├── accel_x, accel_y, accel_z  (int32)  # Acceleration (m/s^2 * 1000)
  └── gyro_x, gyro_y, gyro_z     (int32)  # Angular velocity (rad/s * 1000)
```

#### ChassisSensors
```cpp
msgs_ifaces::msg::ChassisSensors
  ├── mt_lf_encode_msg  (int32)    # Left motor encoder
  ├── mt_rt_encode_msg  (int32)    # Right motor encoder
  ├── sys_current_msg   (float32)  # Battery current (A)
  └── sys_volt_msg      (float32)  # Battery voltage (V)
```

## Files Modified

### 1. STM32 Chassis Controller (mros2-mbed-chassis-dynamics)

#### Message Files
- ✅ **Created**: `ChassisCtrl.msg` (renamed from chassis_ctrl.msg)
- ✅ **Created**: `ChassisIMU.msg` (replaces MainGyroData/SubGyroData)
- ❌ **Removed**: `MainRocon.msg`, `SubRocon.msg`, `MainGyroData.msg`, `SubGyroData.msg`

#### `workspace/chassis_controller/app.cpp`
- **Line ~25**: Changed includes
  - Removed: `msgs_ifaces/msg/main_rocon.hpp`, `msgs_ifaces/msg/sub_rocon.hpp`, `msgs_ifaces/msg/main_gyro_data.hpp`, `msgs_ifaces/msg/sub_gyro_data.hpp`
  - Added: `msgs_ifaces/msg/chassis_ctrl.hpp`, `msgs_ifaces/msg/chassis_imu.hpp`
  
- **Line ~140-150**: Updated IMU message creation (flat structure)
  ```cpp
  // Before (nested)
  msgs_ifaces::msg::SubGyroData sub_msg;
  sub_msg.accel_x = local_accel[0];
  // ...
  msgs_ifaces::msg::MainGyroData main_msg;
  main_msg.maingyrodata_msg = sub_msg;
  imu_pub_ptr->publish(main_msg);
  
  // After (flat)
  msgs_ifaces::msg::ChassisIMU imu_msg;
  imu_msg.accel_x = local_accel[0];
  // ...
  imu_pub_ptr->publish(imu_msg);
  ```

- **Line ~169**: Updated callback signature
  ```cpp
  // Before
  void rover_control_callback(msgs_ifaces::msg::MainRocon* msg)
  
  // After
  void rover_control_callback(msgs_ifaces::msg::ChassisCtrl* msg)
  ```

- **Lines ~176-179**: Changed field access pattern
  ```cpp
  // Before (nested access)
  rover_cmd.front_direction = msg->mainrocon_msg.fdr_msg;
  
  // After (direct access)
  rover_cmd.front_direction = msg->fdr_msg;
  ```

- **Line ~218**: Updated subscription type and topic
  ```cpp
  // Before
  create_subscription<msgs_ifaces::msg::MainRocon>("pub_rovercontrol", ...)
  
  // After
  create_subscription<msgs_ifaces::msg::ChassisCtrl>("tpc_chassis_cmd", ...)
  ```

- **Line ~221**: Updated publisher type and topic
  ```cpp
  // Before
  create_publisher<msgs_ifaces::msg::MainGyroData>("tp_imu_data_d5", ...)
  
  // After
  create_publisher<msgs_ifaces::msg::ChassisIMU>("tpc_chassis_imu", ...)
  ```

### 2. STM32 Sensors Controller (mros2-mbed-sensors-gnss)

#### Message Files
- ✅ **Created**: `ChassisSensors.msg` (replaces MainSensData/SubSensData)
- ❌ **Removed**: `MainRocon.msg`, `SubRocon.msg`, `MainSensData.msg`, `SubSensData.msg`

#### `workspace/sensors_node/app.cpp`
- **Line ~23**: Changed includes
  ```cpp
  // Before
  #include "msgs_ifaces/msg/main_sens_data.hpp"
  
  // After
  #include "msgs_ifaces/msg/chassis_sensors.hpp"
  ```

- **Line ~190**: Updated publisher type and topic
  ```cpp
  // Before
  create_publisher<msgs_ifaces::msg::MainSensData>("tp_sensdata_d6", ...)
  
  // After
  create_publisher<msgs_ifaces::msg::ChassisSensors>("tpc_chassis_sensors", ...)
  ```

- **Lines ~242-247**: Changed field access pattern
  ```cpp
  // Before (nested)
  msgs_ifaces::msg::MainSensData msgs;
  msgs.mainsensdata_msg.mt_lf_encode_msg = enc_A;
  msgs.mainsensdata_msg.mt_rt_encode_msg = enc_B;
  msgs.mainsensdata_msg.sys_current_msg = curr;
  msgs.mainsensdata_msg.sys_volt_msg = vbus;
  
  // After (flat)
  msgs_ifaces::msg::ChassisSensors msgs;
  msgs.mt_lf_encode_msg = enc_A;
  msgs.mt_rt_encode_msg = enc_B;
  msgs.sys_current_msg = curr;
  msgs.sys_volt_msg = vbus;
  ```

### 3. ws_rpi Chassis Controller (ws_rpi)

#### `src/pkg_chassis_control/src/node_chassis_controller.cpp`
- **Line ~126**: Renamed D2 publisher topic
  ```cpp
  // Before
  "pub_rovercontrol_d2"
  
  // After
  "tpc_chassis_ctrl_d2"
  ```

- **Line ~147**: Renamed D5 publisher topic
  ```cpp
  // Before
  "pub_rovercontrol_d5"
  
  // After
  "tpc_chassis_ctrl_d5"
  ```

#### `src/pkg_chassis_control/src/node_domain_bridge.cpp`
- **Line ~16-17**: Updated documentation comments
  ```cpp
  // Before
  * - Subscribes from Domain ID 5: Topic "pub_rovercontrol_d5"
  * - Publishes to Domain ID 2: Topic "pub_rovercontrol"
  
  // After
  * - Subscribes from Domain ID 5: Topic "tpc_chassis_ctrl_d5"
  * - Publishes to Domain ID 2: Topic "tpc_chassis_ctrl"
  ```

- **Line ~26**: Renamed D2 publisher topic
  ```cpp
  // Before
  "pub_rovercontrol"
  
  // After
  "tpc_chassis_ctrl"
  ```

- **Line ~31**: Renamed D5 subscriber topic
  ```cpp
  // Before
  "pub_rovercontrol_d5"
  
  // After
  "tpc_chassis_ctrl_d5"
  ```

- **Line ~89**: Updated warning message
  ```cpp
  // Before
  "Waiting for messages from Domain 5 on 'pub_rovercontrol_d5'..."
  
  // After
  "Waiting for messages from Domain 5 on 'tpc_chassis_ctrl_d5'..."
  ```

## Topic Renaming Summary

| Old Topic Name         | New Topic Name            | Domain | Publisher Node        | Subscriber Node     |
|------------------------|---------------------------|--------|-----------------------|---------------------|
| `pub_rovercontrol`     | `tpc_chassis_ctrl`        | D2     | Domain Bridge         | Chassis Controller  |
| `pub_rovercontrol_d2`  | `tpc_chassis_ctrl_d2`     | D2     | Chassis Controller    | Vision Navigation   |
| `pub_rovercontrol_d5`  | `tpc_chassis_ctrl_d5`     | D5     | Chassis Controller    | Domain Bridge       |
| `pub_rovercontrol`     | `tpc_chassis_cmd`         | D5     | Domain Bridge         | STM32 Chassis       |
| `tp_imu_data_d5`       | `tpc_chassis_imu`         | D5     | STM32 Chassis         | Chassis Sensors     |
| `tp_sensdata_d6`       | `tpc_chassis_sensors`     | D6     | STM32 Sensors         | Chassis Sensors     |

## Naming Convention Rationale

**Prefix**: `tpc_` (Topic)  
**Pattern**: `tpc_<subsystem>_<data_type>`  
**Benefits**:
- Consistent namespace for all topics
- Easy to distinguish from services (`srv_`) and actions (`act_`)
- Improved searchability and maintainability

## Next Steps

### STM32 Build Process

#### Chassis Dynamics Workspace
1. Regenerate message headers for ChassisCtrl and ChassisIMU:
   ```bash
   cd mros2-mbed-chassis-dynamics
   ./build.bash
   ```

2. Verify generated headers:
   - `mros2_add_msgs/mros2_msgs/msgs_ifaces/msg/chassis_ctrl.hpp`
   - `mros2_add_msgs/mros2_msgs/msgs_ifaces/msg/chassis_imu.hpp`

3. Flash updated firmware to STM32 Chassis board

#### Sensors GNSS Workspace
1. Regenerate message headers for ChassisSensors:
   ```bash
   cd mros2-mbed-sensors-gnss
   ./build.bash
   ```

2. Verify generated header:
   - `mros2_add_msgs/mros2_msgs/msgs_ifaces/msg/chassis_sensors.hpp`

3. Flash updated firmware to STM32 Sensors board

### ws_rpi Build Process
1. Clean and rebuild workspace:
   ```bash
   cd ws_rpi
   colcon build --cmake-clean-cache
   source install/setup.bash
   ```

2. Verify topic names:
   ```bash
   ros2 topic list | grep tpc_chassis
   ```

### Testing
1. Start domain bridge node (RPi, Domain 2):
   ```bash
   ros2 run pkg_chassis_control node_domain_bridge
   ```

2. Start chassis controller (RPi, Domain 2+5):
   ```bash
   ros2 run pkg_chassis_control node_chassis_controller
   ```

3. Monitor topics:
   ```bash
   # Domain 2
   ros2 topic echo /tpc_chassis_ctrl
   
   # Domain 5
   ROS_DOMAIN_ID=5 ros2 topic echo /tpc_chassis_ctrl_d5
   ```

4. Publish test command (Domain 5):
   ```bash
   ROS_DOMAIN_ID=5 ros2 topic pub /tpc_chassis_cmd msgs_ifaces/msg/ChassisCtrl \
     "{fdr_msg: 2, ro_ctrl_msg: 0.5, spd_msg: 100, bdr_msg: 1}"
   ```

## Backward Compatibility

**Breaking Changes**: Yes - all nodes using old topic names must be updated.

**Migration Required**:
- ✅ STM32 Chassis Controller firmware
- ✅ ws_rpi Chassis Controller node
- ✅ ws_rpi Domain Bridge node
- ⏳ ws_jetson Vision Navigation (if subscribes to chassis topics)
- ⏳ ws_base Ground Station (if monitors chassis topics)
- ⏳ Documentation (README.md, TOPIC_VERIFICATION_REPORT.md)

## Architecture Clarification

**Domain Bridge Purpose**:  
The domain bridge (D5→D2) is an **intentional cross-domain relay**, not a bug. This architecture exists because:
- Two STM32 boards (Chassis + Sensors) cannot operate on the same ROS domain due to mros2 limitations
- Domain bridge enables communication between isolated domains
- Chassis STM32 (D5) ← Bridge (D2↔D5) ← Vision/Control Nodes (D2)

## Benefits

1. **Code Consistency**: All systems use same message structure (ChassisCtrl)
2. **Maintainability**: Standardized topic naming makes codebase easier to navigate
3. **Type Safety**: Eliminates potential for struct nesting errors
4. **Documentation**: Clearer message flow with consistent naming

## References

- Message Definition: `ws_rpi/src/msgs_ifaces/msg/ChassisCtrl.msg`
- STM32 Application: `mros2-mbed-chassis-dynamics/workspace/chassis_controller/app.cpp`
- Domain Bridge: `ws_rpi/src/pkg_chassis_control/src/node_domain_bridge.cpp`
- Chassis Controller: `ws_rpi/src/pkg_chassis_control/src/node_chassis_controller.cpp`
