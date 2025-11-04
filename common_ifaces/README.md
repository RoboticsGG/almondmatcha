# Common Interfaces

**Single source of truth for all ROS2 interface definitions used across the autonomous rover system.**

## Purpose

This directory contains shared interface packages (messages, actions, services) that are used by multiple workspaces in the rover system. By centralizing interface definitions, we ensure:

- ✅ **Consistency**: All nodes use identical message structures
- ✅ **Maintainability**: Update once, apply everywhere
- ✅ **Version control**: Single point of interface versioning
- ✅ **No duplication**: Eliminates drift between workspace copies

## Structure

```
common_ifaces/
├── msgs_ifaces/              Message definitions
│   ├── msg/
│   │   ├── ChassisCtrl.msg       Chassis control commands (steering, speed)
│   │   ├── ChassisIMU.msg        IMU sensor data (accel, gyro)
│   │   ├── ChassisSensors.msg    Encoder and battery telemetry
│   │   └── SpresenseGNSS.msg     GNSS position data
│   ├── CMakeLists.txt
│   └── package.xml
│
├── action_ifaces/            Action definitions
│   ├── action/
│   │   └── DesData.action        Destination navigation goal
│   ├── CMakeLists.txt
│   └── package.xml
│
└── services_ifaces/          Service definitions
    ├── srv/
    │   └── SpdLimit.srv          Speed limit service
    ├── CMakeLists.txt
    └── package.xml
```

## Usage

### For ROS2 Workspaces (ws_rpi, ws_base, ws_jetson)

**Method 1: Symbolic Links (Recommended for development)**
```bash
cd ws_rpi/src
ln -s ../../common_ifaces/msgs_ifaces .
ln -s ../../common_ifaces/action_ifaces .
ln -s ../../common_ifaces/services_ifaces .
```

**Method 2: Build from common_ifaces**
```bash
cd common_ifaces
colcon build
source install/setup.bash

cd ../ws_rpi
colcon build  # Will find interfaces in common_ifaces/install
```

### For STM32 Workspaces (mros2-mbed)

STM32 workspaces require message files to be copied into their structure due to mros2 build system requirements.

**Automated sync script** (run before STM32 builds):
```bash
# Copy only required messages to STM32 chassis
cp common_ifaces/msgs_ifaces/msg/ChassisCtrl.msg \
   mros2-mbed-chassis-dynamics/mros2_add_msgs/mros2_msgs/msgs_ifaces/msg/

cp common_ifaces/msgs_ifaces/msg/ChassisIMU.msg \
   mros2-mbed-chassis-dynamics/mros2_add_msgs/mros2_msgs/msgs_ifaces/msg/

# Copy only required messages to STM32 sensors
cp common_ifaces/msgs_ifaces/msg/ChassisSensors.msg \
   mros2-mbed-sensors-gnss/mros2_add_msgs/mros2_msgs/msgs_ifaces/msg/
```

## Interface Definitions

### Messages (msgs_ifaces)

#### ChassisCtrl.msg
Motor control commands for the rover chassis.
```
uint8 fdr_msg       # Front direction: 1=right, 2=straight, 3=left
float32 ro_ctrl_msg # Steering control value (0.0 to 1.0)
uint8 spd_msg       # Speed command (0-255)
uint8 bdr_msg       # Back direction: 0=stop, 1=forward, 2=backward
```

#### ChassisIMU.msg
Inertial measurement unit sensor data.
```
int32 accel_x  # Acceleration along X-axis (m/s^2 * 1000)
int32 accel_y  # Acceleration along Y-axis (m/s^2 * 1000)
int32 accel_z  # Acceleration along Z-axis (m/s^2 * 1000)
int32 gyro_x   # Angular velocity around X-axis (rad/s * 1000)
int32 gyro_y   # Angular velocity around Y-axis (rad/s * 1000)
int32 gyro_z   # Angular velocity around Z-axis (rad/s * 1000)
```

#### ChassisSensors.msg
Motor encoder and battery monitoring data.
```
int32 mt_lf_encode_msg  # Left motor encoder count
int32 mt_rt_encode_msg  # Right motor encoder count
float32 sys_current_msg # System battery current (Amperes)
float32 sys_volt_msg    # System battery voltage (Volts)
```

#### SpresenseGNSS.msg
GPS/GNSS position data from Sony Spresense board.
```
string date              # Date string (YYYY-MM-DD)
string time              # Time string (HH:MM:SS)
int32 num_satellites     # Number of satellites in view
bool fix                 # GNSS fix status
float64 latitude         # Latitude in decimal degrees
float64 longitude        # Longitude in decimal degrees
float64 altitude         # Altitude above mean sea level (meters)
```

### Actions (action_ifaces)

#### DesData.action
Navigation destination goal action.
```
# Goal
float64 des_lat   # Destination latitude
float64 des_long  # Destination longitude
---
# Result
string result_fser  # Result message
---
# Feedback
float64 dis_remain  # Remaining distance to destination (km)
```

### Services (services_ifaces)

#### SpdLimit.srv
Speed limit configuration service.
```
# Request
uint8 rover_spd  # Speed limit (0-100%)
---
# Response
# (acknowledgment)
```

## Build Order

When building from scratch, follow this order:

```bash
# 1. Build common interfaces first
cd common_ifaces
colcon build --symlink-install

# 2. Source the interfaces
source install/setup.bash

# 3. Build dependent workspaces
cd ../ws_rpi
colcon build

cd ../ws_base
colcon build

cd ../ws_jetson
colcon build
```

## Updating Interfaces

**Important**: Always update interfaces in `common_ifaces/` only!

1. Edit the message/action/service in `common_ifaces/`
2. Rebuild common_ifaces: `colcon build --packages-select msgs_ifaces`
3. Source: `source install/setup.bash`
4. Rebuild dependent packages
5. For STM32: Run sync script to copy updated files

## Migration Notes

**Previous Structure**: Each workspace had its own copy of interface packages
- ws_rpi/src/msgs_ifaces/
- ws_base/src/msgs_ifaces/
- STM32 workspaces had their own copies

**Current Structure**: Single source in common_ifaces/
- ROS2 workspaces link or depend on common_ifaces
- STM32 workspaces copy from common_ifaces during build prep

This change was made on 2025-11-04 to eliminate maintenance overhead and prevent version drift.

## Version History

- **v1.0.0** (2025-11-04): Initial consolidation
  - Migrated from distributed copies to single source
  - Unified ChassisCtrl, ChassisIMU, ChassisSensors
  - Standardized SpresenseGNSS (replaced GnssData variants)
