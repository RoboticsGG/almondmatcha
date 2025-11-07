# Topic Reference - ws_base and System-Wide Topics

This document lists all ROS2 topics used in the autonomous rover system, showing which workspaces publish/subscribe to each topic.

## Topic Naming Convention

All topics follow the pattern: `tpc_<subsystem>_<description>`

- `tpc_` - Topic prefix (standard across all workspaces)
- `<subsystem>` - Component identifier (gnss, chassis, rover, etc.)
- `<description>` - Specific data type or function

## Topics Used in ws_base

### Subscribed Topics (Monitoring Node)

| Topic Name | Message Type | Publisher | Description |
|------------|--------------|-----------|-------------|
| `tpc_gnss_mission_active` | `std_msgs/Bool` | ws_rpi (pkg_gnss_navigation) | Mission active/inactive flag |
| `tpc_gnss_mission_remain_dist` | `std_msgs/Float64` | ws_rpi (pkg_gnss_navigation) | Distance remaining to target (km) |
| `tpc_gnss_spresense` | `msgs_ifaces/SpresenseGNSS` | ws_rpi (pkg_gnss_navigation) | Current GPS position |
| `tpc_rover_dest_coordinate` | `std_msgs/Float64MultiArray` | ws_rpi (pkg_gnss_navigation) | Target coordinates [lat, long] |
| `tpc_chassis_cmd` | `msgs_ifaces/ChassisCtrl` | ws_rpi (pkg_chassis_control) | Rover control commands |

### Published Topics (Command Node)

| Topic Name | Message Type | Subscriber | Description |
|------------|--------------|------------|-------------|
| *None* | - | - | Command node uses services/actions, not topics |

## System-Wide Topic Map

### GNSS and Navigation Topics

| Topic | Type | Publisher | Subscriber(s) | Domain |
|-------|------|-----------|---------------|--------|
| `tpc_gnss_spresense` | `SpresenseGNSS` | ws_rpi (node_gnss_spresense) | ws_rpi (node_gnss_mission_monitor)<br>ws_base (monitoring_node) | 5, 6 |
| `tpc_gnss_mission_active` | `Bool` | ws_rpi (node_gnss_mission_monitor) | ws_rpi (node_chassis_controller)<br>ws_base (monitoring_node) | 4, 6 |
| `tpc_gnss_mission_remain_dist` | `Float64` | ws_rpi (node_gnss_mission_monitor) | ws_base (monitoring_node) | 4, 6 |
| `tpc_rover_dest_coordinate` | `Float64MultiArray` | ws_rpi (node_gnss_mission_monitor) | ws_base (monitoring_node) | 4, 6 |

### Chassis and Motor Control Topics

| Topic | Type | Publisher | Subscriber(s) | Domain |
|-------|------|-----------|---------------|--------|
| `tpc_chassis_cmd` | `ChassisCtrl` | ws_rpi (node_chassis_controller) | ws_base (monitoring_node) | 2, 6 |
| `tpc_chassis_sensors` | `ChassisSensors` | STM32 (mros2-mbed-sensors-gnss) | ws_rpi (node_chassis_sensors) | 2, 6 |
| `tpc_chassis_imu` | `ChassisIMU` | STM32 (mros2-mbed-chassis-dynamics) | ws_rpi (node_chassis_imu) | 2, 6 |

### Vision and Lane Detection Topics

| Topic | Type | Publisher | Subscriber(s) | Domain |
|-------|------|-----------|---------------|--------|
| `tpc_rover_d415_rgb` | `sensor_msgs/Image` | ws_jetson (camera_stream_node) | ws_jetson (lane_detection_node) | 3 |
| `tpc_rover_d415_depth` | `sensor_msgs/Image` | ws_jetson (camera_stream_node) | - (optional) | 3 |
| `tpc_rover_nav_lane` | `Float32MultiArray` | ws_jetson (lane_detection_node) | ws_jetson (steering_control_node) | 3 |
| `tpc_rover_fmctl` | `Float32MultiArray` | ws_jetson (steering_control_node) | ws_rpi (node_chassis_controller) | 3, 2 |

## Topic Details

### GNSS Topics

#### tpc_gnss_spresense
**Type**: `msgs_ifaces/SpresenseGNSS`
```yaml
date: string              # YYYY-MM-DD
time: string              # HH:MM:SS
num_satellites: uint8     # Number of satellites in view
fix: uint8                # GNSS fix quality (0=no fix, 1=GPS, 2=DGPS, etc.)
latitude: float64         # Decimal degrees
longitude: float64        # Decimal degrees
altitude: float32         # Meters above mean sea level
```

**Publisher**: ws_rpi - `node_gnss_spresense` (Domain 5)
**Subscribers**: 
- ws_rpi - `node_gnss_mission_monitor` (Domain 4)
- ws_base - `mission_monitoring_node` (Domain 6)

#### tpc_gnss_mission_active
**Type**: `std_msgs/Bool`
```yaml
data: bool  # true=mission active, false=inactive
```

**Publisher**: ws_rpi - `node_gnss_mission_monitor` (Domain 4)
**Subscribers**: 
- ws_rpi - `node_chassis_controller` (Domain 2)
- ws_base - `mission_monitoring_node` (Domain 6)

#### tpc_gnss_mission_remain_dist
**Type**: `std_msgs/Float64`
```yaml
data: float64  # Remaining distance to target in kilometers
```

**Publisher**: ws_rpi - `node_gnss_mission_monitor` (Domain 4)
**Subscribers**: 
- ws_base - `mission_monitoring_node` (Domain 6)

#### tpc_rover_dest_coordinate
**Type**: `std_msgs/Float64MultiArray`
```yaml
data: [latitude, longitude]  # Target coordinates in decimal degrees
```

**Publisher**: ws_rpi - `node_gnss_mission_monitor` (Domain 4)
**Subscribers**: 
- ws_base - `mission_monitoring_node` (Domain 6)

### Chassis Topics

#### tpc_chassis_cmd
**Type**: `msgs_ifaces/ChassisCtrl`
```yaml
fdr_msg: uint8      # Front direction: 1=right turn, 2=straight, 3=left turn
ro_ctrl_msg: float32  # Steering control value (0.0 to 1.0)
spd_msg: uint16     # Speed command (0-255, mapped to percentage)
bdr_msg: uint8      # Back direction: 0=stop, 1=forward, 2=backward
```

**Publisher**: ws_rpi - `node_chassis_controller` (Domain 2)
**Subscribers**: 
- ws_base - `mission_monitoring_node` (Domain 6)
- (Internal: STM32 chassis dynamics receives commands via different mechanism)

#### tpc_chassis_sensors
**Type**: `msgs_ifaces/ChassisSensors`
```yaml
mt_lf_encode_msg: int32   # Left encoder count
mt_rt_encode_msg: int32   # Right encoder count
sys_current_msg: float32  # System current (Amps)
sys_volt_msg: float32     # System voltage (Volts)
```

**Publisher**: STM32 (mros2-mbed-sensors-gnss, Domain 6)
**Subscribers**: 
- ws_rpi - `node_chassis_sensors` (Domain 6)

#### tpc_chassis_imu
**Type**: `msgs_ifaces/ChassisIMU`
```yaml
ax: float32  # Acceleration X (m/s²)
ay: float32  # Acceleration Y (m/s²)
az: float32  # Acceleration Z (m/s²)
gx: float32  # Gyro X (rad/s)
gy: float32  # Gyro Y (rad/s)
gz: float32  # Gyro Z (rad/s)
```

**Publisher**: STM32 (mros2-mbed-chassis-dynamics, Domain 2)
**Subscribers**: 
- ws_rpi - `node_chassis_imu` (Domain 2)

### Vision Topics

#### tpc_rover_d415_rgb
**Type**: `sensor_msgs/Image` (bgr8 encoding)

**Publisher**: ws_jetson - `camera_stream_node` (Domain 3)
**Subscribers**: 
- ws_jetson - `lane_detection_node` (Domain 3)

#### tpc_rover_d415_depth
**Type**: `sensor_msgs/Image` (16UC1 encoding)

**Publisher**: ws_jetson - `camera_stream_node` (Domain 3)
**Subscribers**: 
- (Optional, for future obstacle detection)

#### tpc_rover_nav_lane
**Type**: `std_msgs/Float32MultiArray`
```yaml
data: [theta, b, detected]
  theta: float      # Lane angle in radians
  b: float          # Lane offset parameter
  detected: float   # 1.0 if lane detected, 0.0 otherwise
```

**Publisher**: ws_jetson - `lane_detection_node` (Domain 3)
**Subscribers**: 
- ws_jetson - `steering_control_node` (Domain 3)

#### tpc_rover_fmctl
**Type**: `std_msgs/Float32MultiArray`
```yaml
data: [steer_angle, detected]
  steer_angle: float  # Steering angle command
  detected: float     # 1.0 if lane guidance available
```

**Publisher**: ws_jetson - `steering_control_node` (Domain 3)
**Subscribers**: 
- ws_rpi - `node_chassis_controller` (Domain 2)

## Domain Mapping

| Domain ID | Name | Workspaces | Purpose |
|-----------|------|------------|---------|
| 2 | Chassis Dynamics | STM32 (mbed-chassis-dynamics), ws_rpi | Motor control, IMU |
| 3 | Vision | ws_jetson | Camera, lane detection, steering |
| 4 | Navigation | ws_rpi | GNSS processing, mission planning |
| 5 | Spresense GNSS | STM32 (mbed-sensors-gnss), ws_rpi | Raw GNSS data |
| 6 | Base Station | ws_base, STM32 sensors | Mission control, monitoring |

## Topic Flow Diagrams

### Mission Control Data Flow
```
STM32 (Domain 6)
  └─ tpc_chassis_sensors ───┐
                             │
ws_rpi (Domain 4)            │
  ├─ tpc_gnss_spresense ────┤
  ├─ tpc_gnss_mission_active ┤
  ├─ tpc_gnss_mission_remain_dist ┤
  ├─ tpc_rover_dest_coordinate ┤
  │                           │
ws_rpi (Domain 2)            │
  └─ tpc_chassis_cmd ────────┤
                             │
                             ↓
                    ws_base (Domain 6)
                    mission_monitoring_node
                    (Displays real-time status)
```

### Navigation Control Flow
```
ws_base (Domain 6)
  mission_command_node
  │
  ├─ Action: /des_data ─────────→ ws_rpi (node_gnss_mission_monitor)
  └─ Service: /spd_limit ───────→ ws_rpi (node_chassis_controller)
                                      │
                                      ↓
                                  tpc_chassis_cmd
                                      │
                                      ↓
                                  STM32 Motors
```

## Services and Actions (Not Topics)

ws_base uses these for command transmission:

### Actions
- `/des_data` - Navigation goal action
  - Type: `action_ifaces/action/DesData`
  - Server: ws_rpi - `node_gnss_mission_monitor`
  - Client: ws_base - `mission_command_node`

### Services
- `/spd_limit` - Speed limit service
  - Type: `services_ifaces/srv/SpdLimit`
  - Server: ws_rpi - `node_chassis_controller`
  - Client: ws_base - `mission_command_node`

## Verification Commands

### List All Topics
```bash
ros2 topic list
```

### Check Topic Type
```bash
ros2 topic info /tpc_gnss_mission_active
ros2 topic type /tpc_chassis_cmd
```

### Echo Topic Data
```bash
ros2 topic echo /tpc_gnss_spresense
ros2 topic echo /tpc_gnss_mission_remain_dist
```

### Monitor Topic Rate
```bash
ros2 topic hz /tpc_chassis_sensors
ros2 topic hz /tpc_gnss_mission_active
```

## Topic Naming Compliance Checklist

All topics used in ws_base comply with system-wide naming conventions:

- [x] Use `tpc_` prefix
- [x] Use snake_case (lowercase with underscores)
- [x] Include subsystem identifier (gnss, chassis, rover)
- [x] Descriptive name indicating data content
- [x] Consistent with publisher workspace naming
- [x] No version suffixes (e.g., avoid `_v1`, `_v2`)
- [x] No domain suffixes in topic name (domains handled by ROS_DOMAIN_ID)

## Common Issues

### Topic Not Found
**Symptom**: `ros2 topic list` doesn't show expected topic
**Solution**: 
1. Verify publisher node is running: `ros2 node list`
2. Check domain ID matches: `echo $ROS_DOMAIN_ID`
3. Verify network connectivity for multi-machine setup

### Wrong Message Type
**Symptom**: Subscriber fails with type mismatch
**Solution**: 
1. Check topic type: `ros2 topic type /tpc_<name>`
2. Rebuild interface packages: `colcon build --packages-select msgs_ifaces`
3. Re-source workspace: `source install/setup.bash`

### No Data Received
**Symptom**: Topic exists but no messages flow
**Solution**: 
1. Check publisher is sending: `ros2 topic echo /tpc_<name>`
2. Monitor publish rate: `ros2 topic hz /tpc_<name>`
3. Check QoS compatibility between pub/sub

## Maintenance

When adding new topics:
1. Follow naming convention: `tpc_<subsystem>_<description>`
2. Update this document with topic details
3. Document message type and structure
4. List all publishers and subscribers
5. Specify domain(s) where topic is used
6. Test cross-domain communication if applicable

## References

- [ROS2 Topics Documentation](https://docs.ros.org/en/latest/Concepts/Basic/About-Topics.html)
- [Message Type Definitions](../common_ifaces/)
- [ws_rpi Topic Documentation](../../ws_rpi/docs/TOPICS.md)
- [ws_jetson Topic Documentation](../../ws_jetson/docs/TOPICS.md)
