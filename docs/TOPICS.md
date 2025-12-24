# ROS2 Topics Reference

Complete topic reference for Almondmatcha rover system.

**Domain Architecture:**
- **Domain 5 (Control):** Network-wide control topics visible to all systems
- **Domain 6 (Vision):** Jetson localhost vision topics (camera streams)

## Topic Naming Convention

- Prefix: `tpc_` (topic)
- Descriptive middle section
- Domain-specific topics noted in description

## Vision Topics (Jetson)

### `tpc_rover_d415_rgb`

**Type:** `sensor_msgs/msg/Image`  
**Publisher:** `camera_stream`  
**Subscribers:** `lane_detection`  
**Rate:** 30 FPS  
**QoS:** Best Effort, Depth 1  
**Domain:** 6 (Jetson localhost only)  

RGB image stream from Intel RealSense D415 camera.

**Fields:**
```yaml
header:
  stamp: {sec, nanosec}
  frame_id: "camera_rgb_optical_frame"
height: 720              # pixels
width: 1280              # pixels
encoding: "rgb8"         # RGB 8-bit per channel
is_bigendian: 0
step: 3840               # bytes per row (1280 * 3)
data: [...]              # Raw pixel data
```

---

### `tpc_rover_d415_depth`

**Type:** `sensor_msgs/msg/Image`  
**Publisher:** `camera_stream`  
**Subscribers:** (reserved for obstacle avoidance)  
**Rate:** 30 FPS  
**QoS:** Best Effort, Depth 1  
**Domain:** 6 (Jetson localhost only)  

Depth image stream from Intel RealSense D415 camera.

**Fields:**
```yaml
header:
  stamp: {sec, nanosec}
  frame_id: "camera_depth_optical_frame"
height: 720
width: 1280
encoding: "16UC1"        # 16-bit unsigned depth in mm
is_bigendian: 0
step: 2560               # bytes per row (1280 * 2)
data: [...]              # Depth values (0-65535 mm)
```

---

### `tpc_rover_nav_lane`

**Type:** `std_msgs/msg/Float32MultiArray`  
**Publisher:** `lane_detection` (Domain 6)  
**Subscribers:** `steering_control` (Domain 5)  
**Rate:** 25-30 FPS  
**QoS:** Reliable, Depth 10  
**Domain:** 5 (published to control network)  

Lane detection parameters for steering control. Cross-domain topic published from Domain 6 to Domain 5.

**Fields:**
```yaml
layout:
  dim: []
  data_offset: 0
data: [theta, b, detected]
```

**Data Array:**
- `data[0]` - **theta (degrees):** Heading error angle
  - Positive: lane on right (need to turn right)
  - Negative: lane on left (need to turn left)
  - Range: typically -30° to +30°
- `data[1]` - **b (pixels):** Lateral offset from center
  - Positive: camera right of lane center
  - Negative: camera left of lane center
  - Range: -640 to +640 (half of image width)
- `data[2]` - **detected (0 or 1):** Lane detection confidence
  - 1.0: Lane detected successfully
  - 0.0: No lane detected

---

---

## Control Topics (Jetson/Base)

### `tpc_rover_fmctl`

**Type:** `std_msgs/msg/Float32MultiArray`  
**Publisher:** `steering_control`  
**Subscribers:** `node_chassis_controller`  
**Rate:** 50 Hz  
**QoS:** Reliable, Depth 10  
**Domain:** 5  

Steering control output from PID controller.

**Fields:**
```yaml
data: [steering_angle, detected]
```

**Data Array:**
- `data[0]` - **steering_angle (degrees):** Commanded steering angle
  - Positive: turn right
  - Negative: turn left
  - Range: -45° to +45° (configurable limits)
- `data[1]` - **detected (0 or 1):** Lane detection status pass-through

---

## Chassis Control Topics (RPi ↔ STM32)

### `tpc_chassis_cmd`

**Type:** `msgs_ifaces/msg/ChassisCtrl`  
**Publisher:** `node_chassis_controller`  
**Subscribers:** `chassis_controller` (STM32)  
**Rate:** 50 Hz  
**QoS:** Reliable, Depth 10  
**Domain:** 5  

Motor and steering commands to chassis controller.

**Message Definition:**
```
uint8 fdr_msg           # Front direction: 0=straight, 1=left, 2=right
float32 ro_ctrl_msg     # Steering angle in degrees (-45 to +45)
uint8 bdr_msg           # Back direction: 0=forward, 1=backward, 2=stop
uint8 spd_msg           # Motor speed 0-100%
```

**Example:**
```yaml
fdr_msg: 0
ro_ctrl_msg: 5.0        # 5° right turn
bdr_msg: 0              # Forward
spd_msg: 50             # 50% speed
```

---

### `tpc_chassis_imu`

**Type:** `msgs_ifaces/msg/ChassisIMU`  
**Publisher:** `chassis_controller` (STM32)  
**Subscribers:** `node_chassis_imu`, `node_ekf_fusion` (future)  
**Rate:** 10 Hz  
**QoS:** Reliable, Depth 10  
**Domain:** 5  

IMU sensor data from LSM6DSV16X (sampled at 100 Hz, published at 10 Hz).

**Message Definition:**
```
int32 accel_x          # Acceleration X-axis (raw sensor units)
int32 accel_y          # Acceleration Y-axis
int32 accel_z          # Acceleration Z-axis
int32 gyro_x           # Angular rate X-axis (raw sensor units)
int32 gyro_y           # Angular rate Y-axis
int32 gyro_z           # Angular rate Z-axis
```

**Unit Conversion (LSM6DSV16X):**
- Accel sensitivity: ±2g → 0.061 mg/LSB
- Gyro sensitivity: ±250 dps → 8.75 mdps/LSB

---

### `tpc_chassis_sensors`

**Type:** `msgs_ifaces/msg/ChassisSensors`  
**Publisher:** `sensors_node` (STM32)  
**Subscribers:** `node_chassis_sensors`, `node_ekf_fusion` (future)  
**Rate:** 4 Hz (aggregated from 3 tasks)  
**QoS:** Reliable, Depth 10  
**Domain:** 5  

Encoder, power, and GNSS data from sensors board.

**Message Definition:**
```
int32 mt_lf_encode_msg     # Left encoder count
int32 mt_rt_encode_msg     # Right encoder count
float32 sys_volt_msg       # System voltage (V)
float32 sys_current_msg    # System current (A)
```

**Example:**
```yaml
mt_lf_encode_msg: 12345    # Left motor encoder count
mt_rt_encode_msg: 12340    # Right motor encoder count
sys_volt_msg: 12.5         # 12.5V battery
sys_current_msg: 2.3       # 2.3A draw
```

---

## GNSS Navigation Topics (RPi)

### `tpc_gnss_spresense`

**Type:** `msgs_ifaces/msg/SpresenseGNSS`  
**Publisher:** `node_gnss_spresense`  
**Subscribers:** `node_gnss_mission_monitor`, `node_ekf_fusion` (future)  
**Rate:** 10 Hz  
**QoS:** Reliable, Depth 10  
**Domain:** 5  

GPS position data from Sony Spresense module.

**Message Definition:**
```
float64 latitude           # Decimal degrees (WGS84)
float64 longitude          # Decimal degrees (WGS84)
float64 altitude           # Meters above mean sea level
float32 accuracy           # Position accuracy (m)
uint8 fix_quality          # 0=no fix, 1=GPS, 2=DGPS, 3=RTK
uint8 num_satellites       # Number of satellites in view
```

**Example:**
```yaml
latitude: 7.007286
longitude: 100.502030
altitude: 15.5
accuracy: 2.5              # ±2.5m accuracy
fix_quality: 1             # Standard GPS fix
num_satellites: 8
```

---

### `tpc_gnss_mission_active`

**Type:** `std_msgs/msg/Bool`  
**Publisher:** `node_gnss_mission_monitor`  
**Subscribers:** `node_chassis_controller`  
**Rate:** 10 Hz  
**QoS:** Reliable, Depth 10  
**Domain:** 5  

Mission status flag (used for cruise control logic).

**Fields:**
```yaml
data: true    # Mission active
data: false   # Mission inactive or completed
```

---

### `tpc_gnss_mission_remain_dist`

**Type:** `std_msgs/msg/Float64`  
**Publisher:** `node_gnss_mission_monitor`  
**Subscribers:** Base station monitoring  
**Rate:** 10 Hz  
**QoS:** Reliable, Depth 10  
**Domain:** 5  

Remaining distance to waypoint in kilometers.

**Fields:**
```yaml
data: 0.125    # 125 meters to target
data: 0.0      # Arrived at waypoint
```

---

### `tpc_rover_dest_coordinate`

**Type:** `std_msgs/msg/Float64MultiArray`  
**Publisher:** Service response from `node_gnss_mission_monitor`  
**Subscribers:** Internal mission monitor use  
**Rate:** Event-driven  
**QoS:** Reliable, Depth 10  
**Domain:** 5  

Destination coordinates for navigation mission.

**Fields:**
```yaml
data: [latitude, longitude]
```

**Example:**
```yaml
data: [7.007286, 100.502030]    # Target waypoint in Thailand
```

---

## Actions and Services (Domain 5)

### `/des_data` (Action)

**Type:** `action_ifaces/DesData`  
**Client:** `mission_command_node` (ws_base)  
**Server:** `node_gnss_mission_monitor` (ws_rpi)  
**Domain:** 5  

Navigation goal action for waypoint missions.

**Goal:**
```yaml
float64 des_lat     # Target latitude
float64 des_long    # Target longitude
```

**Feedback:**
```yaml
float64 dis_remain  # Distance remaining (km)
```

**Result:**
```yaml
uint8 result_fser   # Result code
```

### `/srv_spd_limit` (Service)

**Type:** `services_ifaces/SpdLimit`  
**Client:** `mission_command_node` (ws_base)  
**Server:** `node_chassis_controller` (ws_rpi)  
**Domain:** 5  

Speed limit command service.

**Request:**
```yaml
uint8 rover_spd     # Speed limit (0-100%)
```

**Response:**
```yaml
uint8 ack           # Acknowledgment
```

---

## Archived Topics (No Longer Used)

### `tpc_telemetry` (Removed)

This topic was planned for aggregated telemetry but is no longer needed with Domain 5 unification. All telemetry topics are directly accessible to ws_base.# Sensors
float32 battery_voltage
float32 system_current

# Status
bool mission_active
float64 distance_to_waypoint
uint8 lane_detected
```

---
### `tpc_command` (Archived - No Longer Used)

Commands from base station are now sent directly via actions and services on Domain 5:
- Navigation goals: `/des_data` action
- Speed limits: `/srv_spd_limit` service

This topic is no longer needed with Domain 5 unification.float64 param2          # Longitude (if waypoint)
```

---

## Topic Dependency Graph

**Vision Processing (Domain 6 → Domain 5):**
```
camera_stream (D6)
    └── tpc_rover_d415_rgb (D6)
            └── lane_detection (D6)
                    └── tpc_rover_nav_lane (D5)
                            └── steering_control (D5)
                                    └── tpc_rover_fmctl (D5)
```

**Chassis Control (Domain 5):**
```
node_chassis_controller (RPi)
    └── tpc_chassis_cmd (D5)
            └── chassis_controller (STM32 mROS2)
                    └── tpc_chassis_imu (D5)
                            └── node_chassis_imu (RPi)
```

**Sensor Data Flow (Domain 5):**
```
sensors_node (STM32 mROS2)
    └── tpc_chassis_sensors (D5)
            └── node_chassis_sensors (RPi)
                    └── (Data logging/processing)

node_gnss_spresense (RPi)
    └── tpc_gnss_spresense (D5)
            └── node_gnss_mission_monitor (RPi)
```

---

## QoS Policy Summary

| Topic | Reliability | Durability | History | Depth |
|-------|-------------|-----------|---------|-------|
| `tpc_rover_d415_rgb` | Best Effort | Volatile | Keep Last | 1 |
| `tpc_rover_d415_depth` | Best Effort | Volatile | Keep Last | 1 |
| `tpc_rover_nav_lane` | Reliable | Volatile | Keep Last | 10 |
| `tpc_rover_fmctl` | Reliable | Volatile | Keep Last | 10 |
| `tpc_chassis_cmd` | Reliable | Volatile | Keep Last | 10 |
| `tpc_chassis_imu` | Reliable | Volatile | Keep Last | 10 |
| `tpc_chassis_sensors` | Reliable | Volatile | Keep Last | 10 |
| `tpc_gnss_*` | Reliable | Volatile | Keep Last | 10 |

**Rationale:**
- **Vision streams:** Best Effort for high throughput, low latency
- **Control/sensor data:** Reliable to ensure no command loss
- **History depth:** 10 for sensor data allows brief buffering during CPU spikes

---

## Message Interface Packages

All custom message types defined in `common_ifaces/`:

- **msgs_ifaces:** ChassisCtrl, ChassisIMU, ChassisSensors, SpresenseGNSS
- **action_ifaces:** DesData (navigation goals)
- **services_ifaces:** SpdLimit (speed limit updates)

Build order: `colcon build --packages-select action_ifaces msgs_ifaces services_ifaces`

---

**See Also:**
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture
- [DOMAINS.md](DOMAINS.md) - Domain configuration details
