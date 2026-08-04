# Topics Reference

## Domain 5 — mission_command_node

### Action Clients
| Name | Type | Direction | Purpose |
|------|------|-----------|---------|
| `/des_data` | DesData | BASE→ROVER | Navigation goal (lat/long, feedback: distance remaining) |

### Service Clients
| Name | Type | Direction | Purpose |
|------|------|-----------|---------|
| `/srv_spd_limit` | SpdLimit | BASE→ROVER | Speed limit command (0-100%), to `chassis_controller_node` on the RPi |

`mission_command_node` does not subscribe to any topics — it only calls the
action and service above, retrying the goal on a timer
(`mission_retry_sec`) until it is accepted.

## Domain 4 — mission_monitoring_node_pc

### Subscribed Topics
| Name | Type | Rate | Source | Purpose |
|------|------|------|--------|---------|
| `tpc_telemetry_relay` | TelemetryRelay | 5 Hz | ws_rpi (`mission_monitoring_node_rpi`) | Single aggregated telemetry topic — the only thing this node reads |

`mission_monitoring_node_pc` never joins Domain 5. It has no visibility into
the raw D5 topics (`tpc_gnss_spresense`, `tpc_chassis_cmd`, etc.) — those are
aggregated into `TelemetryRelay` by the RPi relay node before crossing to D4.

## Message Types

### DesData (Action)
```yaml
# Goal
float64 des_lat
float64 des_long

# Feedback
float64 dis_remain     # Distance remaining (km)

# Result
string result_fser     # Result message
```

### SpdLimit (Service)
```yaml
# Request
uint8 rover_spd        # Speed limit, 0-100% PWM duty cycle

# Response
string spd_result      # Result message
```

### TelemetryRelay (Topic, subscribed on D4)

Aggregates all Domain 5 rover state into one message at 5 Hz. Key fields
(see `common_ifaces/msgs_ifaces/msg/TelemetryRelay.msg` for the complete,
authoritative list):

```yaml
std_msgs/Header header

bool mission_active
float64 distance_remaining_km

bool spresense_valid
float32 spresense_latitude, spresense_longitude, spresense_altitude
int32 spresense_satellites

bool ublox_valid
float64 ublox_latitude, ublox_longitude, ublox_altitude
string ublox_fix_quality
float32 ublox_centimeter_error
int32 ublox_satellites

bool chassis_cmd_valid
float32 chassis_cmd_steer_dir   # fdr_msg: 1=right 2=straight 3=left
float32 chassis_cmd_drive_dir   # bdr_msg: 0=stop 1=forward 2=backward

bool chassis_sensors_valid
float32 encoder_left, encoder_right, voltage, current, power_watts

bool chassis_imu_valid
float32 accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

bool lane_valid
float32 lane_theta, lane_b
bool lane_detected               # always false — raw lane data stays D6-only, by design

bool destination_valid
float32 destination_latitude, destination_longitude
```

## Topic Naming Convention

Pattern: `tpc_<subsystem>_<description>`

Examples:
- `tpc_telemetry_relay` - aggregated D5→D4 telemetry
- `tpc_gnss_spresense` - GNSS subsystem, Spresense data (D5, not subscribed here)
- `tpc_chassis_cmd` - Chassis subsystem, command data (D5, not subscribed here)

## Domain Configuration

Base station is **dual-domain**, not unified:

| Workspace | Domain | Components |
|-----------|--------|------------|
| ws_rpi | 5 | GNSS, Chassis Control, Sensors, Mission Monitor/Relay (8 nodes) |
| ws_base | 5 | `mission_command_node` |
| ws_base | 4 | `mission_monitoring_node_pc` |
| ws_jetson | 5 | `rover_kinematic_control` (D5-side context) |
| ws_jetson | 4 | `rover_local_monitoring_node` |
| STM32 Chassis | 5 | IMU, Motor Encoders (mROS2) |
| STM32 Sensors | 5 | GPS/GNSS, encoders, power (mROS2) |

**Note:** `mission_command_node` communicates directly with rover-side D5
nodes (native DDS discovery, no bridge). `mission_monitoring_node_pc` is
deliberately kept off D5 — it only ever sees the aggregated D4 relay, which
keeps the base station's monitoring role from adding any STM32 memory cost.

## Debug Commands

```bash
# List all topics (per domain)
export ROS_DOMAIN_ID=5
ros2 topic list
export ROS_DOMAIN_ID=4
ros2 topic list

# Topic info
ROS_DOMAIN_ID=5 ros2 topic info /des_data
ROS_DOMAIN_ID=4 ros2 topic info tpc_telemetry_relay

# Echo the relay topic
ROS_DOMAIN_ID=4 ros2 topic echo tpc_telemetry_relay

# Check action status
ROS_DOMAIN_ID=5 ros2 action list
ROS_DOMAIN_ID=5 ros2 action info /des_data

# Service list
ROS_DOMAIN_ID=5 ros2 service list
ROS_DOMAIN_ID=5 ros2 service type /srv_spd_limit
```
