# QoS (Quality of Service) Configuration Guide

## Overview

This document explains the QoS policies used across the rover system to ensure reliable communication between:
- **STM32 Nucleo boards** (mbed mros2) - Hardware sensors
- **ws_rpi nodes** (ROS2 Humble) - Raspberry Pi application nodes  
- **ws_base nodes** (ROS2 Humble) - Base station monitoring (Domain 5 unified)
- **ws_jetson nodes** (ROS2 Humble) - Vision navigation (Domain 5 unified)

All systems operate on **Domain 5** for direct communication without bridges.

## QoS Policy Summary

### Critical Principle: **Subscriber QoS Must Match Publisher QoS**

For successful communication, subscribers must use **compatible** QoS policies with publishers:
- **Durability**: Subscriber can be more flexible (volatile can receive transient_local)
- **Reliability**: Subscriber must match (best_effort cannot receive reliable-only)

## System Architecture

```
Domain 5 (Unified System - All Components)
┌─────────────────────────────────────────────────────────────┐
│ STM32 Nucleo Boards            ws_rpi Nodes                 │
│ (mbed mros2 publishers)        (ROS2 subscribers/pubs)      │
│  - chassis_imu                  - node_chassis_imu          │
│  - chassis_sensors              - node_chassis_sensors      │
│  QoS: best_effort +             - node_gnss_spresense       │
│       volatile (default)        - node_mission_monitor      │
│                                 - node_chassis_controller   │
│  Direct DDS ◄──────────►       QoS: Various (see below)    │
│                                                              │
│ ws_base (Ground Station)       ws_jetson (Vision)           │
│  - mission_command_node         - camera_stream_node        │
│  - mission_monitoring_node      - lane_detection_node       │
│  QoS: reliable +                - steering_control_node     │
│       transient_local           QoS: reliable +             │
│                                      transient_local         │
│  All nodes communicate directly on Domain 5                 │
└─────────────────────────────────────────────────────────────┘
```

## Topic-by-Topic QoS Configuration

### 1. STM32 Sensor Topics (Domain 5)

#### `tpc_chassis_imu` - IMU Sensor Data
**Publisher**: STM32 Nucleo (mbed mros2)
```cpp
// mbed mros2 default QoS
node.create_publisher<msgs_ifaces::msg::ChassisIMU>("tpc_chassis_imu", 10);
// → best_effort + volatile (mbed default)
```

**Subscriber**: ws_rpi `node_chassis_imu`
```cpp
rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
qos.best_effort();  // Match STM32 mbed
// volatile is default
```

**Rationale**: 
- STM32 mbed uses best_effort by default (lower overhead)
- Volatile durability for real-time sensor streams (no late-joiner support needed)
- High-frequency data (10 Hz) where occasional packet loss is acceptable

---

#### `tpc_chassis_sensors` - Encoder/Power Data
**Publisher**: STM32 Nucleo (mbed mros2)
```cpp
node.create_publisher<msgs_ifaces::msg::ChassisSensors>("tpc_chassis_sensors", 10);
// → best_effort + volatile
```

**Subscriber**: ws_rpi `node_chassis_sensors`
```cpp
rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
qos.best_effort();  // Match STM32
```

**Rationale**: Same as IMU - real-time sensor data tolerates packet loss

---

### 2. ws_rpi Application Topics (Domain 5)

#### `tpc_gnss_spresense` - GNSS Position
**Publisher**: ws_rpi `node_gnss_spresense`
```cpp
rclcpp::QoS qos(10);
qos.reliable().transient_local();
```

**Subscribers**: 
- ws_rpi `node_gnss_mission_monitor`
- ws_base `mission_monitoring_node` (direct on Domain 5)
```cpp
rclcpp::QoS qos(10);
qos.reliable().transient_local();
```

**Rationale**:
- Reliable delivery critical for navigation waypoints
- Transient_local provides last-known position to late joiners
- Lower frequency (1 Hz) suits reliable transport

---

#### `tpc_chassis_cmd` - Motor Commands
**Publisher**: ws_rpi `node_chassis_controller`
```cpp
rclcpp::QoS qos(10);
qos.reliable().transient_local();
```

**Subscriber**: STM32 Nucleo (mbed mros2)
```cpp
node.create_subscription<msgs_ifaces::msg::ChassisCtrl>("tpc_chassis_cmd", 10, callback);
// → best_effort + volatile (mbed default)
```

**Rationale**:
- Reliable from controller to ensure command delivery
- STM32 subscriber uses best_effort (compatible - can receive reliable)
- Transient_local preserves last command state

---

#### `tpc_gnss_mission_active` - Mission Status
**Publisher**: ws_rpi `node_gnss_mission_monitor`
```cpp
rclcpp::QoS qos(10);
qos.reliable().transient_local();
```

**Subscribers**: 
- ws_rpi `node_chassis_controller`
- ws_base `mission_monitoring_node` (direct on Domain 5)

**Rationale**:
- Critical state flag requires reliable delivery
- Transient_local ensures late joiners get current mission status

---

#### `tpc_gnss_mission_remain_dist` - Distance to Waypoint
**Publisher**: ws_rpi `node_gnss_mission_monitor`
```cpp
rclcpp::QoS qos(10);
qos.reliable().transient_local();
```

**Subscriber**: ws_base `mission_monitoring_node` (direct on Domain 5)

**Rationale**: Reliable navigation metric for base station monitoring

---

### 3. Base Station Command Topics (Domain 5)

#### `/des_data` - Navigation Goal (Action)
**Action Client**: ws_base `mission_command_node`
**Action Server**: ws_rpi `node_gnss_mission_monitor`

**QoS**: Actions use default ROS 2 reliable QoS

**Rationale**: Mission-critical action requires guaranteed delivery

#### `/srv_spd_limit` - Speed Limit (Service)
**Service Client**: ws_base `mission_command_node`
**Service Server**: ws_rpi `node_chassis_controller`

**QoS**: Services use default ROS 2 reliable QoS

**Rationale**: Critical command requires guaranteed delivery

#### `tpc_rover_dest_coordinate` - Target Waypoint
**Publisher**: ws_base `mission_command_node` (Domain 5)
```cpp
rclcpp::QoS qos(10);
qos.reliable().transient_local();
```

**Subscriber**: ws_rpi `node_gnss_mission_monitor` (Domain 5)
```cpp
rclcpp::QoS qos(10);
qos.reliable().transient_local();
```

**Rationale**:
- Mission-critical commands require guaranteed delivery
- Transient_local preserves target even if connection temporarily interrupted

---

## QoS Compatibility Matrix

| Publisher QoS          | Subscriber QoS         | Compatible? | Notes |
|------------------------|------------------------|-------------|-------|
| best_effort + volatile | best_effort + volatile | ✅ Yes  | Perfect match |
| best_effort + volatile | reliable + volatile    | ❌ No | Subscriber too strict |
| reliable + transient   | reliable + transient   | ✅ Yes | Perfect match |
| reliable + transient   | best_effort + volatile | ❌ No | Subscriber too lenient |
| best_effort + volatile | best_effort + transient| ✅ Yes | Subscriber more flexible |

## Common QoS Errors and Solutions

### Error: "New publisher discovered offering incompatible QoS"

**Symptom**:
```
[WARN] [node_name]: New publisher discovered on topic '/tpc_chassis_imu',
offering incompatible QoS. No messages will be sent to it.
Last incompatible policy: DURABILITY_QOS_POLICY
```

**Cause**: Subscriber uses `transient_local` but publisher uses `volatile`

**Solution**: Change subscriber to match publisher's durability:
```cpp
// Before (incompatible)
qos.best_effort().transient_local();

// After (compatible)
qos.best_effort();  // volatile is default
```

---

### Error: "RELIABILITY_QOS_POLICY" incompatibility

**Cause**: Subscriber uses `best_effort` but publisher uses `reliable`

**Solution**: Change subscriber to `reliable`:
```cpp
// Before (incompatible)
qos.best_effort();

// After (compatible)
qos.reliable();
```

---

## Development Guidelines

### For New Sensor Publishers (STM32 mbed)
```cpp
// Use mbed default (best_effort + volatile)
mros2::Publisher pub = node.create_publisher<MsgType>("topic_name", 10);
```

### For New Application Publishers (ws_rpi)
```cpp
// Use reliable + transient_local for critical data
rclcpp::QoS qos(10);
qos.reliable().transient_local();
auto pub = create_publisher<MsgType>("topic_name", qos);
```

### For New Subscribers
**Rule**: Always check the publisher's QoS and match it!

**STM32 sensors** (best_effort + volatile):
```cpp
rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
qos.best_effort();
```

**ws_rpi application nodes** (reliable + transient_local):
```cpp
rclcpp::QoS qos(10);
qos.reliable().transient_local();
```

---

## Verification Commands

### Check QoS of Active Topics
```bash
# Domain 5 (rover)
export ROS_DOMAIN_ID=5
ros2 topic info /tpc_chassis_imu -v
ros2 topic info /tpc_chassis_sensors -v
ros2 topic info /tpc_gnss_spresense -v
```

### Monitor QoS Warnings
```bash
# Watch for incompatibility warnings
ros2 run pkg_chassis_sensors node_chassis_imu 2>&1 | grep -i "incompatible"
```

---

## References

- [ROS2 QoS Documentation](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [mros2 QoS Defaults](https://github.com/mROS-base/mros2)
- [DDS QoS Policies](https://www.omg.org/spec/DDS/1.4/PDF)

## Author
Almondmatcha Development Team  
Last Updated: November 7, 2025

## Topic-Specific QoS Matrix

| Topic | Publisher | Publisher QoS | Subscriber | Subscriber QoS | Notes |
|-------|-----------|---------------|------------|----------------|-------|
| `tpc_chassis_imu` | STM32 Chassis | sensor_data + best_effort + volatile | node_chassis_imu, ws_base | Same | High frequency IMU |
| `tpc_chassis_sensors` | STM32 GNSS | sensor_data + best_effort + volatile | node_chassis_sensors, ws_base | Same | Encoder/power data |
| `tpc_gnss_spresense` | node_gnss_spresense | reliable + transient_local | node_gnss_mission_monitor, ws_base | Same | GNSS position |
| `tpc_chassis_cmd` | node_chassis_controller | reliable + transient_local | STM32 Chassis | best_effort (compatible) | Motor commands |
| `tpc_gnss_mission_active` | node_gnss_mission_monitor | reliable + transient_local | node_chassis_controller, ws_base | Same | Mission status |
| `tpc_gnss_mission_remain_dist` | node_gnss_mission_monitor | reliable + transient_local | ws_base | Same | Distance remaining |
| `tpc_rover_dest_coordinate` | ws_base mission_command | reliable + transient_local | node_gnss_mission_monitor | Same | Waypoint coords |
| `tpc_rover_fmctl` | ws_jetson/ws_base | reliable + transient_local | node_chassis_controller | Same | Vision control |
| `/des_data` (action) | ws_base mission_command | reliable (default) | node_gnss_mission_monitor | reliable | Navigation goals |
| `/srv_spd_limit` (service) | ws_base mission_command | reliable (default) | node_chassis_controller | reliable | Speed limits |

## Domain-Specific Considerations

### Domain 5 (Unified System)
- **All rover nodes** operate in Domain 5 (ws_rpi, ws_base, ws_jetson)
- **STM32 mbed boards** also publish to Domain 5
- **Direct communication** - no bridge needed
- QoS must match between mbed publishers and ROS2 subscribers
- Actions and services work natively without relay

## Troubleshooting QoS Warnings

### Warning: "incompatible QoS... DURABILITY_QOS_POLICY"
**Cause**: Mismatch between `transient_local` (keeps last value) and `volatile` (no history)

**Solution**: Ensure both publisher and subscriber use `transient_local()`

### Warning: "incompatible QoS... RELIABILITY_QOS_POLICY"
**Cause**: Mismatch between `reliable` and `best_effort`

**Solution**: 
- For sensor topics from STM32: Both must use `best_effort`
- For application topics: Both must use `reliable`

### Warning: "incompatible QoS... DEADLINE_QOS_POLICY"
**Cause**: One side specifies deadline, other doesn't

**Solution**: Either both specify same deadline or neither specifies it

## Implementation Examples

### Example 1: STM32 Sensor Publisher (C++ mbed)
```cpp
// In mbed node (STM32 L1/L2)
rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
qos_profile.best_effort().transient_local();

pub_imu_ = this->create_publisher<msgs_ifaces::msg::ChassisIMU>(
    "tpc_chassis_imu", qos_profile
);
```

### Example 2: ROS2 Sensor Subscriber
```cpp
// In node_chassis_imu.cpp
rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
qos_profile.best_effort().transient_local();

sub_chassis_imu_ = this->create_subscription<msgs_ifaces::msg::ChassisIMU>(
    "tpc_chassis_imu", qos_profile,
    std::bind(&ChassisIMUNode::imuCallback, this, std::placeholders::_1)
);
```

### Example 3: Application Topic with Reliable QoS
```cpp
// In node_gnss_spresense.cpp
rclcpp::QoS qos_reliable(10);
qos_reliable.reliable().transient_local();

pub_gnss_spresense_ = this->create_publisher<msgs_ifaces::msg::SpresenseGNSS>(
    "tpc_gnss_spresense", qos_reliable
);
```

### Example 4: Base Station Subscriber (Domain 5)
```cpp
// In mission_monitoring_node.cpp (ws_base)

// Subscribe to rover topics on Domain 5
rclcpp::QoS qos_reliable(10);
qos_reliable.reliable().transient_local();

sub_gnss_ = this->create_subscription<msgs_ifaces::msg::SpresenseGNSS>(
    "tpc_gnss_spresense", qos_reliable, callback
);
```

## Verification Commands

### Check Topic QoS Settings
```bash
# View QoS for a specific topic
export ROS_DOMAIN_ID=5
ros2 topic info -v /tpc_chassis_imu

# Expected output should show:
# QoS profile:
#   Reliability: BEST_EFFORT
#   Durability: TRANSIENT_LOCAL
```

### Monitor QoS Warnings
```bash
# Run node and watch for QoS warnings in logs
ros2 run pkg_chassis_sensors node_chassis_imu

# Look for warnings like:
# [WARN] ... incompatible QoS ... DURABILITY_QOS_POLICY
```

### Test QoS Compatibility
```bash
# Terminal 1: Publisher with sensor QoS
ros2 topic pub /test_topic std_msgs/msg/String "data: test" \
  --qos-durability transient_local --qos-reliability best_effort

# Terminal 2: Subscriber with matching QoS
ros2 topic echo /test_topic \
  --qos-durability transient_local --qos-reliability best_effort
```

## Best Practices

1. **STM32 Hardware Topics**: Always use `sensor_data + best_effort + transient_local`
2. **Application Topics**: Use `reliable + transient_local` for commands and state
3. **Late Joiners**: Use `transient_local` to ensure new subscribers get last value
4. **High Frequency Data**: Use `best_effort` to avoid retransmission overhead
5. **Critical Commands**: Use `reliable` to ensure delivery

## Future Enhancements

- [ ] Add QoS monitoring node to detect mismatches at runtime
- [ ] Implement adaptive QoS based on network conditions
- [ ] Add QoS profiles for different operational modes (e.g., low-bandwidth)
- [ ] Document QoS settings for ws_base and ws_jetson nodes

## References

- ROS2 QoS Documentation: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- rmw_qos_profile_sensor_data: Optimized profile for sensor topics
- CycloneDDS QoS: https://cyclonedds.io/docs/cyclonedds/latest/config/qos.html

## Author

Almondmatcha Development Team  
Date: November 7, 2025