# Cross-Domain Telemetry Relay Architecture

## Overview

The telemetry relay system uses cross-domain architecture to optimize STM32 memory. Base station operates on two domains: **Domain 5** for bidirectional command/action communication with rover, **Domain 4** for unidirectional telemetry display.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   DOMAIN 5 (Rover Control)                   │
│  RPi, Jetson, STM32 nodes + Base command node                │
│                                                               │
│  ┌──────────────────────────────────────────────┐            │
│  │   mission_command_node (Base)                │            │
│  │   Pub: Commands/actions to RPi servers       │            │
│  └──────────────────────────────────────────────┘            │
│                                                               │
│  ┌──────────────────────────────────────────────┐            │
│  │   mission_monitoring_node_rpi (RPi)          │            │
│  │                                              │            │
│  │   Subscribes:                                │            │
│  │   • /tpc_chassis_sensors                     │            │
│  │   • /tpc_chassis_imu                         │            │
│  │   • /tpc_chassis_cmd                         │            │
│  │   • /tpc_gnss_mission_active                 │            │
│  │   • /tpc_gnss_mission_remain_dist            │            │
│  │   • /tpc_gnss_spresense                      │            │
│  │   • /tpc_gnss_ublox                          │            │
│  │   • /tpc_rover_dest_coordinate               │            │
│  │   • /tpc_rover_ctrl_cmd                         │            │
│  │   • /tpc_rover_nav_lane                      │            │
│  └──────────────────┬───────────────────────────┘            │
│                     │                                         │
│                     │ Publishes to Domain 4                   │
└─────────────────────┼─────────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                DOMAIN 4 (Base Telemetry Only)                │
│                                                               │
│                     /tpc_telemetry_relay                      │
│                     (TelemetryRelay.msg)                      │
│                            │                                  │
│                            ▼                                  │
│  ┌──────────────────────────────────────────────┐            │
│  │   mission_monitoring_node_pc (Base)          │            │
│  │                                              │            │
│  │   Subscribes: /tpc_telemetry_relay           │            │
│  │   Displays: Comprehensive telemetry dash     │            │
│  └──────────────────────────────────────────────┘            │
└─────────────────────────────────────────────────────────────┘
```

## Rationale

### Problem
- STM32 boards have limited SRAM for DDS participant tracking.
- Base station monitoring node as a D5 participant adds unnecessary memory load.
- The legacy `node_rover_monitoring` CSV logger also occupied a D5 participant slot.

### Solution
- `mission_command_node` (Base): remains on D5 for action/service calls to RPi.
- `mission_monitoring_node_pc` (Base) + `node_rover_local_monitoring` (Jetson): isolated to D4, no D5 participation.
- CSV logging merged into `mission_monitoring_node_rpi`, eliminating `node_rover_monitoring` from D5.
- `mission_monitoring_node_rpi`: subscribes D5, publishes D4, writes 6 per-topic CSV files.
- D5 participant count reduced from 12 to 10.

### Benefits
1. STM32 memory: 10 D5 participants vs 12+ original
2. Network isolation: all monitoring/logging traffic on D4, not D5
3. Bidirectional control preserved: mission_command_node on D5
4. Dual CSV logging: RPi high-rate + Jetson aggregated (DB-ready)

## Domain 4 Subscriber Architecture

Two nodes subscribe to `/tpc_telemetry_relay` on Domain 4:

**`mission_monitoring_node_pc` (Base station):**
- Purpose: real-time telemetry display dashboard
- Communication: unidirectional subscribe only
- Launch: `export ROS_DOMAIN_ID=4`

**`node_rover_local_monitoring` (Jetson, `rover_monitor_pkg`):**
- Purpose: secondary CSV logging + future database backend
- Communication: unidirectional subscribe only
- Logs: `ws_jetson/runs/run_NNN_YYYYMMDD_HHMMSS/` (5 CSV files at 5 Hz)
- Launch: `export ROS_DOMAIN_ID=4`

**Base station dual-domain:**
- `mission_command_node` (D5): sends commands, calls RPi action/service servers
- `mission_monitoring_node_pc` (D4): displays telemetry relay
- Launch scripts handle both domains automatically.

## Implementation Details

### mission_monitoring_node_rpi (RPi)

**Multi-Domain Architecture:**
- Main node runs in Domain 5 context (for subscriptions)
- Secondary node runs in Domain 4 context (for publishing)
- Uses `rclcpp::MultiThreadedExecutor` to spin both nodes

**Code Pattern:**
```cpp
// Create Domain 5 context for subscriptions
auto domain5_init_options = rclcpp::InitOptions();
domain5_init_options.set_domain_id(5);
auto domain5_context = std::make_shared<rclcpp::Context>();
domain5_context->init(argc, argv, domain5_init_options);

// Create Domain 4 context for publishing
auto domain4_init_options = rclcpp::InitOptions();
domain4_init_options.set_domain_id(4);
auto domain4_context = std::make_shared<rclcpp::Context>();
domain4_context->init(argc, argv, domain4_init_options);

// Node subscribes in D5, publishes in D4
auto node = std::make_shared<MissionMonitoringNodeRpi>(
    domain5_context, domain4_context);

// Executor spins both nodes
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node);
executor.add_node(node->domain4_node_);
executor.spin();
```

### mission_monitoring_node_pc (Base Station)

**Single-Domain Architecture:**
- Runs entirely in Domain 4
- Only subscribes to `/tpc_telemetry_relay`
- No direct access to Domain 5 topics

**Launch:**
```bash
export ROS_DOMAIN_ID=4
ros2 run mission_control mission_monitoring_node_pc
```

## Message Format

### TelemetryRelay.msg

Comprehensive aggregation of all Domain 5 telemetry:
- Header (timestamp, frame_id)
- Mission status (active, distance remaining)
- Dual GNSS (Spresense + uBlox RTK with validity flags)
- Chassis (commands, sensors, IMU with validity flags)
- Steering control (from Jetson)
- Lane detection (theta, b, detected flag)
- Destination coordinates

**Size**: ~280 bytes per message  
**Rate**: 5 Hz (200ms interval)

See [msgs_ifaces/msg/TelemetryRelay.msg](../common_ifaces/msgs_ifaces/msg/TelemetryRelay.msg) for complete definition.

## Launch Configuration

### RPi (Domain 5)
```bash
export ROS_DOMAIN_ID=5
ros2 run pkg_rover_monitoring mission_monitoring_node_rpi
```
- Joins Domain 5 to subscribe to telemetry topics
- Internally creates Domain 4 publisher for relay

### Base Station (Domain 4)
```bash
export ROS_DOMAIN_ID=4
ros2 run mission_control mission_monitoring_node_pc
```
- Joins only Domain 4
- Not visible to Domain 5 participants

## Verification

### Check Domain 5 Participants (from RPi)
```bash
export ROS_DOMAIN_ID=5
ros2 node list
# Should see: mission_monitoring_node_rpi, mission_command_node, etc.
# Should NOT see: mission_monitoring_node_pc
```

### Check Domain 4 Participants (from Base or Jetson)
```bash
export ROS_DOMAIN_ID=4
ros2 node list
# Should see: mission_monitoring_domain4_pub, mission_monitoring_node_pc,
#             node_rover_local_monitoring
```

### Monitor Telemetry Relay
```bash
# From base station (Domain 4)
export ROS_DOMAIN_ID=4
ros2 topic echo /tpc_telemetry_relay
```

## Troubleshooting

### Symptom: Base station sees no telemetry
**Check:**
1. Base station running on Domain 4: `echo $ROS_DOMAIN_ID` (should be 4)
2. RPi node running: `ros2 node list` on Domain 5
3. Network connectivity: `ping 192.168.1.1` from base
4. Firewall rules: DDS discovery ports (7400, 7401, etc.)

### Symptom: RPi node fails to start
**Check:**
1. Build succeeded: `colcon build --packages-select pkg_rover_monitoring`
2. No compilation errors related to multi-domain context
3. ROS2 Humble installed (multi-domain requires Humble or later)

### Symptom: High CPU usage on RPi
**Expected behavior:** Multi-threaded executor uses ~5-10% CPU for 5 Hz publishing
- If higher, check for topic storms or network issues
- Verify all subscriptions use appropriate QoS profiles

## Performance Metrics

### Memory Impact (STM32)
- Before (12 participants in D5): ~40% free RAM
- After (10 participants in D5): ~55% free RAM
- Savings from removing monitoring node + CSV logger from D5

### Network Bandwidth
- **Domain 5**: Reduced by ~1.4 KB/s (5 Hz × 280 bytes)
- **Domain 4**: Added ~1.4 KB/s (telemetry relay only)
- **Net**: Bandwidth shifted, not increased

### CPU Usage
- **RPi**: +2% (multi-threaded executor overhead)
- **Base**: Unchanged (same subscription pattern)
- **STM32**: Unchanged (no visibility to Domain 4)

## Future Enhancements

1. **Compression**: Add message compression for bandwidth optimization
2. **Adaptive Rate**: Reduce relay rate when stationary (5 Hz → 1 Hz)
3. **Selective Relay**: Optionally disable unused telemetry fields
4. **Multi-Base Support**: Multiple base stations on Domain 4 without affecting D5
5. **Telemetry Recording**: Add rosbag2 recording on Domain 4 for offline analysis

---

**Related Documentation:**
- [DOMAINS.md](DOMAINS.md) - Complete domain architecture
- [TOPICS.md](TOPICS.md) - Topic reference
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture

**Version:** 4.1  
**Last Updated:** February 26, 2026
