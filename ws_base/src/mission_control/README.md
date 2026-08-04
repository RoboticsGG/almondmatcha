# Mission Control Package

Base station mission control system for autonomous rover command and monitoring.

## Overview

The Mission Control package provides the command and monitoring infrastructure for the autonomous rover base station. It manages two primary responsibilities:

1. **Mission Command**: Send navigation goals and speed limits to the rover
2. **Mission Monitoring**: Real-time monitoring of rover telemetry and mission progress

## Architecture

### Nodes

#### MissionCommandNode (mission_command_node)
Handles mission planning and rover command transmission.

**Responsibilities:**
- Load mission parameters (destination, speed limit)
- Send destination goals to rover via ROS2 action
- Send speed limit commands to rover via ROS2 service
- Retry sending the goal on a timer (`mission_retry_sec`) until it is accepted
- Monitor goal progress through feedback; cancel if feedback stalls past `action_watchdog_timeout_sec`
- Handle mission cancellation on shutdown

**Parameters:**
- `rover_spd`: Speed limit as percentage (0-100), default: 50
- `des_lat`: Target latitude coordinate, default: 0.0
- `des_long`: Target longitude coordinate, default: 0.0
- `action_watchdog_timeout_sec`: cancel the goal if no feedback arrives for this long, default: 10.0 (production `params.yaml` sets 20.0)
- `mission_retry_sec`: how often to retry sending the goal until accepted, default: 5.0

**Interfaces:**
- Action Client: `/des_data` (DesData action) - Navigation goal
- Service Client: `/srv_spd_limit` (SpdLimit service) - Speed limit command

#### MissionMonitoringNode (mission_monitoring_node_pc)
Continuous telemetry monitoring via Domain 4 relay.

**Responsibilities:**
- Subscribe to `/tpc_telemetry_relay` on Domain 4 (aggregated from RPi)
- Display formatted telemetry status every 1 second
- Monitor mission progress and rover state from the relay stream
- **Does NOT participate in Domain 5** (no STM32 memory cost)

**Subscriptions:**
- `/tpc_telemetry_relay` (TelemetryRelay, Domain 4) — all rover state at 5 Hz

### Message Interfaces

The package uses custom message types defined in `msgs_ifaces`:

- **DesData** (action): Destination navigation goal
  - `des_lat`: Target latitude
  - `des_long`: Target longitude
  - Feedback: `dis_remain` - Remaining distance
  - Result: `result_fser` - Result string

- **SpdLimit** (service): Speed limit command
  - Request: `rover_spd` - Speed 0-100%
  - Response: `spd_result` - Result string

- **TelemetryRelay** (topic, `/tpc_telemetry_relay`, Domain 4): the only
  message `mission_monitoring_node_pc` subscribes to. This package does not
  touch the raw D5 types (`SpresenseGNSS`, `ChassisCtrl`, etc.) directly —
  those are aggregated into `TelemetryRelay` by `mission_monitoring_node_rpi`
  on the RPi. Key fields: `mission_active`, `distance_remaining_km`,
  `ublox_*`/`spresense_*` GNSS fields, `chassis_cmd_*`/`chassis_sensors_*`/
  `chassis_imu_*` fields, `lane_*` fields (always false/0 — raw lane data
  stays Domain-6-only by design), `destination_*` fields. See
  `common_ifaces/msgs_ifaces/msg/TelemetryRelay.msg` for the complete list.

## Building

### Prerequisites
- ROS 2 (tested on Iron/Humble)
- rclcpp, rclcpp_action
- Custom interfaces: msgs_ifaces, action_ifaces, services_ifaces

### Build Steps

1. Ensure all dependencies are built first:
   ```bash
   cd /home/yupi/almondmatcha/ws_base
   colcon build --packages-select msgs_ifaces action_ifaces services_ifaces
   ```

2. Build the mission_control package:
   ```bash
   colcon build --packages-select mission_control
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### Build Verification
Successful build produces two executable nodes:
- `mission_command_node` - D5 command and action client
- `mission_monitoring_node_pc` - D4 telemetry display

## Usage

### Running Command Node
```bash
ros2 run mission_control mission_command_node --ros-args \
  -p rover_spd:=50 \
  -p des_lat:=35.6892 \
  -p des_long:=139.6917
```

### Running Monitoring Node
```bash
ros2 run mission_control mission_monitoring_node_pc
```

### Launch Together
```bash
ros2 launch mission_control mission_control.launch.py
```

### With Individual Launch Arguments
```bash
ros2 launch mission_control mission_control.launch.py \
  rover_spd:=75 des_lat:=35.6892 des_long:=139.6917
```

The launch file always loads `config/params.yaml` as the base config; these
arguments override individual values on top of it (see
`launch/mission_control.launch.py` — there is no `params_file:=` argument).

## Parameters Configuration

Edit `config/params.yaml`:

```yaml
mission_command_node:
  ros__parameters:
    rover_spd: 15
    des_lat: 8.007286
    des_long: 101.90203
    action_watchdog_timeout_sec: 20.0
```

## Status Output

The monitoring node prints a full telemetry dashboard on every
`tpc_telemetry_relay` message — event-driven, paced by the RPi relay's 5 Hz
publish rate, not a local timer:

```
[INFO] [mission_monitoring_node_pc]: ╔══════════════════════════════════════════════════════════════╗
[INFO] [mission_monitoring_node_pc]: ║              ROVER TELEMETRY DASHBOARD                       ║
[INFO] [mission_monitoring_node_pc]: ╚══════════════════════════════════════════════════════════════╝
[INFO] [mission_monitoring_node_pc]: ┌─ Mission Status ────────────────────────────────────────────┐
[INFO] [mission_monitoring_node_pc]: │ Active: YES
[INFO] [mission_monitoring_node_pc]: │ Distance Remaining: 0.120 km
[INFO] [mission_monitoring_node_pc]: └─────────────────────────────────────────────────────────────┘
[INFO] [mission_monitoring_node_pc]: ┌─ GNSS Position ─────────────────────────────────────────────┐
[INFO] [mission_monitoring_node_pc]: │ RTK (uBlox F9P):
[INFO] [mission_monitoring_node_pc]: │   Position: 35.68912000°, 139.69181000°
[INFO] [mission_monitoring_node_pc]: │   Fix Quality: RTK Fixed
[INFO] [mission_monitoring_node_pc]: └─────────────────────────────────────────────────────────────┘
[INFO] [mission_monitoring_node_pc]: Data Status: RTK=✓ | GPS=✓ | Sensors=✓ | IMU=✓ | Steering=✓ | Lane=✗
```

See `telemetry_callback()` in `mission_monitoring_node_pc.cpp` for the full
set of sections (Chassis Status, Vision & Navigation).

## Code Structure

### mission_command_node.cpp
- **MissionCommandNode**: Main class
  - `init_parameters()`: Load mission parameters
  - `init_clients()`: Initialize ROS2 clients
  - `send_commands()`: Orchestrate command sequence (first attempt)
  - `send_speed_limit()`: Send speed command via service
  - `send_destination_goal()`: Send navigation goal via action
  - `mission_retry_tick()`: Retry `send_commands()` on a timer until the goal is accepted
  - `watchdog_check()`: Cancel the goal if feedback stalls past `action_watchdog_timeout_sec`
  - `cancel_mission()`: Cancel active mission on shutdown

### mission_monitoring_node_pc.cpp
- **MissionMonitoringNodePc**: Main class
  - Constructor subscribes to `/tpc_telemetry_relay` on Domain 4
  - `telemetry_callback()`: single callback — unpacks the message and prints
    the full status dashboard immediately, once per relay message (no
    separate timer or cached-state display step)

## Data Flow

```
Command Node:
  Load Parameters -> Send Speed Limit -> Send Destination Goal -> Monitor Feedback
  -> (retry on mission_retry_sec until accepted; cancel on watchdog timeout)

Monitoring Node (D4):
  Subscribe /tpc_telemetry_relay -> Display Status (event-driven, per message, ~5 Hz)

Rover Communication:
  Command Node -> [Action/Service] -> Rover
  Rover -> [Topics] -> Monitoring Node
```

## Debugging

### Enable Debug Logging
```bash
ROS_LOG_LEVEL=debug ros2 run mission_control mission_command_node
```

### Check Active Topics
```bash
ros2 topic list
ros2 topic echo /tpc_gnss_mission_remain_dist
```

### Check Actions
```bash
ros2 action list
ros2 action send_goal /des_data action_ifaces/action/DesData "{des_lat: 35.6892, des_long: 139.6917}"
```

## Troubleshooting

**Issue**: Destination action server not available
- Ensure rover is running and publishing to `/des_data` action
- Check: `ros2 action list | grep des_data`

**Issue**: Missing telemetry data
- Verify rover telemetry nodes are running
- Check: `ros2 topic list | grep tpc_`

**Issue**: Speed limit service fails
- Ensure rover speed limit service is active
- Check: `ros2 service list | grep spd_limit`

## Performance

- Command transmission: Non-blocking async
- Monitoring update rate: event-driven, matches the RPi relay's 5 Hz publish rate — no local timer
- Action feedback rate: ~0.5 Hz (every 2 s, from `gnss_mission_monitor_node`) — `action_watchdog_timeout_sec` should stay at least 3x this period

## Dependencies

### Runtime
- rclcpp (ROS2 C++ client library)
- rclcpp_action (ROS2 action client)
- msgs_ifaces (custom message definitions)
- action_ifaces (custom action definitions)
- services_ifaces (custom service definitions)

### Build
- CMake 3.18+
- ament_cmake (ROS2 build system)
- Standard C++17

## File Structure

```
mission_control/
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package metadata
├── README.md                # This file
├── src/
│   ├── mission_command_node.cpp     # D5 command node
│   └── mission_monitoring_node_pc.cpp # D4 telemetry display node
├── config/
│   └── params.yaml          # Parameter configuration
└── launch/
    └── mission_control.launch.py  # Launch file
```

## Future Improvements

- Add mission history logging
- Implement mission replay capability
- Add web dashboard interface
- Support multi-goal mission chains
- Real-time map visualization

## License

Apache 2.0 (per the root [README.md](../../../README.md) — no separate LICENSE file is currently checked into the repository).

## Support

For issues or questions, see [ws_base/docs/](../../docs/) and the root
[README.md](../../../README.md) documentation index.
