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
- Monitor goal progress through feedback
- Handle mission cancellation on shutdown

**Parameters:**
- `rover_spd`: Speed limit as percentage (0-100), default: 50
- `des_lat`: Target latitude coordinate, default: 0.0
- `des_long`: Target longitude coordinate, default: 0.0

**Interfaces:**
- Action Client: `/des_data` (DesData action) - Navigation goal
- Service Client: `/spd_limit` (SpdLimit service) - Speed limit command

#### MissionMonitoringNode (mission_monitoring_node)
Continuous telemetry monitoring and status reporting.

**Responsibilities:**
- Subscribe to rover sensor data (GNSS, control state)
- Track mission progress and remaining distance
- Display formatted status updates every 1 second
- Monitor mission active/inactive state

**Subscriptions:**
- `/tpc_gnss_mission_active` (Bool) - Mission active flag
- `/tpc_gnss_mission_remain_dist` (Float64) - Remaining distance (km)
- `/tpc_gnss_spresense` (SpresenseGNSS) - Current GPS position
- `/tpc_rover_dest_coordinate` (Float64MultiArray) - Target [lat, long]
- `/tpc_chassis_ctrl_d2` (ChassisCtrl) - Rover control state

### Message Interfaces

The package uses custom message types defined in `msgs_ifaces`:

- **DesData** (action): Destination navigation goal
  - `des_lat`: Target latitude
  - `des_long`: Target longitude
  - Feedback: `dis_remain` - Remaining distance
  - Result: `result_fser` - Result string

- **SpdLimit** (service): Speed limit command
  - Request: `rover_spd` - Speed 0-100%
  - Response: (acknowledgment)

- **SpresenseGNSS**: GNSS position from Sony Spresense board
  - `date`: Date string (YYYY-MM-DD)
  - `time`: Time string (HH:MM:SS)
  - `num_satellites`: Number of satellites in view
  - `fix`: GNSS fix status
  - `latitude`: Current latitude (decimal degrees)
  - `longitude`: Current longitude (decimal degrees)
  - `altitude`: Altitude above mean sea level (meters)

- **ChassisCtrl**: Rover control commands
  - `fdr_msg`: Front direction (1=right, 2=straight, 3=left)
  - `ro_ctrl_msg`: Steering control value (0.0 to 1.0)
  - `spd_msg`: Speed command (0-255)
  - `bdr_msg`: Back direction (0=stop, 1=forward, 2=backward)

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
- `mission_command_node` - Command transmission and monitoring
- `mission_monitoring_node` - Telemetry display

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
ros2 run mission_control mission_monitoring_node
```

### Launch Together
```bash
ros2 launch mission_control mission_control.launch.py
```

### With ROS2 Parameters File
```bash
ros2 launch mission_control mission_control.launch.py \
  params_file:=src/mission_control/config/mission_params.yaml
```

## Parameters Configuration

Create `config/mission_params.yaml`:

```yaml
mission_command_node:
  ros__parameters:
    rover_spd: 75
    des_lat: 35.6892
    des_long: 139.6917

mission_monitoring_node:
  ros__parameters: {}
```

## Status Output

The monitoring node displays comprehensive status every 1 second:

```
[INFO] [mission_monitoring_node]: ============ MISSION STATUS ============
[INFO] [mission_monitoring_node]: Status: ACTIVE
[INFO] [mission_monitoring_node]: 
[INFO] [mission_monitoring_node]: --- Navigation ---
[INFO] [mission_monitoring_node]: Target: Lat: 35.689200, Long: 139.691700
[INFO] [mission_monitoring_node]: Current: Lat: 35.689120, Long: 139.691810
[INFO] [mission_monitoring_node]: Distance Remaining: 0.12 km
[INFO] [mission_monitoring_node]: 
[INFO] [mission_monitoring_node]: --- Rover Control ---
[INFO] [mission_monitoring_node]: Steering: Maintain Course
[INFO] [mission_monitoring_node]: Movement: Forward at 75%
[INFO] [mission_monitoring_node]: ========================================
```

## Code Structure

### node_commands.cpp
- **MissionCommandNode**: Main class
  - `init_parameters()`: Load mission parameters
  - `init_clients()`: Initialize ROS2 clients
  - `send_commands()`: Orchestrate command sequence
  - `send_speed_limit()`: Send speed command via service
  - `send_destination_goal()`: Send navigation goal via action
  - `cancel_mission()`: Cancel active mission on shutdown

### node_monitoring.cpp
- **MissionMonitoringNode**: Main class
  - `init_subscriptions()`: Create topic subscriptions
  - `init_timer()`: Setup status update timer
  - Topic callbacks: `on_mission_active()`, `on_distance_remaining()`, etc.
  - `format_steering_command()`: Display-ready steering string
  - `format_movement_command()`: Display-ready movement string
  - `publish_status_update()`: Print periodic status

## Data Flow

```
Command Node:
  Load Parameters -> Send Speed Limit -> Send Destination Goal -> Monitor Feedback

Monitoring Node:
  Subscribe Topics -> Update Internal State -> Display Status (1 Hz)

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
- Monitoring update rate: 1 Hz (configurable via timer)
- Feedback rate: Rover-dependent (typically 10 Hz)

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
├── package.xml              # Package metadata (v1.0.0)
├── README.md                # This file
├── src/
│   ├── node_commands.cpp    # Command node (222 lines)
│   └── node_monitoring.cpp  # Monitoring node (315 lines)
├── config/
│   └── mission_params.yaml  # Parameter configuration
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

See LICENSE file in workspace root.

## Support

For issues or questions, refer to workspace documentation in WORK_SESSION files.
