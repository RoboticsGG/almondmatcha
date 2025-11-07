# ws_base Architecture and Design Documentation

## System Overview

**ws_base** is the Base Station workspace for the autonomous rover system, running on Domain 6. It provides mission control capabilities including command transmission and real-time telemetry monitoring.

## Architecture Layers

```
┌────────────────────────────────────────────────────────────────┐
│                    Application Layer                           │
│         Mission Control Commands & Monitoring                  │
├────────────────────────────────────────────────────────────────┤
│                  ROS2 Communication Layer                       │
│      RclPP (C++) | Actions | Services | Topics                 │
├────────────────────────────────────────────────────────────────┤
│                     DDS Middleware                              │
│    Cyclone DDS | RTPS | Discovery | QoS Policies               │
├────────────────────────────────────────────────────────────────┤
│                   Hardware/Network Layer                        │
│              Ethernet | USB | Serial (Optional)                 │
└────────────────────────────────────────────────────────────────┘
```

## Component Breakdown

### 1. Mission Command Node (node_commands.cpp)
**Type**: ROS2 Executable Node
**Domain**: 6 (Base Station)
**Language**: C++17

**Responsibilities**:
- Load mission parameters from parameter server
- Send rover speed limits via service calls
- Send navigation destinations via action goals
- Process goal feedback (distance remaining)
- Handle graceful shutdown with mission cancellation

**Interface Clients**:
```cpp
rclcpp_action::Client<action_ifaces::action::DesData>  // Destination action
rclcpp::Client<services_ifaces::srv::SpdLimit>         // Speed limit service
```

**Internal State**:
```cpp
int rover_speed_percent_;           // 0-100%
double destination_latitude_;       // Decimal degrees
double destination_longitude_;      // Decimal degrees
DesDataGoalHandle::SharedPtr goal_handle_;  // Current goal reference
```

**Lifecycle**:
1. **Construction**: Initializes logging and ROS2 infrastructure
2. **init_parameters()**: Declare and load parameters from server
3. **init_clients()**: Create action/service clients
4. **send_commands()**: Execute command sequence
5. **Spin**: Wait for feedback and result callbacks
6. **Shutdown**: Cancel active goal, cleanup resources

### 2. Mission Monitoring Node (node_monitoring.cpp)
**Type**: ROS2 Executable Node
**Domain**: 6 (Base Station)
**Language**: C++17

**Responsibilities**:
- Subscribe to rover telemetry topics
- Maintain synchronized internal state
- Display formatted status updates (1 Hz)
- Format rover control commands for human readability

**Topic Subscriptions**:
```cpp
rclcpp::Subscription<std_msgs::msg::Bool>                      // Mission active
rclcpp::Subscription<std_msgs::msg::Float64>                   // Distance remaining
rclcpp::Subscription<msgs_ifaces::msg::SpresenseGNSS>          // Current position
rclcpp::Subscription<std_msgs::msg::Float64MultiArray>         // Target coordinates
rclcpp::Subscription<msgs_ifaces::msg::ChassisCtrl>            // Control state
```

**Internal State**:
```cpp
bool mission_active_;
double distance_remaining_km_;
SpresenseGNSS current_position_;
std::pair<double, double> target_coordinate_;
ChassisCtrl rover_control_state_;
```

**Update Cycle**:
1. Subscribe to all topics
2. Create timer (1 Hz callback)
3. Wait for updates via topic callbacks
4. On timer: format and display current state

## Custom Message Types

### From msgs_ifaces

**SpresenseGNSS** - GNSS position data
```yaml
date: string          # YYYY-MM-DD
time: string          # HH:MM:SS
num_satellites: uint8 # Count of satellites in view
fix: uint8            # Fix quality indicator
latitude: float64     # Decimal degrees
longitude: float64    # Decimal degrees
altitude: float32     # Meters above sea level
```

**ChassisCtrl** - Rover motor control
```yaml
fdr_msg: uint8        # Front direction (1=right, 2=straight, 3=left)
ro_ctrl_msg: float32  # Steering control (0.0-1.0)
spd_msg: uint16       # Speed command (0-255)
bdr_msg: uint8        # Back direction (0=stop, 1=forward, 2=backward)
```

### From action_ifaces

**DesData** - Navigation goal action
```yaml
Goal:
  des_lat: float64    # Target latitude
  des_long: float64   # Target longitude
Feedback:
  dis_remain: float64 # Distance remaining in km
Result:
  result_fser: string # Result status string
```

### From services_ifaces

**SpdLimit** - Speed limit service
```yaml
Request:
  rover_spd: int32    # Speed 0-100%
Response:
  (acknowledgment)
```

## Communication Patterns

### Request-Response Pattern (Service)
```
Command Node              Rover
     │                     │
     ├─ /spd_limit service ─→
     │  (rover_spd: 75)     │
     │                      │ Set motor speed
     │                      │
     │ ← response (ack)  ──┤
     │                     │
```

### Request-Feedback-Result Pattern (Action)
```
Command Node              Rover
     │                     │
     ├─ /des_data goal ────→
     │  (des_lat, des_long) │
     │                      │
     │                    Start navigation
     │                      │
     │ ← feedback ────────┤
     │ (dis_remain: 12.3) │ Every 1-2 seconds
     │                    │
     │ ← feedback ────────┤
     │ (dis_remain: 0.5)  │
     │                    │
     │ ← result ────────┤
     │ (result_fser)     │ Navigation complete
     │                    │
```

### Publish-Subscribe Pattern (Topics)
```
        Rover              Base Station
         │                     │
         │ /tpc_gnss_mission_active
         ├──────────────────────→
         │ /tpc_gnss_mission_remain_dist
         ├──────────────────────→
         │ /tpc_gnss_spresense
         ├──────────────────────→
         │ /tpc_rover_dest_coordinate
         ├──────────────────────→
         │ /tpc_chassis_ctrl_d2
         ├──────────────────────→
         │
   [Monitoring Node updates display]
```

## Multi-Domain Communication

```
Domains Involved:
├─ Domain 2: Chassis Dynamics (Motor Control)
│  └─ /cmd_chassis_d2 ← Command inputs
│  └─ /spd_limit ← Service calls
│
├─ Domain 4: Navigation (GNSS Processing)
│  └─ /des_data ← Action goals
│  └─ /tpc_gnss_mission_* ← Feedback topics
│
├─ Domain 5: Spresense (GNSS Raw Data)
│  └─ /tpc_gnss_spresense ← Position data
│
└─ Domain 6: Mission Control (This workspace)
   └─ command_node ← Sends goals/commands
   └─ monitoring_node ← Receives telemetry
```

## Initialization Sequence

```
1. User starts nodes (manually or via launch)
   │
   ├─ Mission Command Node
   │  ├─ rclcpp::init()
   │  ├─ Load parameters from server
   │  ├─ Create action/service clients
   │  ├─ Wait for servers available (2 sec timeout)
   │  ├─ Send speed limit (service call)
   │  ├─ Send destination goal (action goal)
   │  └─ Enter spin loop (process callbacks)
   │
   └─ Mission Monitoring Node
      ├─ rclcpp::init()
      ├─ Create subscriptions (5 topics)
      ├─ Create timer (1 Hz)
      └─ Enter spin loop (wait for messages)

2. Rover nodes must already be running
   - Domain 2: Motor control active
   - Domain 4: Navigation/GNSS processing
   - Domain 5: GNSS receiver feeding data

3. DDS Discovery
   - Participants announce themselves
   - Endpoints discover each other
   - Connections established

4. Data flow begins
```

## State Machine - Command Node

```
┌─────────────┐
│  START      │
│  (Init)     │
└──────┬──────┘
       │
       ├─ Load parameters
       │
       ├─ Wait for clients ready
       │  ├─ Service available? → Wait (2 sec)
       │  └─ Action server available? → Wait (2 sec)
       │
       ├─ Send speed limit (async)
       │  └─ Wait for response
       │
       ├─ Send destination goal (async)
       │  │
       │  ├─ Goal ACCEPTED → Monitor feedback
       │  │  │
       │  │  ├─ Each feedback → Display distance
       │  │  │
       │  │  └─ Result received
       │  │     ├─ SUCCEEDED → Log result
       │  │     ├─ ABORTED → Log abort
       │  │     └─ CANCELED → Log cancel
       │  │
       │  └─ Goal REJECTED → Error exit
       │
       └─ Spin (process callbacks)
          │
          └─ On shutdown → Cancel goal → Exit
```

## State Machine - Monitoring Node

```
┌──────────────┐
│  START       │
│  (Init)      │
└──────┬───────┘
       │
       ├─ Create 5 subscriptions
       │  └─ Callbacks ready to fire
       │
       ├─ Create 1 Hz timer
       │
       └─ Spin (process callbacks)
          │
          ├─ Topic callback arrives
          │  ├─ Update mission_active
          │  ├─ Update distance_remaining
          │  ├─ Update current_position
          │  ├─ Update target_coordinate
          │  └─ Update rover_control_state
          │
          ├─ Timer fires (every 1 second)
          │  ├─ Format all current state
          │  ├─ Apply rover control formatting
          │  └─ Print status to console
          │
          └─ On shutdown → rclcpp::shutdown() → Exit
```

## Thread Safety

### Data Access Patterns

**Command Node**:
- Single-threaded: Parameter reading → Client creation → Goal sending
- Async callbacks handled by ROS2 executor
- Goal state protected by goal_handle_ smart pointer

**Monitoring Node**:
- Multi-threaded: Topic callbacks + timer callback
- State variables (`mission_active_`, `distance_remaining_km_`, etc.) are atomics implicitly
- ROS2 executor serializes callback execution per node

### Mutex Usage
- **Not required**: ROS2 handles serialization per-node
- **Recommended if**: Accessing shared state from multiple threads (external code)

## Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| Command latency | <100ms | Service + Action async calls |
| Monitoring update rate | 1 Hz | Configurable via timer |
| Feedback rate | Rover-dependent | Typically 10 Hz from rover |
| Topic subscription latency | <50ms | ROS2 middleware dependent |
| Memory footprint | ~2-5 MB | Per node |
| CPU usage | <1% | Mostly idle, event-driven |

## Error Handling Strategy

### Service Failures
```cpp
if (!spd_limit_client_->wait_for_service(chrono::seconds(2))) {
    RCLCPP_WARN(..., "Service not available. Continuing anyway...");
    return;  // Non-fatal, continue
}
```

### Action Server Failures
```cpp
if (!des_action_client_->wait_for_action_server(chrono::seconds(2))) {
    RCLCPP_ERROR(..., "Action server not available.");
    return;  // Fatal, node continues but no navigation
}
```

### Callback Exceptions
```cpp
try {
    auto response = future.get();
    // Process response
} catch (const std::exception &e) {
    RCLCPP_ERROR(..., "Service call failed: %s", e.what());
}
```

## Launch System

### Direct Execution
```bash
ros2 run mission_control mission_command_node --ros-args -p rover_spd:=75
```

### Launch File Execution
```bash
ros2 launch mission_control mission_control.launch.py rover_spd:=75
```

**Launch File** (`node_comlaunch.py`):
1. Declares launch arguments
2. Creates two Node actions
3. Sets parameters via LaunchConfiguration
4. Returns LaunchDescription

## Testing Scenarios

### Scenario 1: Basic Navigation
```bash
# Terminal 1: Rover nodes (simulated)
ros2 run simulator rover_simulator

# Terminal 2: Mission control
ros2 launch mission_control mission_control.launch.py \
  rover_spd:=50 des_lat:=35.6892 des_long:=139.6917

# Expected: Speed set, goal sent, feedback received, navigation completes
```

### Scenario 2: Monitoring Only
```bash
# Terminal 1: Rover nodes
ros2 run simulator rover_simulator

# Terminal 2: Monitor only (no commands)
ros2 run mission_control mission_monitoring_node

# Expected: Real-time status display without sending commands
```

### Scenario 3: Service Failures
```bash
# Start monitoring node without rover
ros2 run mission_control mission_monitoring_node

# Expected: Waits for topics, displays defaults, no errors
```

## Integration Points

### With Domain 2 (Chassis Dynamics)
- **Input**: `/spd_limit` service, speed percentage (0-100)
- **Output**: Motor commands forwarded to physical motors
- **Protocol**: ROS2 Service (request-response)

### With Domain 4 (Navigation)
- **Input**: `/des_data` action goal with destination
- **Output**: Feedback on distance remaining, final result
- **Protocol**: ROS2 Action (request-feedback-result)

### With Domain 5 (Spresense GNSS)
- **Input**: `/tpc_gnss_spresense` topics with GPS position
- **Output**: None (monitoring only)
- **Protocol**: ROS2 Topic (publish-subscribe)

## Configuration Management

### Parameter Hierarchy
1. **Default values** (in code): `rover_spd: 50`, `des_lat: 0.0`
2. **Launch arguments** (override defaults): Passed via `--ros-args`
3. **Parameter file** (YAML, highest priority): `mission_params.yaml`

### Example Configuration
```yaml
mission_command_node:
  ros__parameters:
    rover_spd: 75
    des_lat: 35.6892
    des_long: 139.6917
```

## Debugging and Monitoring

### ROS2 CLI Tools
```bash
# List all nodes
ros2 node list

# Show node info
ros2 node info /mission_command_node

# List topics
ros2 topic list

# Monitor topic
ros2 topic echo /tpc_gnss_mission_remain_dist

# Check services
ros2 service list

# Call service manually
ros2 service call /spd_limit services_ifaces/srv/SpdLimit "{rover_spd: 50}"
```

### Logging Levels
```bash
# Debug mode
ROS_LOG_LEVEL=debug ros2 run mission_control mission_command_node

# Info mode (default)
ROS_LOG_LEVEL=info ros2 run mission_control mission_command_node
```

## Future Enhancements

1. **Mission History**: Log all sent commands and results
2. **Replay System**: Re-execute previous missions
3. **Web Dashboard**: Browser-based control interface
4. **Multi-Goal Chains**: Support sequential destinations
5. **Map Visualization**: Real-time rover position overlay
6. **Obstacle Avoidance**: Dynamic waypoint adjustment
7. **Telemetry Recording**: Data logging for post-analysis
8. **Autonomous Charging**: Return-to-base when low battery

## Maintenance Notes

### Regular Updates
- Check ROS2 updates for compatibility
- Update interface definitions if rover domains change
- Review performance metrics quarterly

### Common Issues and Fixes
| Issue | Cause | Fix |
|-------|-------|-----|
| Nodes don't connect | Wrong domain ID | Verify ROS_DOMAIN_ID environment variable |
| No telemetry | Rover not running | Start rover domain nodes first |
| Service timeout | Service not implemented | Check rover node implementation |
| Action rejected | Invalid parameters | Validate lat/long range |

## References

- [ROS2 Concepts](https://docs.ros.org/en/latest/Concepts/Basic/About-Nodes.html)
- [ROS2 Actions](https://docs.ros.org/en/latest/Concepts/Basic/About-Actions.html)
- [ROS2 Services](https://docs.ros.org/en/latest/Concepts/Basic/About-Services.html)
- [rclcpp Documentation](https://docs.ros.org/en/latest/p/rclcpp/)
