# Running ws_base Mission Control Nodes

This guide explains how to run the mission control nodes in ws_base for autonomous rover command and monitoring.

## Quick Start

### 1. Build the Workspace
```bash
cd /home/yupi/almondmatcha/ws_base
colcon build
source install/setup.bash
```

### 2. Run Both Nodes with Default Parameters
```bash
ros2 launch mission_control mission_control.launch.py
```

This starts:
- **mission_command_node**: Sends rover to default destination (0.0, 0.0) at 50% speed
- **mission_monitoring_node**: Displays status every 1 second

### 3. Run with Custom Parameters
```bash
ros2 launch mission_control mission_control.launch.py \
  rover_spd:=75 \
  des_lat:=35.6892 \
  des_long:=139.6917
```

## Nodes Overview

### mission_command_node
**Purpose**: Send mission commands to the rover

**Functionality**:
- Loads mission parameters from ROS2 parameter server
- Sends speed limit command via `/spd_limit` service
- Sends navigation goal via `/des_data` action
- Monitors goal progress through feedback
- Gracefully cancels mission on shutdown

**Parameters**:
- `rover_spd` (int, default: 50) - Speed limit as percentage (0-100%)
- `des_lat` (double, default: 0.0) - Target latitude coordinate
- `des_long` (double, default: 0.0) - Target longitude coordinate

**Run Command**:
```bash
ros2 run mission_control mission_command_node --ros-args \
  -p rover_spd:=75 \
  -p des_lat:=35.6892 \
  -p des_long:=139.6917
```

**Expected Output**:
```
[INFO] [mission_command_node]: Initializing Mission Control Command Node...
[INFO] [mission_command_node]: Loading mission parameters...
[INFO] [mission_command_node]: Mission Parameters Loaded: speed=75%, target=(35.689200, 139.691700)
[INFO] [mission_command_node]: Initializing ROS2 clients...
[INFO] [mission_command_node]: Sending mission commands to rover...
[INFO] [mission_command_node]: Sending speed limit: 75%
[INFO] [mission_command_node]: Speed limit command acknowledged by rover.
[INFO] [mission_command_node]: Sending destination goal: (35.689200, 139.691700)
[INFO] [mission_command_node]: Navigation goal accepted by rover.
[INFO] [mission_command_node]: Distance Remaining: 12.45 km
[INFO] [mission_command_node]: Mission Complete: Navigation successful
```

### mission_monitoring_node
**Purpose**: Display real-time rover telemetry and mission status

**Functionality**:
- Subscribes to rover sensor data and control state
- Updates internal state with latest telemetry
- Displays formatted status every 1 second
- Monitors mission active/inactive state

**Subscriptions**:
- `/tpc_gnss_mission_active` (Bool) - Mission active flag
- `/tpc_gnss_mission_remain_dist` (Float64) - Remaining distance in km
- `/tpc_gnss_spresense` (SpresenseGNSS) - Current GNSS position
- `/tpc_rover_dest_coordinate` (Float64MultiArray) - Target [lat, long]
- `/tpc_chassis_ctrl_d2` (ChassisCtrl) - Rover control state

**Run Command**:
```bash
ros2 run mission_control mission_monitoring_node
```

**Expected Output**:
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

## Configuration Files

### Create mission_params.yaml
Create `ws_base/src/mission_control/config/mission_params.yaml`:

```yaml
mission_command_node:
  ros__parameters:
    rover_spd: 75
    des_lat: 35.6892
    des_long: 139.6917

mission_monitoring_node:
  ros__parameters: {}
```

Then run with:
```bash
ros2 launch mission_control mission_control.launch.py \
  params_file:=src/mission_control/config/mission_params.yaml
```

## Advanced Usage

### Run Nodes in Separate Terminals
**Terminal 1 - Command Node**:
```bash
source /home/yupi/almondmatcha/ws_base/install/setup.bash
ros2 run mission_control mission_command_node
```

**Terminal 2 - Monitoring Node**:
```bash
source /home/yupi/almondmatcha/ws_base/install/setup.bash
ros2 run mission_control mission_monitoring_node
```

### Enable Debug Logging
```bash
ROS_LOG_LEVEL=debug ros2 run mission_control mission_command_node
```

### Monitor Topics
```bash
# List all available topics
ros2 topic list

# Monitor a specific topic
ros2 topic echo /tpc_gnss_mission_remain_dist

# Show topic info
ros2 topic info /tpc_chassis_sensors
```

### Check Available Services and Actions
```bash
# List all services
ros2 service list

# List all actions
ros2 action list

# Call a service manually
ros2 service call /spd_limit services_ifaces/srv/SpdLimit "{rover_spd: 50}"

# Send an action goal manually
ros2 action send_goal /des_data action_ifaces/action/DesData \
  "{des_lat: 35.6892, des_long: 139.6917}"
```

## Troubleshooting

### Issue: "Destination action server not available"
**Solution**: 
- Ensure rover nodes are running
- Check: `ros2 action list | grep des_data`
- Verify rover is publishing DesData action

### Issue: "Speed limit service not available"
**Solution**:
- Ensure rover service node is running
- Check: `ros2 service list | grep spd_limit`
- Run command node with: `ROS_LOG_LEVEL=debug` for more details

### Issue: No telemetry data appearing in monitoring node
**Solution**:
- Verify rover telemetry nodes are running: `ros2 topic list | grep tpc_`
- Check if topics have data: `ros2 topic echo /tpc_gnss_mission_remain_dist`
- Ensure GNSS and sensors are initialized on rover

### Issue: Build fails with missing interfaces
**Solution**:
```bash
# Rebuild interface packages first
colcon build --packages-select msgs_ifaces action_ifaces services_ifaces
# Then rebuild mission_control
colcon build --packages-select mission_control
```

### Issue: "ros2 command not found"
**Solution**:
```bash
source /opt/ros/iron/setup.bash  # or humble/latest ROS2 version
cd /home/yupi/almondmatcha/ws_base
source install/setup.bash
```

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Middleware (DDS)                    │
└─────────────────────────────────────────────────────────────┘
         ↑                                          ↑
         │                                          │
    Commands & Goals                          Telemetry Data
         │                                          │
    ┌────┴────────────┐                   ┌────────┴─────────┐
    │                 │                   │                  │
    │ Command Node    │                   │ Monitoring Node  │
    │ (Sends)         │                   │ (Receives)       │
    └────┬────────────┘                   └────────┬─────────┘
         │                                          │
         │ /spd_limit service                      │
         │ /des_data action goal                   │ /tpc_gnss_mission_active
         │ /des_data action feedback          /tpc_gnss_mission_remain_dist
         │                                    /tpc_gnss_spresense
         │                                    /tpc_rover_dest_coordinate
         │                                    /tpc_chassis_ctrl_d2
         │                                          │
         └──────────────────────────────────────────┘
                          ↑
                 ┌─────────┴────────┐
                 │                  │
              Rover            Other Telemetry
           (Domain 2,4)         Sources
```

## Performance Characteristics

- **Command Node**: Non-blocking async communication
- **Monitoring Node**: Status updates at 1 Hz (configurable)
- **Feedback Rate**: Dependent on rover (typically 10 Hz)
- **Latency**: <100ms for commands (network dependent)

## Integration with Rover Domains

This mission control runs on **Domain 6 (Base Station)** and communicates with:
- **Domain 2** (Chassis Dynamics) - Motor commands
- **Domain 4** (Navigation) - GNSS and mission state
- **Domain 5** (Spresense GNSS) - Position data

Message flow uses **ROS2 DDS (Cyclone DDS)** for distributed communication.

## Next Steps

1. **Start rover nodes** in their respective workspaces:
   - ws_jetson (navigation, GNSS processing)
   - ws_rpi (telemetry aggregation)
   
2. **Run mission control** in ws_base:
   ```bash
   ros2 launch mission_control mission_control.launch.py
   ```

3. **Monitor rover** with:
   ```bash
   ros2 topic list
   ros2 topic echo /tpc_gnss_mission_remain_dist
   ```

4. **Cancel missions** by stopping the command node (Ctrl+C)

## Documentation References

- ROS2 Documentation: https://docs.ros.org/en/latest/
- ROS2 Actions: https://docs.ros.org/en/latest/Concepts/Basic/About-Actions.html
- ROS2 Services: https://docs.ros.org/en/latest/Concepts/Basic/About-Services.html
- ROS2 Topics: https://docs.ros.org/en/latest/Concepts/Basic/About-Topics.html
