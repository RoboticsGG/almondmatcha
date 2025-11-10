# Rover Launch System

## Overview

Centralized launch system for the Almondmatcha rover, managing all ROS2 nodes on unified Domain 5 for seamless communication across all systems (rover, base station, vision).

## Architecture

### Domain 5 - Unified Architecture
All systems operate on Domain 5 for direct, low-latency communication without bridges:

```
┌─────────────────────────────────────────────────────────┐
│              Domain 5 (Unified System)                  │
├─────────────────────────────────────────────────────────┤
│ ws_rpi (Raspberry Pi):                                  │
│   ├─ node_gnss_spresense         (GNSS positioning)    │
│   ├─ node_gnss_mission_monitor   (Mission tracking)    │
│   ├─ node_chassis_controller     (Motor coordination)  │
│   ├─ node_chassis_imu            (IMU logging)         │
│   └─ node_chassis_sensors        (Encoder logging)     │
│                                                          │
│ ws_jetson (Jetson Orin):                                │
│   ├─ camera_stream_node          (D415 RGB/Depth)      │
│   ├─ lane_detection_node         (Vision processing)   │
│   └─ steering_control_node       (PID control)         │
│                                                          │
│ ws_base (Ground Station):                               │
│   ├─ mission_command_node        (Send goals/speed)    │
│   └─ mission_monitoring_node     (Telemetry display)   │
│                                                          │
│ STM32 Boards (mROS2):                                   │
│   ├─ chassis_dynamics            (Motor+IMU)           │
│   └─ sensors_gnss                (Encoders+GNSS)       │
└─────────────────────────────────────────────────────────┘
```

**Key Benefits:**
- Direct DDS/RTPS discovery - no bridge needed
- Native action/service support (ws_base ↔ ws_rpi)
- Lower latency (no relay overhead)
- Simplified architecture

## Usage

### Launch All Rover Nodes

```bash
cd ~/almondmatcha/ws_rpi

# Using tmux (recommended)
./launch_rover_tmux.sh

# Or using ros2 launch
source install/setup.bash
ros2 launch rover_launch_system rover_startup.launch.py
```

This launches 5 rover nodes on Domain 5:
- GNSS Spresense (positioning)
- GNSS Mission Monitor (waypoint tracking)
- Chassis Controller (motor coordination)
- Chassis IMU (logging)
- Chassis Sensors (logging)

### Launch with ws_jetson (Vision System)

On Jetson Orin Nano:
```bash
cd ~/almondmatcha/ws_jetson
source install/setup.bash
export ROS_DOMAIN_ID=5
ros2 launch jetson_launch_system jetson_startup.launch.py
```

### Launch ws_base (Ground Station)

On base station computer:
```bash
cd ~/almondmatcha/ws_base

# Using screen
./launch_base_screen.sh

# Or using tmux
./launch_base_tmux.sh

# Or manual
source install/setup.bash
export ROS_DOMAIN_ID=5
ros2 run mission_control mission_command_node
ros2 run mission_control mission_monitoring_node
```

## Node Details

### Domain 5 Nodes (ws_rpi)

| Node | Package | Purpose | Topics |
|------|---------|---------|--------|
| **node_gnss_spresense** | pkg_gnss_navigation | Sony Spresense GNSS reader | Publishes: `tpc_gnss_spresense` |
| **node_gnss_mission_monitor** | pkg_gnss_navigation | Waypoint tracking | Subscribes: `tpc_gnss_spresense`<br>Publishes: `tpc_gnss_mission_active`, `tpc_gnss_mission_remain_dist` |
| **node_chassis_controller** | pkg_chassis_control | Motor command coordination | Subscribes: `tpc_rover_fmctl`, `tpc_gnss_mission_active`<br>Publishes: `tpc_chassis_cmd` |
| **node_chassis_imu** | pkg_chassis_sensors | IMU data logger | Subscribes: `tpc_chassis_imu` |
| **node_chassis_sensors** | pkg_chassis_sensors | Encoder/power logger | Subscribes: `tpc_chassis_sensors` |

### Domain 5 Nodes (ws_base)

| Node | Package | Purpose | Topics |
|------|---------|---------|--------|
| **mission_command_node** | mission_control | Send navigation goals and speed limits | Action: `/des_data`<br>Service: `/srv_spd_limit` |
| **mission_monitoring_node** | mission_control | Display telemetry | Subscribes: All rover telemetry topics |

## Communication (Domain 5 - Direct)

All communication happens directly on Domain 5 without bridges:

**Actions (ws_base → ws_rpi):**
- `/des_data` - Navigation goals (latitude/longitude)

**Services (ws_base → ws_rpi):**
- `/srv_spd_limit` - Speed limit commands (0-100%)

**Topics (ws_rpi → all):**
- `tpc_chassis_imu` - IMU sensor data (10 Hz)
- `tpc_chassis_sensors` - Encoder/power data (10 Hz)
- `tpc_gnss_spresense` - GNSS position (10 Hz)
- `tpc_chassis_cmd` - Motor commands (50 Hz)
- `tpc_gnss_mission_active` - Mission status (10 Hz)
- `tpc_gnss_mission_remain_dist` - Distance to waypoint (10 Hz)

**Topics (ws_base → ws_rpi):**
- `tpc_rover_dest_coordinate` - Target waypoint coordinates

## Verification

### Check Domain 5 (All Systems)
```bash
export ROS_DOMAIN_ID=5
ros2 node list          # Should show all rover + base + vision nodes
ros2 topic list         # Should show all topics
ros2 topic hz tpc_chassis_imu
ros2 topic hz tpc_gnss_spresense
ros2 action list        # Should show /des_data
ros2 service list       # Should show /srv_spd_limit
```

### Monitor Specific Topics
```bash
export ROS_DOMAIN_ID=5
ros2 topic echo tpc_chassis_imu
ros2 topic echo tpc_gnss_spresense
```

## Network Configuration

### Local Testing (Same Network)
All systems on Domain 5 communicate via DDS discovery. Ensure all devices on same subnet.

### Remote Base Station
For remote ws_base communication:

1. **Network**: Ensure rover (192.168.1.x) and base station are on same subnet or routable
2. **Firewall**: Allow UDP ports 7400-7500 for DDS discovery
3. **DDS Config**: Configure CycloneDDS multicast (optional):

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>auto</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
```

### STM32 Boards
- **Chassis**: 192.168.1.2 (Domain 5, GUID ...10,13)
- **GNSS**: 192.168.1.6 (Domain 5, GUID ...10,12)
- Both boards configured for 16 participants max

## Troubleshooting

### Nodes not appearing
```bash
# Check if nodes are running
ps aux | grep ros2
export ROS_DOMAIN_ID=5
ros2 node list
```

### No topics visible
- Verify all systems set to Domain 5: `echo $ROS_DOMAIN_ID`
- Check network connectivity between devices
- Verify firewall allows DDS ports (UDP 7400-7500)

### STM32 topics missing
- Ensure ws_rpi is running (STM32 needs subscribers to appear)
- Ping STM32 boards: `ping 192.168.1.2` and `ping 192.168.1.6`
- Check serial console for memory errors

### Action/Service not working
```bash
export ROS_DOMAIN_ID=5
ros2 action list        # Should show /des_data
ros2 action info /des_data
ros2 service list       # Should show /srv_spd_limit
ros2 service type /srv_spd_limit
```

### High CPU usage
- Check log output: `tail -f ~/.ros/log/latest/*/stdout.log`
- Reduce topic rates if necessary
- Monitor with: `htop` or `top`

## Log Files

Logs are automatically organized by timestamp:
```
~/almondmatcha/runs/ros_logs/YYYYMMDD_HHMMSS/
├─ spresense_gnss_node/
├─ gnss_mission_monitor_node/
├─ chassis_controller_node/
├─ chassis_imu_node/
└─ chassis_sensors_node/
```

View logs:
```bash
tail -f ~/almondmatcha/runs/ros_logs/latest/*/stdout.log
```

## Future Enhancements

- [ ] Add node_ekf_fusion for Extended Kalman Filter sensor fusion
- [ ] Health monitoring and auto-restart for critical nodes
- [ ] Dynamic topic rate adjustment based on bandwidth
- [ ] Aggregated telemetry message (reduced bandwidth)
- [ ] Launch file parameters for conditional node startup

## Author

Almondmatcha Development Team  
Date: November 7, 2025
