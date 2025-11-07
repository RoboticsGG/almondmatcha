# Rover Launch System

## Overview

Centralized launch system for the Almondmatcha rover, managing all ROS2 nodes across multiple domains for optimal sensor fusion and base station communication.

## Architecture

### Domain 5 - Rover Internal (Sensor Fusion Enabled)
All rover-internal nodes operate on Domain 5 for direct, low-latency communication:

```
┌─────────────────────────────────────────────────────────┐
│              Domain 5 (Rover Internal)                  │
├─────────────────────────────────────────────────────────┤
│ ws_rpi (Raspberry Pi):                                  │
│   ├─ node_gnss_spresense         (GNSS positioning)    │
│   ├─ node_gnss_mission_monitor   (Mission tracking)    │
│   ├─ node_chassis_controller     (Motor coordination)  │
│   ├─ node_chassis_imu            (IMU logging)         │
│   ├─ node_chassis_sensors        (Encoder logging)     │
│   └─ node_base_bridge (D5 side)  (Topic subscriber)    │
│                                                          │
│ ws_jetson (Jetson Orin):                                │
│   ├─ camera_stream_node          (D415 RGB/Depth)      │
│   ├─ lane_detection_node         (Vision processing)   │
│   └─ steering_control_node       (PID control)         │
│                                                          │
│ STM32 Boards (mROS2):                                   │
│   ├─ chassis_controller          (Motor+IMU)           │
│   └─ sensors_node                (Encoders+Power)      │
└─────────────────────────────────────────────────────────┘
```

### Domain 2 - Base Station Bridge
Bridge node relays telemetry to ground station:

```
┌─────────────────────────────────────────────────────────┐
│         Domain 2 (Base Station Communication)           │
├─────────────────────────────────────────────────────────┤
│ ws_rpi (Raspberry Pi):                                  │
│   └─ node_base_bridge (D2 side)  (Topic publisher)     │
│                                                          │
│ ws_base (Ground Station):                               │
│   ├─ node_monitoring             (Telemetry display)   │
│   └─ node_commands               (Mission commands)    │
└─────────────────────────────────────────────────────────┘
```

## Usage

### Launch All Rover Nodes

```bash
cd ~/almondmatcha/ws_rpi
source install/setup.bash
ros2 launch rover_launch_system rover_startup.launch.py
```

This launches:
- 5 rover nodes on Domain 5 (GNSS, chassis control, sensors)
- 2 bridge node instances (one on D5, one on D2)

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
source install/setup.bash
export ROS_DOMAIN_ID=2
ros2 run mission_control node_monitoring
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
| **node_base_bridge (D5)** | pkg_base_bridge | Bridge subscriber side | Subscribes: All rover telemetry |

### Domain 2 Nodes (ws_rpi)

| Node | Package | Purpose | Topics |
|------|---------|---------|--------|
| **node_base_bridge (D2)** | pkg_base_bridge | Bridge publisher side | Publishes: All rover telemetry to base |

## Topics Relayed D5 → D2

The bridge node relays these topics from Domain 5 to Domain 2:
- `tpc_chassis_imu` - IMU sensor data (10 Hz)
- `tpc_chassis_sensors` - Encoder/power data (10 Hz)
- `tpc_gnss_spresense` - GNSS position (10 Hz)
- `tpc_chassis_cmd` - Motor commands (50 Hz)
- `tpc_gnss_mission_active` - Mission status (10 Hz)
- `tpc_gnss_mission_remain_dist` - Distance to waypoint (10 Hz)

## Topics Relayed D2 → D5

Commands from base station:
- `tpc_rover_dest_coordinate` - Target waypoint coordinates

## Verification

### Check Domain 5 (Rover)
```bash
export ROS_DOMAIN_ID=5
ros2 node list
ros2 topic list
ros2 topic hz tpc_chassis_imu
ros2 topic hz tpc_gnss_spresense
```

### Check Domain 2 (Base)
```bash
export ROS_DOMAIN_ID=2
ros2 node list
ros2 topic list
ros2 topic echo tpc_chassis_imu
```

### Monitor Bridge Node
```bash
export ROS_DOMAIN_ID=5
ros2 node info /base_bridge_d5_node
```

```bash
export ROS_DOMAIN_ID=2
ros2 node info /base_bridge_d2_node
```

## Network Configuration

### Local Testing (Same Machine)
Both domains will communicate via localhost. No special configuration needed.

### Remote Base Station
For remote ws_base communication:

1. **Network**: Ensure rover (192.168.1.1) and base station are on same subnet
2. **Firewall**: Allow UDP ports 7400-7500 for DDS discovery
3. **DDS Config**: Configure CycloneDDS multicast (optional):

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>auto</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
```

## Troubleshooting

### Nodes not appearing
```bash
# Check if nodes are running
ps aux | grep ros2
ros2 node list
```

### No topics on Domain 2
- Verify bridge nodes are running on both domains
- Check network connectivity for remote base station
- Ensure ROS_DOMAIN_ID is set correctly

### High CPU usage
- Check log output: `tail -f ~/.ros/log/latest/*/stdout.log`
- Reduce topic rates if necessary
- Monitor with: `htop` or `ros2 run demo_nodes_cpp talker --ros-args --log-level debug`

### Bridge not relaying
```bash
# Domain 5 side
export ROS_DOMAIN_ID=5
ros2 topic hz tpc_chassis_imu

# Domain 2 side (separate terminal)
export ROS_DOMAIN_ID=2
ros2 topic hz tpc_chassis_imu

# Both should show ~10 Hz if bridge is working
```

## Log Files

Logs are automatically organized by timestamp:
```
~/almondmatcha/runs/ros_logs/YYYYMMDD_HHMMSS/
├─ spresense_gnss_node/
├─ gnss_mission_monitor_node/
├─ chassis_controller_node/
├─ chassis_imu_node/
├─ chassis_sensors_node/
├─ base_bridge_d5_node/
└─ base_bridge_d2_node/
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
