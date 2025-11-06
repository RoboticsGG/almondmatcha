# Base Bridge Node (pkg_base_bridge)

## Purpose

Multi-domain communication bridge between:
- **Domain 5** (Rover Internal): All rover sensor and control nodes
- **Domain 2** (Base Station): Ground station monitoring and command nodes

## Architecture

```
Domain 5 (Rover)                Domain 2 (Base Station)
┌────────────────┐             ┌──────────────────┐
│  Rover Nodes   │             │   Base Nodes     │
│  - chassis_*   │             │   - monitoring   │
│  - gnss_*      │             │   - commands     │
│  - vision_*    │             │                  │
└────────┬───────┘             └────────┬─────────┘
         │                              │
         │   ┌──────────────────┐      │
         └──►│  Bridge Node     │◄─────┘
             │  (Run on both    │
             │   domains)       │
             └──────────────────┘
```

## Topics Relayed

### Domain 5 → Domain 2 (Telemetry)
- `tpc_chassis_imu` - IMU sensor data (10 Hz)
- `tpc_chassis_sensors` - Encoder/power data (10 Hz)
- `tpc_gnss_spresense` - GNSS position (10 Hz)
- `tpc_chassis_cmd` - Motor commands (50 Hz)
- `tpc_gnss_mission_active` - Mission status (10 Hz)
- `tpc_gnss_mission_remain_dist` - Distance to waypoint (10 Hz)

### Domain 2 → Domain 5 (Commands)
- `tpc_rover_dest_coordinate` - Target waypoint coordinates

## Usage

### Method 1: DDS Domain Routing (Recommended)

Configure CycloneDDS to route topics between domains using XML config:

```xml
<!-- cyclonedds_bridge.xml -->
<CycloneDDS>
  <Domain id="5">
    <Tracing><Verbosity>info</Verbosity></Tracing>
  </Domain>
  <Domain id="2">
    <Tracing><Verbosity>info</Verbosity></Tracing>
  </Domain>
  <!-- Domain routing rules -->
  <DomainRoute from="5" to="2">
    <Topic>tpc_chassis_imu</Topic>
    <Topic>tpc_chassis_sensors</Topic>
    <Topic>tpc_gnss_spresense</Topic>
    <Topic>tpc_chassis_cmd</Topic>
    <Topic>tpc_gnss_mission_active</Topic>
    <Topic>tpc_gnss_mission_remain_dist</Topic>
  </DomainRoute>
  <DomainRoute from="2" to="5">
    <Topic>tpc_rover_dest_coordinate</Topic>
  </DomainRoute>
</CycloneDDS>
```

Then run:
```bash
export CYCLONEDDS_URI=file://$(pwd)/cyclonedds_bridge.xml
export ROS_DOMAIN_ID=5
ros2 run pkg_base_bridge node_base_bridge
```

### Method 2: Dual Process Bridge

Run the bridge node twice with different domains:

**Terminal 1 (Domain 5 - Rover side):**
```bash
cd ~/almondmatcha/ws_rpi
source install/setup.bash
export ROS_DOMAIN_ID=5
ros2 run pkg_base_bridge node_base_bridge
```

**Terminal 2 (Domain 2 - Base side):**
```bash
cd ~/almondmatcha/ws_rpi
source install/setup.bash
export ROS_DOMAIN_ID=2
ros2 run pkg_base_bridge node_base_bridge
```

The two processes will relay topics between domains via local loopback.

### Method 3: Launch File Integration

Add to rover startup launch file:

```python
# In rover_startup.launch.py
Node(
    package='pkg_base_bridge',
    executable='node_base_bridge',
    name='base_bridge_node',
    output='screen',
    parameters=[{'use_sim_time': False}],
    environment={'ROS_DOMAIN_ID': '5'}
),
```

## Verification

### Check Domain 5 Topics (Rover)
```bash
export ROS_DOMAIN_ID=5
ros2 topic list
ros2 topic hz tpc_chassis_imu
ros2 topic hz tpc_gnss_spresense
```

### Check Domain 2 Topics (Base Station)
```bash
export ROS_DOMAIN_ID=2
ros2 topic list
ros2 topic echo tpc_chassis_imu
ros2 topic echo tpc_gnss_spresense
```

### Monitor Bridge Activity
```bash
export ROS_DOMAIN_ID=5
ros2 node info /base_bridge_node
```

## Network Configuration

For remote base station communication:

1. **Same Network**: Ensure rover and base station are on same subnet
   - Rover: 192.168.1.1
   - Base: 192.168.1.x

2. **Firewall**: Allow UDP ports 7400-7500 for DDS discovery

3. **DDS Config**: Set multicast or unicast peers in CycloneDDS config

## Troubleshooting

### No topics visible on Domain 2
- Check `ROS_DOMAIN_ID` is set correctly
- Verify bridge node is running: `ros2 node list`
- Check network connectivity: `ping <base_station_ip>`

### High latency
- Use wired connection instead of WiFi
- Reduce topic rates for non-critical data
- Consider aggregating topics into single telemetry message

### Topics not relaying
- Verify topic names match exactly (no leading slashes)
- Check message types match between domains
- Monitor bridge node logs for errors

## Future Enhancements

- [ ] Aggregated telemetry message (reduce bandwidth)
- [ ] QoS profile optimization for reliability
- [ ] Bandwidth monitoring and throttling
- [ ] Health check heartbeat topic
- [ ] Command acknowledgment feedback

## Dependencies

- rclcpp
- std_msgs
- msgs_ifaces (custom message definitions)

## Author

Almondmatcha Development Team  
Date: November 6, 2025
