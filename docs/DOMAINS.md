# ROS2 Domain Architecture

Almondmatcha rover system uses a two-domain ROS2 architecture optimized for centralized sensor fusion.

## Domain Assignment

| Domain ID | Purpose | Nodes | Rationale |
|-----------|---------|-------|-----------|
| **5** | Rover-internal processing | All rover nodes (RPi, Jetson, STM32s) | Direct low-latency sensor access for EKF fusion |
| **2** | Base station bridge | ws_base ↔ node_base_bridge | Isolated telemetry/command interface |

## Why Two Domains?

### Problem: Fragmented Sensor Access

Previous architectures had sensors spread across multiple domains:
- Vision on Domain 2 (Jetson)
- IMU/chassis on Domain 5 (STM32)
- GNSS on Domain 2 (RPi)

**Issues:**
- Cross-domain bridging added 10-50 ms latency per hop
- EKF sensor fusion required bridge nodes to relay data
- Complex multi-domain synchronization
- Difficult to add new sensors

### Solution: Centralized Domain 5

All rover-internal nodes communicate on Domain 5:

**Benefits:**
1. **Direct Sensor Access:** EKF node subscribes directly to all sensors
2. **Low Latency:** No bridge relay overhead
3. **Unified Timestamps:** All sensors on same domain for synchronized fusion
4. **Scalability:** Easy to add new sensors without architecture changes
5. **Atomic Updates:** Transaction-like semantics for multi-sensor fusion cycles

### Domain 2 Bridge

Single bridge node (`node_base_bridge`) isolates base station:

**Benefits:**
1. **Security:** Base station cannot directly command motors
2. **Bandwidth:** Telemetry aggregated before transmission
3. **Reliability:** Rover continues operation if base disconnects
4. **Multi-Rover:** Base can monitor multiple rovers on different domains

## Domain Configuration

### Setting Domain ID

**RPi/Jetson Nodes (Domain 5):**
```bash
export ROS_DOMAIN_ID=5
ros2 run <package> <node>
```

**Base Station (Domain 2):**
```bash
export ROS_DOMAIN_ID=2
ros2 run mission_control <node>
```

**Bridge Node (Multi-Domain):**
```bash
# node_base_bridge internally creates two ROS contexts:
# - Primary context on Domain 2 (for base station topics)
# - Secondary context on Domain 5 (for rover topics)
# Launch file handles configuration automatically
```

**STM32 Firmware (Domain 5):**
```cpp
// In app.cpp
setenv("ROS_DOMAIN_ID", "5", 5);

// In platform/rtps/config.h
const uint8_t DOMAIN_ID = 5;
```

### Verifying Domain Configuration

**Check Active Nodes:**
```bash
# Domain 5 (rover)
export ROS_DOMAIN_ID=5
ros2 node list

# Expected output:
# /camera_stream
# /lane_detection
# /steering_control
# /node_chassis_controller
# /node_chassis_imu
# /node_chassis_sensors
# /node_gnss_spresense
# /node_gnss_mission_monitor
```

```bash
# Domain 2 (base)
export ROS_DOMAIN_ID=2
ros2 node list

# Expected output:
# /node_base_bridge (visible on both domains)
# /mission_control
# /mission_monitoring
```

**Check Topics:**
```bash
# Domain 5
export ROS_DOMAIN_ID=5
ros2 topic list

# Should see all rover topics (tpc_rover_*, tpc_chassis_*, tpc_gnss_*)
```

## Network Discovery

### DDS Discovery Mechanism

ROS2 uses DDS (Data Distribution Service) for inter-node communication:

1. **Multicast Discovery (Default):**
   - Nodes broadcast presence on multicast group `239.255.0.1`
   - All nodes on same LAN and domain discover each other
   - No central broker required

2. **Domain Isolation:**
   - Domain ID embedded in multicast packets
   - Nodes ignore discovery packets from other domains
   - Perfect isolation even on same physical network

3. **Participant Matching:**
   - Publishers and subscribers matched by topic name AND data type
   - QoS policies must be compatible
   - Automatic connection establishment

### Firewall Configuration

Ensure firewall allows DDS traffic:

```bash
# Ubuntu (UFW)
sudo ufw allow from 192.168.1.0/24  # Allow all traffic from rover network

# Or disable for testing
sudo ufw disable
```

**Required Ports:**
- UDP 7400-7500: DDS discovery and data exchange
- Multicast: 239.255.0.1 (RTPS discovery)

## Domain Migration History

### Evolution

**v1.0 (Initial):** Three domains (2, 5, 6)
- Domain 2: Base station + high-level rover control
- Domain 5: IMU/chassis STM32
- Domain 6: Sensors/GNSS STM32
- **Issue:** Required domain bridge, complex synchronization

**v2.0 (Current):** Two domains (2, 5)
- Domain 2: Base station bridge only
- Domain 5: All rover-internal processing
- **Benefit:** Direct sensor access, low latency, simplified architecture

**Rationale for Consolidation:**
- mROS2 limitation is per-board participant count (10 max), not per-domain
- Multiple STM32 boards can share same domain without resource conflicts
- Sensor fusion requires all sensors on same domain

See `docs/archive/DOMAIN_CONSOLIDATION_SUMMARY.md` for detailed migration history.

## Multi-Rover Considerations

### Scaling to Multiple Rovers

Each rover uses dedicated Domain ID:

| Rover | Domain ID | IP Subnet |
|-------|-----------|-----------|
| Rover 1 | 5 | 192.168.1.0/24 |
| Rover 2 | 8 | 192.168.2.0/24 |
| Rover 3 | 11 | 192.168.3.0/24 |
| Base | 2, 8, 11 | All subnets |

**Base Station Configuration:**
```bash
# Monitor rover 1
export ROS_DOMAIN_ID=5
ros2 topic list

# Switch to rover 2
export ROS_DOMAIN_ID=8
ros2 topic list
```

**Fleet Management:**
- Separate ROS contexts per rover in base station software
- Aggregate telemetry from all domains
- Command individual rovers via specific domain

## Troubleshooting

### Topics Not Visible

**Symptom:** `ros2 topic list` shows no topics or missing topics

**Solutions:**
1. Verify domain ID matches:
   ```bash
   echo $ROS_DOMAIN_ID  # Should be 5 for rover, 2 for base
   ```

2. Check network connectivity:
   ```bash
   ping 192.168.1.1  # RPi
   ping 192.168.1.5  # Jetson
   ping 192.168.1.2  # Chassis STM32
   ping 192.168.1.6  # Sensors STM32
   ```

3. Restart ROS2 daemon:
   ```bash
   ros2 daemon stop
   ros2 daemon start
   ```

4. Check DDS environment:
   ```bash
   ros2 doctor  # Diagnose DDS issues
   ```

### Cross-Domain Communication Fails

**Symptom:** Bridge node not relaying topics between domains

**Solutions:**
1. Verify bridge node running:
   ```bash
   ros2 node list  # Should see /node_base_bridge on both domains
   ```

2. Check bridge node logs:
   ```bash
   ros2 topic echo /rosout | grep base_bridge
   ```

3. Restart bridge node:
   ```bash
   ros2 run pkg_chassis_control node_base_bridge
   ```

### STM32 Not Visible on Network

**Symptom:** STM32 topics not appearing in `ros2 topic list`

**Solutions:**
1. Check serial console (115200 baud) for initialization:
   ```
   [MROS2_INFO] Network connected successfully
   [MROS2_INFO] ROS_DOMAIN_ID: 5
   [MROS2_INFO] Publisher initialized: tpc_chassis_imu
   ```

2. Verify Domain ID in STM32 firmware:
   ```cpp
   // app.cpp
   setenv("ROS_DOMAIN_ID", "5", 5);  // Must match RPi/Jetson
   ```

3. Rebuild and reflash firmware if domain mismatch

### Multicast Not Working

**Symptom:** Nodes on different machines don't discover each other

**Solutions:**
1. Use unicast discovery (static peers):
   ```bash
   export ROS_STATIC_PEERS=192.168.1.1,192.168.1.5
   ```

2. Check switch supports multicast (some switches block multicast by default)

3. Use `tcpdump` to verify multicast traffic:
   ```bash
   sudo tcpdump -i eth0 multicast
   ```

## Best Practices

### Development

1. **Use Correct Domain:** Always set `ROS_DOMAIN_ID` before running ROS2 commands
2. **Separate Terminals:** Use different terminals for different domains (avoid switching)
3. **Verify Before Deploy:** Check topics/nodes with `ros2 topic list` and `ros2 node list`

### Production

1. **Single Domain for Rover:** Keep all rover-internal nodes on Domain 5
2. **Dedicated Bridge:** Run `node_base_bridge` as system service
3. **Monitor Health:** Log bridge relay statistics for diagnostics

### Debugging

1. **Use `ros2 doctor`:** Diagnose DDS configuration issues
2. **Check Logs:** Monitor `/rosout` topic for error messages
3. **Network Tools:** Use `tcpdump`, `wireshark` for deep packet inspection

---

**See Also:**
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture overview
- [TOPICS.md](TOPICS.md) - Complete topic reference
- `docs/archive/SENSOR_FUSION_ARCHITECTURE.md` - Sensor fusion design details
