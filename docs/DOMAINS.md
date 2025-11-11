# ROS2 Domain Architecture

Almondmatcha rover system uses a unified Domain 5 architecture for seamless communication across all systems.

## Domain Assignment

| Domain ID | Purpose | Nodes | Rationale |
|-----------|---------|-------|-----------|
| **5** | All systems | ws_rpi, ws_base, ws_jetson, STM32 boards | Direct DDS discovery, native actions/services, low latency |

## Why Unified Domain 5?

### Evolution from Multi-Domain Architecture

Previous architectures had systems spread across multiple domains:
- Vision on Domain 2 (Jetson)
- Rover on Domain 5 (RPi, STM32)
- Base station on Domain 2 (ws_base)

**Issues:**
- Cross-domain bridging added 10-50 ms latency per relay
- Bridge nodes required for telemetry/command relay
- Actions and services didn't work across domains
- Complex multi-domain synchronization
- Additional CPU/network overhead

### Solution: Unified Domain 5

All systems communicate directly on Domain 5:

**Benefits:**
1. **Direct Communication:** All systems discover each other natively via DDS
2. **Low Latency:** No relay overhead - direct point-to-point communication
3. **Native Actions/Services:** ws_base can call actions and services on ws_rpi directly
4. **Unified Timestamps:** All nodes on same domain for synchronized fusion
5. **Scalability:** Easy to add new nodes without bridge configuration
6. **Simplified Architecture:** Fewer components, easier to maintain and debug
7. **Lower CPU Usage:** No bridge relay processing required

## Domain Configuration

### Setting Domain ID

**RPi/Jetson Nodes (Domain 5):**
```bash
export ROS_DOMAIN_ID=5
ros2 run <package> <node>
```

**All Systems Use Domain 5:**
```bash
# ws_rpi (Raspberry Pi)
export ROS_DOMAIN_ID=5
ros2 run pkg_gnss_navigation node_gnss_spresense

# ws_base (Base Station)
export ROS_DOMAIN_ID=5
ros2 run mission_control mission_command_node

# ws_jetson (Vision System)
export ROS_DOMAIN_ID=5
ros2 run vision_navigation camera_stream_node
```

**STM32 Firmware (Domain 5):**
```cpp
// In platform/rtps/config.h
const uint8_t DOMAIN_ID = 5;
```

### Verifying Domain Configuration

**Check Active Nodes:**
```bash
# All systems on Domain 5
export ROS_DOMAIN_ID=5
ros2 node list

# Expected output (all nodes visible):
# /camera_stream
# /lane_detection
# /steering_control
# /node_chassis_controller
# /mission_command_node
# /mission_monitoring_node
# /node_chassis_imu
# /node_chassis_sensors
# /node_gnss_spresense
# /node_gnss_mission_monitor
# /node_gnss_spresense
# /node_gnss_mission_monitor
# /node_chassis_imu
# /node_chassis_sensors
```

**Check Topics:**
```bash
# All topics visible on Domain 5
export ROS_DOMAIN_ID=5
ros2 topic list

# Should see all topics from all systems
# - Rover topics: tpc_chassis_*, tpc_gnss_*
# - Base topics: tpc_rover_dest_coordinate
# - Vision topics: tpc_camera_*, tpc_lane_*
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

Ensure firewall allows DDS traffic on the Ethernet switch network:

```bash
# Ubuntu (UFW) - Allow all traffic on local subnet
sudo ufw allow from 192.168.1.0/24 to any
sudo ufw allow to 192.168.1.0/24 from any

# Or specific ports for DDS
sudo ufw allow 7400:7500/udp  # DDS discovery and data
sudo ufw allow 239.255.0.1    # RTPS multicast

# For testing, temporarily disable firewall
sudo ufw disable

# Re-enable after testing
sudo ufw enable
```

**Required Ports:**
- UDP 7400-7500: DDS discovery and data exchange
- Multicast: 239.255.0.1 (RTPS discovery group)

**Network Requirements:**
- All systems must be on same L2 broadcast domain (same switch)
- Switch must support multicast forwarding (most do by default)
- No firewall/router between systems that blocks UDP multicast

## Domain Migration History

### Evolution

**v1.0 (Initial):** Three domains (2, 5, 6)
- Domain 2: Base station + high-level rover control
- Domain 5: IMU/chassis STM32
- Domain 6: Sensors/GNSS STM32
- **Issue:** Required domain bridges, complex synchronization, no action/service support across domains

**v2.0 (Intermediate):** Two domains (2, 5)
- Domain 2: Base station with bridge relay
- Domain 5: All rover-internal processing
- **Issue:** Still required bridge node, added latency, actions/services didn't work across domains

**v3.0 (Current - November 2025):** Unified Domain 5
- Domain 5: All systems (ws_rpi, ws_base, ws_jetson, STM32 boards)
- **Benefits:** Direct DDS, native actions/services, lower latency, simplified architecture

**Rationale for Unification:**
- Bridge relay added unnecessary overhead
- Actions and services work natively on same domain
- Sensor fusion benefits from direct access to all data
- All systems can communicate directly via DDS

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
1. Verify domain ID matches on all systems:
   ```bash
   echo $ROS_DOMAIN_ID  # Should be 5 on all machines
   ```

2. Check network connectivity (all systems must reach each other):
   ```bash
   ping 192.168.1.1  # RPi
   ping 192.168.1.5  # Jetson
   ping 192.168.1.2  # Chassis STM32
   ping 192.168.1.6  # Sensors STM32
   ping 192.168.1.10 # Base station
   ```

3. Verify all systems connected to same Ethernet switch:
   ```bash
   # Check link status
   ip link show eth0  # Should show "UP"
   
   # Check IP configuration
   ip addr show eth0  # Should show 192.168.1.x/24
   ```

4. Test multicast support on switch:
   ```bash
   # Terminal 1 (on any machine)
   ros2 multicast receive
   
   # Terminal 2 (on different machine, same switch)
   ros2 multicast send
   
   # Should see messages in Terminal 1
   ```

5. Check switch configuration:
   - Verify multicast is NOT blocked
   - Check no VLAN isolation between ports
   - Verify no port mirroring/monitoring enabled

6. Restart ROS2 daemon:
   ```bash
   ros2 daemon stop
   ros2 daemon start
   ```

7. Check DDS environment:
   ```bash
   ros2 doctor  # Diagnose DDS issues
   ```

### Cross-Domain Communication Fails

**Symptom:** Bridge node not relaying topics between domains

**Solutions:**
1. Verify all systems on Domain 5:
   ```bash
   export ROS_DOMAIN_ID=5
   ros2 node list  # Should see all nodes
   ros2 topic list  # Should see all topics
   ```

2. Check action/service availability:
   ```bash
   ros2 action list  # Should see /des_data
   ros2 service list  # Should see /srv_spd_limit
   ```

3. Test direct communication:
   ```bash
   ros2 topic echo tpc_gnss_spresense  # Should see GNSS data
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
   // app.cpp
   setenv("ROS_DOMAIN_ID", "5", 5);  // Must match RPi/Jetson
   ```

3. Rebuild and reflash firmware if domain mismatch

### Multicast Not Working

**Symptom:** Nodes on different machines don't discover each other

**Solutions:**
1. **Verify switch supports multicast:**
   - Managed switches: Check multicast forwarding is enabled
   - Unmanaged switches: Should work by default (check datasheet)
   - Some cheap switches block multicast - replace if needed

2. **Test multicast manually:**
   ```bash
   # On machine 1
   socat UDP4-RECV:7400,reuseaddr -
   
   # On machine 2
   echo "test" | socat - UDP4-DATAGRAM:239.255.0.1:7400
   
   # Should see "test" on machine 1
   ```

3. **Use unicast discovery (workaround):**
   ```bash
   # On each machine, list IPs of all other participants
   export ROS_STATIC_PEERS=192.168.1.1,192.168.1.5,192.168.1.2,192.168.1.6,192.168.1.10
   
   # Then launch nodes
   ros2 run <package> <node>
   ```

4. **Check for IGMP snooping issues:**
   - Managed switches: Verify IGMP snooping configured correctly
   - Some switches drop multicast when IGMP snooping enabled without IGMP querier
   - Solution: Disable IGMP snooping or configure IGMP querier

5. **Verify no Layer 3 routing:**
   - All systems must be on same subnet (192.168.1.0/24)
   - No router/gateway between systems
   - Use `ip route` to verify routing table

6. **Check switch port configuration:**
   - All ports on same VLAN (VLAN 1 default)
   - No port isolation features enabled
   - Storm control disabled (can block multicast)

7. **Use `tcpdump` to debug:**
   ```bash
   # Capture DDS multicast traffic
   sudo tcpdump -i eth0 -n 'multicast and port 7400'
   
   # Should see SPDP announcements every 500ms from all participants
   ```

## Best Practices

### Development

1. **Use Correct Domain:** Always set `ROS_DOMAIN_ID=5` before running ROS2 commands
2. **Consistent Domain:** All systems on Domain 5 - no need for domain switching
3. **Verify Before Deploy:** Check topics/nodes with `ros2 topic list` and `ros2 node list`

### Production

1. **Unified Domain:** All systems operate on Domain 5
2. **Network Security:** Use firewall rules to control DDS traffic
3. **Monitor Health:** Log communication statistics for diagnostics

### Debugging

1. **Use `ros2 doctor`:** Diagnose DDS configuration issues
2. **Check Logs:** Monitor `/rosout` topic for error messages
3. **Network Tools:** Use `tcpdump`, `wireshark` for deep packet inspection
4. **Domain Verification:** Ensure all systems report `ROS_DOMAIN_ID=5`

---

**See Also:**
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture overview
- [TOPICS.md](TOPICS.md) - Complete topic reference
- `docs/archive/SENSOR_FUSION_ARCHITECTURE.md` - Sensor fusion design details
