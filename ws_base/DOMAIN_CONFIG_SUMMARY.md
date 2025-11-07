# Domain Configuration Summary

## ws_base ↔ ws_rpi Communication

### Domain Architecture (Corrected)

```
┌─────────────────┐                    ┌──────────────────┐
│   ws_rpi (D5)   │                    │  ws_base (D2)    │
│                 │                    │                  │
│ - Rover Sensors │                    │ - Command Node   │
│ - GNSS          │                    │ - Monitoring     │
│ - Chassis       │  pkg_base_bridge   │   Node           │
│ - Control       │ ◄─────────────────►│                  │
│                 │   (Domain relay)   │                  │
└─────────────────┘                    └──────────────────┘
     Domain 5                              Domain 2
```

### Corrected Configuration (2025-01-05)

**Previous (Incorrect):**
- ws_base was configured for Domain 6
- No documentation of bridge requirements

**Current (Correct):**
- ws_base now uses Domain 2
- Bridge node documented and required

### Bridge Node: pkg_base_bridge

**Location:** `ws_rpi/src/pkg_base_bridge/`

**Purpose:** Multi-domain topic relay between rover internal (D5) and base station (D2)

**Topics Relayed:**

**D5 → D2 (Telemetry to base station):**
- `tpc_chassis_imu` - IMU sensor data
- `tpc_chassis_sensors` - Chassis telemetry  
- `tpc_gnss_spresense` - GPS position
- `tpc_chassis_cmd` - Chassis commands
- `tpc_gnss_mission_active` - Mission status
- `tpc_gnss_mission_remain_dist` - Distance remaining

**D2 → D5 (Commands to rover):**
- `tpc_rover_dest_coordinate` - Destination waypoint

### Running the System

**1. Start ws_rpi (on Raspberry Pi):**
```bash
cd ~/almondmatcha/ws_rpi
export ROS_DOMAIN_ID=5
source install/setup.bash

# Option A: Launch all including bridge
ros2 launch pkg_rover rover_startup.launch.py

# Option B: Manual bridge start
ros2 run pkg_base_bridge node_base_bridge
```

**2. Start ws_base (on base station):**
```bash
cd ~/almondmatcha/ws_base
export ROS_DOMAIN_ID=2
source install/setup.bash

# Option A: Tmux launcher
./launch_base_tmux.sh

# Option B: GNU Screen launcher
./launch_base_screen.sh

# Option C: Manual
ros2 run mission_control node_commands
ros2 run mission_control node_monitoring
```

### Verification

**On base station (Domain 2):**
```bash
export ROS_DOMAIN_ID=2
ros2 topic list
# Should see: tpc_gnss_spresense, tpc_chassis_sensors, etc.

ros2 topic echo /tpc_gnss_spresense
# Should receive GPS data from rover
```

**On rover (Domain 5):**
```bash
export ROS_DOMAIN_ID=5
ros2 topic list
# Should see: tpc_rover_dest_coordinate (from base)

ros2 topic echo /tpc_rover_dest_coordinate
# Should receive commands from base station
```

### Files Updated (2025-01-05)

✅ **Launch Scripts:**
- `launch_base_screen.sh` - Domain 6 → Domain 2
- `launch_base_tmux.sh` - Domain 6 → Domain 2

✅ **Documentation:**
- `README.md` - Domain references corrected
- `docs/ARCHITECTURE.md` - Bridge architecture added
- `docs/QUICK_START.md` - Domain corrected
- `docs/LAUNCH.md` - Domain corrected  
- `docs/TOPICS.md` - Domain corrected
- `docs/SETUP.md` - Domain export commands corrected

### Domain Summary Table

| Workspace   | Domain | Purpose                  | Network       |
|-------------|--------|--------------------------|---------------|
| ws_rpi      | 5      | Rover internal           | Isolated      |
| ws_base     | 2      | Base station control     | Via bridge    |
| ws_jetson   | 7      | Vision processing        | Independent   |

**Key Points:**
- ws_base must use Domain 2 (not 6) to communicate with ws_rpi
- pkg_base_bridge must be running on ws_rpi for cross-domain communication
- Bridge uses DDS domain routing or dual-process method
- All communication is bidirectional through the bridge

### Troubleshooting

**No topics visible on ws_base:**
- Verify ROS_DOMAIN_ID=2 is set
- Check pkg_base_bridge is running on ws_rpi
- Verify network connectivity between machines
- Check firewall allows DDS traffic (UDP ports)

**Commands not reaching rover:**
- Verify tpc_rover_dest_coordinate is being published on D2
- Check bridge is subscribing/publishing correctly
- Use `ros2 topic info /tpc_rover_dest_coordinate` on both domains

**For detailed bridge documentation:**
See `ws_rpi/src/pkg_base_bridge/README.md`
