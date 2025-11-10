# ws_base - Base Station Workspace

ROS2 workspace for ground station telemetry monitoring and rover command/control.

## Quick Start

```bash
cd ~/almondmatcha/ws_base
colcon build
source install/setup.bash
./launch_base_screen.sh
```

## Overview

- **Purpose:** Monitor rover telemetry and send commands from base station
- **Platform:** Linux PC (Ubuntu 20.04/22.04)
- **Domain:** ROS2 Domain 5 (unified with rover)
- **Communication:** Direct Ethernet to rover network (192.168.1.0/24)

## Nodes

| Node | Function |
|------|----------|
| `mission_command_node` | Send navigation goals, speed limits |
| `mission_monitoring_node` | Display real-time telemetry |

## Building

```bash
cd ~/almondmatcha/ws_base
colcon build
source install/setup.bash
```

**Build Output:**
- Compiled nodes in `install/mission_control/lib/`
- Launch files in `install/mission_control/share/`

## Running

### Launch Script (Recommended)

**GNU Screen:**
```bash
./launch_base_screen.sh
```

**Tmux:**
```bash
./launch_base_tmux.sh
```

**Screen Commands:**
- `Ctrl+a` then `n` - Next window
- `Ctrl+a` then `p` - Previous window
- `Ctrl+a` then `d` - Detach session
- `screen -r base` - Reattach session
- `Ctrl+a` then `k` - Kill window

### Manual Launch

```bash
export ROS_DOMAIN_ID=5
cd ~/almondmatcha/ws_base
source install/setup.bash

# Terminal 1: Command node
ros2 run mission_control mission_command_node

# Terminal 2: Monitoring node
ros2 run mission_control mission_monitoring_node
```

### ROS2 Launch File

```bash
export ROS_DOMAIN_ID=5
ros2 launch mission_control node_comlaunch.py
```

## Configuration

### Parameters File

Edit `src/mission_control/config/params.yaml`:

```yaml
node_commands:
  ros__parameters:
    rover_spd: 15              # Speed limit (0-100%)
    des_lat: 7.007286          # Target latitude (decimal degrees)
    des_long: 100.50203        # Target longitude (decimal degrees)
```

**Apply Changes:**

```bash
# Rebuild (if params.yaml is installed resource)
colcon build --packages-select mission_control
source install/setup.bash

# Or edit installed file directly:
nano install/mission_control/share/mission_control/config/params.yaml
# No rebuild needed, restart nodes
```

### Network Setup

Base station must be on rover network:

```bash
# Temporary
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip link set eth0 up

# Verify connectivity to rover
ping 192.168.1.1    # Raspberry Pi (bridge node)
```

**Firewall:**
```bash
# Allow DDS traffic from rover network
sudo ufw allow from 192.168.1.0/24

# Or disable for testing
sudo ufw disable
```

## Communication

### Topics from Rover (Domain 5)

Direct access to rover network:

| Topic | Type | Content |
|-------|------|---------|
| `tpc_gnss_spresense` | SpresenseGNSS | GPS position |
| `tpc_gnss_mission_active` | Bool | Mission status |
| `tpc_gnss_mission_remain_dist` | Float64 | Distance to waypoint |
| `tpc_chassis_cmd` | ChassisCtrl | Motor commands (monitoring) |
| `tpc_chassis_imu` | ChassisIMU | IMU sensor data from STM32 |
| `tpc_chassis_sensors` | ChassisSensors | GNSS/encoders from STM32 |

### Commands to Rover (Domain 5)

| Interface | Type | Purpose |
|-----------|------|---------|
| `/des_data` | Action | Navigation goal (lat/lon) |
| `/spd_limit` | Service | Speed limit (0-100%) |

## Testing

### Verify Domain Configuration

```bash
export ROS_DOMAIN_ID=5
ros2 node list

# Expected (when rover is running):
# /mission_command_node
# /mission_monitoring_node
# /chassis_controller
# /gnss_mission_monitor
# /spresense_gnss_node
# ... and other rover nodes
```

### Monitor Rover Telemetry

```bash
export ROS_DOMAIN_ID=5

# GPS position
ros2 topic echo tpc_gnss_spresense

# Mission status
ros2 topic echo tpc_gnss_mission_active

# Distance remaining
ros2 topic echo tpc_gnss_mission_remain_dist

# STM32 IMU data
ros2 topic echo tpc_chassis_imu

# STM32 GNSS/encoder data
ros2 topic echo tpc_chassis_sensors
```

### Send Commands

**Set Navigation Goal:**
```bash
ros2 action send_goal /des_data action_ifaces/action/DesData \
    "{des_lat: 7.007286, des_long: 100.502030}"
```

**Set Speed Limit:**
```bash
ros2 service call /spd_limit services_ifaces/srv/SpdLimit \
    "{spd_limit: 20}"
```

## Troubleshooting

### No Topics Visible

**Symptom:** `ros2 topic list` shows no rover topics

**Solutions:**
```bash
# Verify domain
echo $ROS_DOMAIN_ID  # Should be 5

# Check network connectivity
ping 192.168.1.1  # Raspberry Pi

# Verify rover nodes running
export ROS_DOMAIN_ID=5
ros2 node list

# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

### Build Failures

**Symptom:** `colcon build` errors

**Solutions:**
```bash
# Clean rebuild
rm -rf build install log
colcon build
source install/setup.bash

# Check ROS2 environment
source /opt/ros/humble/setup.bash
colcon build
```

### Node Won't Start

**Symptom:** "no such package/node" error

**Solutions:**
```bash
# Source environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Verify package built
ls install/mission_control/lib/mission_control/
```

### Commands Not Received by Rover

**Symptom:** Action/service calls timeout

**Solutions:**
```bash
# Verify domain is set correctly
export ROS_DOMAIN_ID=5

# Verify action server available
ros2 action list
# Should show: /des_data

# Verify service available
ros2 service list | grep spd
# Should show: /srv_spd_limit

# Check rover nodes are running
ros2 node list
# Should show: /gnss_mission_monitor, /chassis_controller, etc.
```

## Directory Structure

```
ws_base/
├── README.md                       # This file
├── launch_base_screen.sh           # GNU Screen launcher
├── launch_base_tmux.sh             # Tmux launcher
├── DOMAIN_CONFIG_SUMMARY.md        # Domain architecture notes
├── docs/                           # Detailed documentation
│   ├── QUICK_START.md
│   ├── ARCHITECTURE.md
│   ├── LAUNCH.md
│   ├── TOPICS.md
│   └── SETUP.md
└── src/
    ├── common_ifaces/              # Symlinks to shared interfaces
    │   ├── action_ifaces/
    │   ├── msgs_ifaces/
    │   └── services_ifaces/
    └── mission_control/            # Main package
        ├── config/
        │   └── params.yaml         # Configuration parameters
        ├── launch/
        │   └── node_comlaunch.py
        └── src/
            ├── node_commands.cpp   # Command dispatcher
            └── node_monitoring.cpp # Telemetry display
```

## System Integration

**Base Station Role:**
- Unified with rover network on Domain 5
- Direct access to all rover telemetry and control interfaces
- Sends high-level commands (waypoints, speed limits)
- Can monitor all topics including STM32 sensor data

**Unified Architecture:**
```
Domain 5: ws_base ←→ ws_rpi ←→ ws_jetson
                      ↕
                 STM32 Boards (Chassis + GNSS)
```

All systems communicate directly on Domain 5:
- ws_base: Mission command and monitoring
- ws_rpi: Rover control, GNSS navigation, action/service servers
- ws_jetson: Vision processing, lane detection
- STM32 Chassis: Motor control, IMU data (mROS2)
- STM32 GNSS: GPS, encoders, power monitor (mROS2)

## Detailed Documentation

See `docs/` subdirectory for:
- [QUICK_START.md](docs/QUICK_START.md) - Fast reference
- [ARCHITECTURE.md](docs/ARCHITECTURE.md) - System design
- [LAUNCH.md](docs/LAUNCH.md) - Launch scripts guide
- [TOPICS.md](docs/TOPICS.md) - Communication interfaces
- [SETUP.md](docs/SETUP.md) - Installation and troubleshooting

---

**Platform:** Linux PC (Ubuntu 20.04+)  
**ROS2:** Humble or Iron  
**Domain:** 5 (unified with rover)  
**Network:** 192.168.1.0/24 (rover network)
