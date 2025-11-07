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
- **Domain:** ROS2 Domain 2 (base station bridge)
- **Communication:** Ethernet to rover network via `node_base_bridge`

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
export ROS_DOMAIN_ID=2
cd ~/almondmatcha/ws_base
source install/setup.bash

# Terminal 1: Command node
ros2 run mission_control mission_command_node

# Terminal 2: Monitoring node
ros2 run mission_control mission_monitoring_node
```

### ROS2 Launch File

```bash
export ROS_DOMAIN_ID=2
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

### Topics from Rover (Domain 2)

Relayed by `node_base_bridge` on Raspberry Pi:

| Topic | Type | Content |
|-------|------|---------|
| `tpc_gnss_spresense` | SpresenseGNSS | GPS position |
| `tpc_gnss_mission_active` | Bool | Mission status |
| `tpc_gnss_mission_remain_dist` | Float64 | Distance to waypoint |
| `tpc_chassis_cmd` | ChassisCtrl | Motor commands (monitoring) |

### Commands to Rover (Domain 2)

| Interface | Type | Purpose |
|-----------|------|---------|
| `/des_data` | Action | Navigation goal (lat/lon) |
| `/spd_limit` | Service | Speed limit (0-100%) |

## Testing

### Verify Domain Configuration

```bash
export ROS_DOMAIN_ID=2
ros2 node list

# Expected:
# /mission_command_node
# /mission_monitoring_node
# /node_base_bridge (if rover is running)
```

### Monitor Rover Telemetry

```bash
export ROS_DOMAIN_ID=2

# GPS position
ros2 topic echo tpc_gnss_spresense

# Mission status
ros2 topic echo tpc_gnss_mission_active

# Distance remaining
ros2 topic echo tpc_gnss_mission_remain_dist
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
echo $ROS_DOMAIN_ID  # Should be 2

# Check network connectivity
ping 192.168.1.1  # Raspberry Pi

# Verify bridge node running on rover
ssh pi@192.168.1.1
ros2 node list | grep base_bridge

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
# Check bridge node is running
export ROS_DOMAIN_ID=2
ros2 node info /node_base_bridge

# Verify action server available
ros2 action list

# Verify service available
ros2 service list

# Check rover is in Domain 5
ssh pi@192.168.1.1
export ROS_DOMAIN_ID=5
ros2 node list
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
- Isolated from rover-internal processing (Domain 2 vs Domain 5)
- Receives aggregated telemetry via `node_base_bridge`
- Sends high-level commands (waypoints, speed limits)
- No direct access to low-level control (motor commands)

**Bridge Architecture:**
```
Rover (Domain 5) ←→ node_base_bridge ←→ ws_base (Domain 2)
```

Bridge node runs on Raspberry Pi and relays:
- Telemetry: Rover → Base (GPS, mission status, sensor data)
- Commands: Base → Rover (waypoints, speed limits)

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
**Domain:** 2 (base station bridge)  
**Network:** 192.168.1.0/24 (rover network)
