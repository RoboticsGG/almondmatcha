# ws_base - Base Station Workspace

ROS2 workspace for ground station telemetry monitoring and rover command/control.

## Quick Start

```bash
# 1. Build common_ifaces first (provides msgs_ifaces)
cd ~/almondmatcha/common_ifaces
colcon build --symlink-install --packages-select msgs_ifaces

# 2. Build ws_base (sources common_ifaces automatically)
cd ~/almondmatcha/ws_base
bash build_clean.sh
source install/setup.bash

# 3. Launch (POC single-domain)
bash launch_base_single_domain.sh
```

## Overview

- **Purpose:** Monitor rover telemetry and send commands from base station
- **Platform:** Linux PC (Ubuntu 20.04/22.04)
- **Network:** Connect to rover Ethernet switch (192.168.1.10 recommended)
- **Domain:** ROS2 Domain **5** (single-domain POC — all participants on D5)
- **Communication:** Gigabit Ethernet via switch - all systems on same LAN

## Nodes

| Node | Function |
|------|----------|
| `mission_command_node` | Send navigation goals (action `/des_data`), speed limits (service `/srv_spd_limit`) — **Domain 5** |
| `mission_monitoring_node_pc` | Subscribe `/tpc_telemetry_relay` on **Domain 4** — real-time telemetry display |

## Building

`ws_base/src/msgs_ifaces` is a symlink to `common_ifaces/msgs_ifaces`. To avoid duplicate
`.so` files in `LD_LIBRARY_PATH` (which causes a typesupport import failure at runtime),
`msgs_ifaces` is **not** built inside ws_base. Instead, `build_clean.sh` sources it from
`common_ifaces/install/` and builds only `action_ifaces`, `services_ifaces`, and
`mission_control` locally.

**Always build in this order:**

```bash
# Step 1 — build msgs_ifaces in common_ifaces (single source of truth)
cd ~/almondmatcha/common_ifaces
colcon build --symlink-install --packages-select msgs_ifaces

# Step 2 — build ws_base (sources common_ifaces, builds action_ifaces + services_ifaces + mission_control)
cd ~/almondmatcha/ws_base
bash build_clean.sh          # clean build
# or: bash build_inc.sh     # incremental (after common_ifaces is already built)
source install/setup.bash
```

**If you change any .msg file in common_ifaces/msgs_ifaces/msg/:**
```bash
# Must do a clean rebuild of common_ifaces first to avoid stale .so artifacts
cd ~/almondmatcha/common_ifaces && rm -rf build/ install/ log/
colcon build --symlink-install --packages-select msgs_ifaces
cd ~/almondmatcha/ws_base && bash build_clean.sh
```

**Build Output:**
- `install/action_ifaces/` — DesData action typesupport
- `install/services_ifaces/` — SpdLimit service typesupport
- `install/mission_control/lib/` — compiled nodes
- `install/mission_control/share/` — launch files and params

## Running

### Launch Script (Recommended)

**Using ROS2 Launch File:**
```bash
export ROS_DOMAIN_ID=5
cd ~/almondmatcha/ws_base
source install/setup.bash
ros2 launch mission_control mission_control.launch.py
```

This automatically loads parameters from `config/params.yaml`.

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

### Manual Launch (Advanced)

**Note:** When running nodes manually with `ros2 run`, you must specify parameters:

```bash
export ROS_DOMAIN_ID=5
cd ~/almondmatcha/ws_base
source install/setup.bash

# Option 1: Load from params.yaml
ros2 run mission_control mission_command_node --ros-args --params-file src/mission_control/config/params.yaml

# Option 2: Specify parameters inline
ros2 run mission_control mission_command_node --ros-args \
  -p rover_spd:=15 \
  -p des_lat:=8.007286 \
  -p des_long:=101.50203

# Terminal 2: Monitoring node (Domain 4)
ros2 run mission_control mission_monitoring_node_pc
```

## Configuration

### Parameters File

Edit `src/mission_control/config/params.yaml`:

```yaml
mission_command_node:
  ros__parameters:
    rover_spd: 15              # Speed limit (0-100%)
    des_lat: 8.007286          # Target latitude (decimal degrees)
    des_long: 101.50203        # Target longitude (decimal degrees)
    action_watchdog_timeout_sec: 10.0  # Cancel goal if no feedback for this many seconds
```

**Apply Changes:**

After editing the YAML file, restart the mission control nodes:

```bash
# If using launch file (recommended):
export ROS_DOMAIN_ID=5
ros2 launch mission_control mission_control.launch.py

# Or rebuild to update installed config:
colcon build --packages-select mission_control
source install/setup.bash
ros2 launch mission_control mission_control.launch.py
```

### Network Setup

**Topology:** Connect base station to same Ethernet switch as rover systems

**Recommended IP:** 192.168.1.10 (or any .10-.99 to avoid conflicts)

```bash
# Option 1: NetworkManager (recommended)
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.10/24
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con up "Wired connection 1"

# Option 2: Temporary (testing)
sudo ip addr add 192.168.1.10/24 dev eth0
sudo ip link set eth0 up
```

**Verify Connectivity (all via switch):**
```bash
# Check Ethernet link
ip link show eth0  # Should show UP

# Ping all rover systems on switch
ping 192.168.1.1    # Raspberry Pi
ping 192.168.1.5    # Jetson
ping 192.168.1.2    # STM32 chassis
ping 192.168.1.6    # STM32 sensors
```

**Firewall Configuration:**
```bash
# Allow DDS traffic from rover network
sudo ufw allow from 192.168.1.0/24 to any
sudo ufw allow to 192.168.1.0/24 from any

# Or disable for testing
sudo ufw disable

# Re-enable after testing
sudo ufw enable
```

## Communication

### Topics from Rover (Domain 5)

All systems on Domain 5 via Ethernet switch - direct DDS discovery:

| Topic | Type | Content |
|-------|------|---------|
| `tpc_gnss_spresense` | SpresenseGNSS | GPS position (RPi, D5) |
| `tpc_gnss_mission_active` | Bool | Mission status (RPi, D5) |
| `tpc_gnss_mission_remain_dist` | Float64 | Distance to waypoint (RPi, D5) |
| `tpc_chassis_cmd` | ChassisCtrl | Motor commands (RPi → STM32, D5) |
| `tpc_chassis_imu` | ChassisIMU | IMU sensor data (STM32 → RPi, D5) |
| `tpc_chassis_sensors` | ChassisSensors | Encoders/power (STM32 → RPi, D5) |
| `tpc_rover_nav_lane` | Float32MultiArray | Lane parameters (Jetson, D5) |
| `tpc_rover_ctrl_cmd` | Float32MultiArray | Kinematic control commands (Jetson, D5) |

**Note:** In the single-domain POC, camera topics (`tpc_rover_d415_rgb`, `tpc_rover_d415_depth`) are on D5 and visible from the base PC (stress-test traffic). In the baseline (main branch) they run on Domain 6 (Jetson localhost only).

### Commands to Rover (Domain 5)

| Interface | Type | Purpose |
|-----------|------|---------|
| `/des_data` | Action | Navigation goal (lat/lon) to RPi |
| `/srv_spd_limit` | Service | Speed limit (0-100%) to RPi |

## Testing

### Verify Domain Configuration

```bash
export ROS_DOMAIN_ID=5
ros2 node list

# Expected (when rover is running):
# /mission_command_node           (D5)
# /mission_monitoring_node_pc     (D4 — only visible if ROS_DOMAIN_ID=4)
# /chassis_controller_node        (ws_rpi, D5)
# /gnss_mission_monitor_node      (ws_rpi, D5)
# /gnss_spresense_node            (ws_rpi, D5)
# ... and other rover nodes on D5
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
ros2 service call /srv_spd_limit services_ifaces/srv/SpdLimit \
    "{rover_spd: 20}"
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

**Symptom:** `colcon build` errors or `"Could not import rosidl_typesupport_c for package msgs_ifaces"` at runtime

**Solutions:**
```bash
# Always rebuild common_ifaces first (clean to avoid stale rover_status artifacts)
cd ~/almondmatcha/common_ifaces && rm -rf build/ install/ log/
colcon build --symlink-install --packages-select msgs_ifaces

# Then clean rebuild ws_base
cd ~/almondmatcha/ws_base && bash build_clean.sh
source install/setup.bash
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
ros2 service list | grep srv_spd
# Should show: /srv_spd_limit

# Check rover nodes are running
ros2 node list
# Should show: /gnss_mission_monitor_node, /chassis_controller_node, etc. (D5)
```

## Directory Structure

```
ws_base/
├── README.md
├── build_clean.sh              # Clean build (sources common_ifaces, skips msgs_ifaces rebuild)
├── build_inc.sh                # Incremental build
├── launch_base_single_domain.sh  # POC single-domain launch
├── launch_base_tmux.sh
├── launch_poc_lab.sh            # Laboratory POC run (camera fallback, verbose)
├── launch_poc_field.sh          # On-field POC run (live camera, Ctrl-C emergency stop)
├── launch_poc_experiment.sh    # Legacy POC script (superseded by lab/field split)
├── fastdds_base.xml            # FastDDS profile pinned to 192.168.1.4
├── tools/
│   ├── monitoring/             # collect_topic_bw.py, collect_net_stats.py
│   ├── stm32_serial/           # collect_stm32_memory.py
│   ├── tracing/                # collect_latency.py, start_trace.sh, stop_and_collect_trace.sh
│   └── poc_run/                # merge_run_csv.py; single_domain/ & multi_domain/ run dirs (git-ignored)
└── src/
    ├── action_ifaces/          # DesData action (built here)
    ├── msgs_ifaces  -> ../../common_ifaces/msgs_ifaces  # symlink — NOT built here
    ├── services_ifaces/        # SpdLimit service (built here)
    └── mission_control/
        ├── config/params.yaml  # rover_spd, des_lat, des_long
        └── src/
            ├── mission_command_node.cpp       # D5: sends navigation goals + speed limit
            └── mission_monitoring_node_pc.cpp # subscribes /tpc_telemetry_relay
```

## System Integration

**Base Station Dual-Domain Role:**
- `mission_command_node` (D5): sends mission goals/speed limits, calls RPi action/service servers
- `mission_monitoring_node_pc` (D4): subscribes to `/tpc_telemetry_relay` — telemetry display only (NOT a D5 participant)

**Architecture:**
```
Domain 5: mission_command_node (Base) ←→ ws_rpi ←→ ws_jetson(rover_kinematic_control)
                                              ↕
                                         STM32 Boards

Domain 4: mission_monitoring_node_pc (Base) + rover_local_monitoring_node (Jetson)
                                        ↑
                             tpc_telemetry_relay (5 Hz relay from RPi)
```

Domain 4 nodes are invisible to STM32 boards — no additional STM32 memory cost.

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
**Domains:** 5 (command) + 4 (monitoring)  
**Network:** 192.168.1.0/24 (rover network)
