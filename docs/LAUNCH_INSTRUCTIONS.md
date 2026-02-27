# System Launch Instructions

Complete guide to launching the Almondmatcha rover system across all platforms.

## Quick Launch Summary

**Recommended tmux-based launch (organized terminal sessions):**

1. **Jetson (ws_jetson):** `./launch_jetson_tmux.sh` → 3 panes (camera, detection, control)
2. **Raspberry Pi (ws_rpi):** `./launch_rover_tmux.sh` → 5 panes (GNSS, chassis nodes)
3. **Base Station (ws_base):** Manual launch (optional)

**Alternative background scripts:**
- Jetson headless: `./launch_headless.sh`
- Jetson with GUI: `./launch_gui.sh`

## System Overview

- **Control Loop (Domain 5):** ws_rpi, ws_base, STM32 boards, ws_jetson control interface
- **Vision Processing (Domain 6):** ws_jetson camera and lane detection (localhost only)

### Domain 5 Participants (Network-Wide, visible to STM32)

Total: 11 nodes

- ws_rpi: 7 nodes (node_gnss_spresense, node_gnss_ublox, node_gnss_mission_monitor, node_chassis_controller, node_chassis_imu, node_chassis_sensors, mission_monitoring_node_rpi)
- ws_base: 1 node (mission_command_node)
- ws_jetson: 1 node (rover_kinematic_control)
- STM32: 2 nodes (chassis_controller, sensors_node)

### Domain 4 Participants (Telemetry, NOT visible to STM32)

- ws_base: mission_monitoring_node_pc (telemetry display)
- ws_jetson: node_rover_local_monitoring (CSV logging, future DB)

### Domain 6 Participants (Jetson Localhost Only)

- camera_stream
- lane_detection

Domain 6 nodes are invisible to STM32 boards and other systems.

1. All systems powered on and connected to network (192.168.1.0/24)
2. STM32 firmware built and flashed
3. All ROS2 workspaces built

**Typical network IPs:**
- ws_rpi: 192.168.1.1
- ws_jetson: 192.168.1.5
- STM32 chassis: 192.168.1.2
- STM32 sensors: 192.168.1.6
- ws_base: 192.168.1.10

## Launch Sequence

### Step 1: Power On STM32 Boards

Connect STM32 boards to power. Allow 5-10 seconds for initialization.

**Verify serial console output (115200 baud):**
```bash
minicom -D /dev/ttyACM0  # Chassis board
minicom -D /dev/ttyACM1  # Sensors board
```

Expected startup messages:
```
[MROS2] Network initialized
[MROS2] ROS_DOMAIN_ID: 5
[MROS2] Publisher initialized: tpc_chassis_imu
[MROS2] Publisher initialized: tpc_gnss_spresense
```

### Step 2: Launch ws_jetson Vision System

**Option A: Tmux launch (recommended):**
```bash
cd ~/almondmatcha/ws_jetson
source install/setup.bash
# Note: Script handles Domain 6 (vision) and Domain 5 (control) automatically
./launch_jetson_tmux.sh
```

This creates a 3-pane tmux session with:
- Pane 0: Camera stream (Domain 6)
- Pane 1: Lane detection (Domain 6)
- Pane 2: Steering control (Domain 5)

**Tmux controls:**
- `Ctrl+b` → arrows: Navigate panes
- `Ctrl+b` → `z`: Zoom pane
- `Ctrl+b` → `d`: Detach (session keeps running)

**Option B: Background launch script:**
```bash
cd ~/almondmatcha/ws_jetson
source install/setup.bash
# Note: Script handles Domain 6 (vision) and Domain 5 (control) automatically
./launch_headless.sh
```

**Option C: Manual multi-terminal launch:**

Terminal 1 - Vision Processing (Domain 6):
```bash
cd ~/almondmatcha/ws_jetson
source install/setup.bash
export ROS_DOMAIN_ID=6
ros2 launch vision_navigation vision_domain6.launch.py
```

Terminal 2 - Control Interface (Domain 5):
```bash
cd ~/almondmatcha/ws_jetson
source install/setup.bash
export ROS_DOMAIN_ID=5
ros2 launch vision_navigation control_domain5.launch.py
```

**Expected output:**
```
[camera_stream]: Starting camera stream at 30 FPS
[lane_detection]: Lane detection pipeline initialized
[rover_kinematic_control]: Waiting for lane detection data...
[rover_kinematic_control]: Rover Kinematic Control node initialized on Domain 6
```

Wait for all nodes to be ready (30 FPS messages flowing).

### Step 3: Launch ws_rpi Control System

**On Raspberry Pi:**
```bash
cd ~/almondmatcha/ws_rpi
source install/setup.bash
export ROS_DOMAIN_ID=5
./launch_rover_tmux.sh
```

This launches 5 nodes in tmux session:
- GNSS Spresense node
- GNSS mission monitor  
- Chassis controller
- Chassis IMU logger
- Chassis sensors logger

**Verify tmux session:**
```bash
tmux list-sessions
# Expected: rover:0

tmux attach-session -t rover  # View all panes
```

**Navigate tmux panes:**
- `Ctrl+b` then arrow keys: Switch between panes
- `Ctrl+b` then `z`: Zoom current pane
- `Ctrl+b` then `d`: Detach session

### Step 4: Launch ws_base Mission Control (Optional)

**On Base Station:**
```bash
cd ~/almondmatcha/ws_base
source install/setup.bash
# Script automatically launches both nodes on correct domains:
# - mission_command_node: Domain 5 (commands/actions)
# - mission_monitoring_node_pc: Domain 4 (telemetry display)
./launch_base_tmux.sh
```

This enables mission planning on Domain 5 and telemetry monitoring on Domain 4 from the base station.

## Verification Checklist

### Domain 6 (Vision) - On Jetson

```bash
export ROS_DOMAIN_ID=6
ros2 node list
# Expected: /camera_stream, /lane_detection

ros2 topic list
# Expected: /tpc_rover_d415_rgb, /tpc_rover_d415_depth, /tpc_rover_nav_lane

ros2 topic hz /tpc_rover_nav_lane
# Expected: ~30 Hz
```

### Domain 5 (Control) - On Any System

```bash
export ROS_DOMAIN_ID=5
ros2 node list
# Expected: 10 nodes total (8 without ws_base, 10 with ws_base)
# /rover_kinematic_control      (Jetson - kinematic control interface)
# /node_chassis_controller    (ws_rpi)
# /node_gnss_mission_monitor  (ws_rpi)
# /node_gnss_spresense        (ws_rpi)
# /node_chassis_imu           (ws_rpi)
# /node_chassis_sensors       (ws_rpi)
# /rover_node                 (STM32 chassis)
# /sensors_node               (STM32 sensors)
# /mission_command_node       (ws_base, if launched)
# /mission_monitoring_node    (ws_base, if launched)

ros2 topic list
# Should see: tpc_rover_ctrl_cmd, tpc_chassis_cmd, tpc_chassis_imu, tpc_rover_nav_lane, etc.
# Should NOT see camera topics like tpc_rover_d415_rgb (Domain 6 isolation)

ros2 topic hz /tpc_rover_ctrl_cmd
# Expected: ~50 Hz
```

### STM32 Communication

**Serial console (Domain 5 only, not Domain 6):**
```bash
minicom -D /dev/ttyACM0
# Should see motor commands being processed
# Should NOT see camera-related messages
```

## Shutdown Procedure

### Graceful Shutdown

**1. Stop ws_base (if running):**
```bash
# In mission_control terminal
Ctrl+C
```

**2. Stop ws_jetson:**
```bash
# Option A: Kill tmux session
tmux kill-session -t jetson_vision

# Option B: If using background scripts
pkill -f "vision_navigation"

# Option C: Manual shutdown
Ctrl+C  # In each terminal
```

**3. Stop ws_rpi:**
```bash
# Detach from tmux session if attached
Ctrl+b then d

# Kill tmux session
tmux kill-session -t rover
```

**4. Power down STM32 boards:**
- Disconnect power supply
- Wait 5 seconds before reconnecting

## Common Issues

### STM32 Boards Not Visible

**Symptom:** `ros2 node list` on Domain 5 doesn't show /rover_node or /sensors_node

**Troubleshooting:**
```bash
# Check serial console
minicom -D /dev/ttyACM0

# Check network connectivity
ping 192.168.1.2  # STM32 chassis

# Rebuild and reflash if domain mismatch
cd ~/almondmatcha/mros2-mbed-chassis-dynamics
sudo ./build.bash all NUCLEO_F767ZI chassis_controller
```

### Vision Data Not Reaching Control

**Symptom:** `/rover_kinematic_control` node stuck waiting for lane data

**Troubleshooting:**
```bash
# Verify Domain 6 vision is running
export ROS_DOMAIN_ID=6
ros2 topic list | grep nav_lane

# Check control node subscribing to Domain 6
export ROS_DOMAIN_ID=5
ros2 node info /rover_kinematic_control
# Should show subscription to tpc_rover_nav_lane

# Verify localhost DDS discovery
ros2 topic info /tpc_rover_nav_lane -v
```

### High STM32 Memory Usage

**Symptom:** Serial console shows `[MemoryPool] RESSOURCE LIMIT EXCEEDED`

**Troubleshooting:**
```bash
# Verify Domain 5 node count
export ROS_DOMAIN_ID=5
ros2 node list | wc -l
# Should be 8-10 nodes (without ws_base: 8, with ws_base: 10)

# Check for accidental Domain 6 nodes on Domain 5
ros2 topic list | grep d415
# Should return nothing (camera topics are Domain 6 only)

# If still failing, increase STM32 config
# Edit: mros2-mbed-chassis-dynamics/platform/rtps/config.h
# MAX_NUM_UNMATCHED_REMOTE_WRITERS = 40
# MAX_NUM_UNMATCHED_REMOTE_READERS = 50
```

### Topics Not Visible on Domain 5

**Symptom:** `ros2 topic list` shows incomplete or missing topics

**Troubleshooting:**
```bash
# Verify correct domain
echo $ROS_DOMAIN_ID  # Should be 5

# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start

# Check network connectivity
ping 192.168.1.1  # RPi
ping 192.168.1.5  # Jetson
ping 192.168.1.2  # STM32 chassis

# Check multicast support
ros2 multicast send
# On another machine:
ros2 multicast receive
```

## Performance Verification

### Latency Check

```bash
export ROS_DOMAIN_ID=5

# Control command latency (end-to-end)
ros2 topic delay /tpc_rover_ctrl_cmd
# Expected: 50-100 ms

# Camera-to-control latency (includes cross-domain)
export ROS_DOMAIN_ID=6
ros2 topic delay /tpc_rover_nav_lane  # Get D6 timestamp
export ROS_DOMAIN_ID=5
ros2 topic delay /tpc_rover_ctrl_cmd  # Compare timestamps
# Expected: 100-150 ms end-to-end
```

### CPU Usage

```bash
# On Jetson
top -p $(pgrep -f camera_stream)
top -p $(pgrep -f lane_detection)
top -p $(pgrep -f rover_kinematic_control)
# Expected: 40-50% total CPU

# On RPi
top -p $(pgrep -f chassis_controller)
# Expected: 10-20% CPU

# On STM32 (via serial console)
# Monitor free memory - should have >100KB free
```

### Network Bandwidth

Domain 6 (localhost only, not network traffic):
```bash
# Domain 6 uses ~100 Mbps localhost (shared memory)
```

Domain 5 (network traffic):
```bash
# Monitor with tcpdump
sudo tcpdump -i eth0 'port 7400-7500'
# Expected: 10-50 Mbps sustained (DDS multicast)
```

## Multi-Rover Setup

For multiple rovers, use different Domain IDs:

| Rover | Domain 5 | Domain 6 | Network |
|-------|----------|----------|---------|
| Rover 1 | 5 | 6 | 192.168.1.0/24 |
| Rover 2 | 8 | 9 | 192.168.2.0/24 |
| Base | 5, 8 | - | All networks |

**Launch Rover 2:**
```bash
# On Rover 2 systems, set domain before launch
export ROS_DOMAIN_ID=8  # Domain 5 equivalent for Rover 2
```

## See Also

- [DOMAINS.md](DOMAINS.md) - Domain architecture details
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture overview
- [TOPICS.md](TOPICS.md) - Complete topic reference
- `ws_jetson/README.md` - Jetson-specific instructions
- `ws_rpi/README.md` - RPi-specific instructions
- `ws_base/README.md` - Base station instructions
