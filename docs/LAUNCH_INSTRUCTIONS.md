# System Launch Instructions

Complete guide to launching the Almondmatcha rover system across all platforms.

## Quick Launch Summary

> **Branch: `single-domain`** — Use the `_single_domain` scripts on every machine. All nodes run on **ROS_DOMAIN_ID=5**.

**Recommended launch order:**

1. **STM32 boards:** power on, wait ~10 s for DDS discovery
2. **Raspberry Pi (ws_rpi):** `./launch_rover_single_domain.sh`
3. **Jetson (ws_jetson):** `./launch_jetson_single_domain.sh`
4. **Base Station (ws_base):** `./launch_base_single_domain.sh` (optional)

## System Overview

- **All communication on Domain 5:** ws_rpi, ws_base, ws_jetson, STM32 boards
- Total participants on D5: ~16 nodes

### Domain 5 Participants

- ws_rpi: 8 nodes (gnss_spresense_node, gnss_ublox_node, gnss_mission_monitor_node, chassis_controller_node, chassis_imu_node, chassis_sensors_node, mission_monitoring_node_rpi, rover_monitoring_node)
- ws_base: 2 nodes (mission_command_node, mission_monitoring_node_pc)
- ws_jetson: 3 nodes (camera_stream_node, lane_detection_node, rover_kinematic_control)
- STM32: 2 nodes (chassis_controller, sensors_node)

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

This creates a 3-pane tmux session:
- Pane 0: Camera stream (Domain 6)
- Pane 1: Lane detection (Domain 6)
- Pane 2: Rover kinematic control (dual-context D6 sub / D5 pub)

**Tmux controls:** `Ctrl+b` + arrows to navigate · `z` to zoom · `d` to detach

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
[camera_stream_node]: Starting camera stream at 30 FPS
[lane_detection_node]: Lane detection pipeline initialized
[rover_kinematic_control]: Waiting for lane detection data...
[rover_kinematic_control]: Rover Kinematic Control node initialized on Domain 6
```

Wait for all nodes to be ready (30 FPS messages flowing).

### Step 3: Launch ws_rpi Control System

**On Raspberry Pi** (`ssh curry@192.168.1.1`):
```bash
cd ~/almondmatcha/ws_rpi
source install/setup.bash
export ROS_DOMAIN_ID=5
./launch_rover_tmux.sh
```

This launches 7 nodes in a tmux session (`rover`):
- GNSS Spresense, GNSS mission monitor, Chassis controller
- Chassis IMU, Chassis sensors, Mission monitoring node
- gnss_ublox_node

**Tmux:** `Ctrl+b`+arrows / `z` zoom / `d` detach / `tmux attach -t rover`

### Step 4: Launch ws_base Mission Control (Optional)

**On Base Station:**
```bash
cd ~/almondmatcha/ws_base
source install/setup.bash
export ROS_DOMAIN_ID=5
./launch_base_single_domain.sh
```

## Verification Checklist

### Domain 5 — On Any System

```bash
export ROS_DOMAIN_ID=5
ros2 node list
# Expected: ~16 nodes total
# /rover_kinematic_control        (Jetson)
# /camera_stream_node             (Jetson)
# /lane_detection_node            (Jetson)
# /chassis_controller_node        (ws_rpi)
# /gnss_mission_monitor_node      (ws_rpi)
# /gnss_spresense_node            (ws_rpi)
# /gnss_ublox_node                (ws_rpi)
# /chassis_imu_node               (ws_rpi)
# /chassis_sensors_node           (ws_rpi)
# /mission_monitoring_node_rpi    (ws_rpi)
# /rover_monitoring_node          (ws_rpi)
# /chassis_controller             (STM32 chassis)
# /sensors_node                   (STM32 sensors)
# /mission_command_node           (ws_base, if launched)
# /mission_monitoring_node_pc     (ws_base, if launched)

ros2 topic list
# Should see: tpc_rover_ctrl_cmd, tpc_chassis_cmd, tpc_chassis_imu,
#             tpc_rover_nav_lane, tpc_rover_d415_rgb, tpc_telemetry_relay, etc.

ros2 topic hz /tpc_rover_ctrl_cmd
# Expected: ~50 Hz
```

### STM32 Communication

```bash
export ROS_DOMAIN_ID=5
minicom -D /dev/ttyACM0
# Should see motor commands being processed
```

## Shutdown

```bash
tmux kill-session -t jetson_vision   # Jetson
tmux kill-session -t rover           # RPi
# ws_base: Ctrl+C
```

Power down STM32 boards last.

## Common Issues

### STM32 Boards Not Visible
```bash
ping 192.168.1.2  # STM32 chassis
minicom -D /dev/ttyACM0  # Check serial console
```

### Vision Data Not Reaching Control
```bash
export ROS_DOMAIN_ID=5
ros2 topic list | grep nav_lane       # Confirm lane topic visible on D5
ros2 topic hz /tpc_rover_nav_lane     # Confirm ~30 Hz
```

### High STM32 Memory Usage
`[MemoryPool] RESSOURCE LIMIT EXCEEDED` — check D5 participant count:
```bash
export ROS_DOMAIN_ID=5
ros2 node list | wc -l  # Should be ~16 (all D5 nodes)
```
See [STM32_RTPS_MEMORY_CALCULATION.md](STM32_RTPS_MEMORY_CALCULATION.md) for config details.

### Topics Not Visible
```bash
echo $ROS_DOMAIN_ID          # Verify correct domain
ros2 daemon stop && ros2 daemon start
ping 192.168.1.1             # RPi reachable?
```

---

**See Also:** [DOMAINS.md](DOMAINS.md) · [ARCHITECTURE.md](ARCHITECTURE.md) · [TOPICS.md](TOPICS.md)
