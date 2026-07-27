# System Launch Instructions

Complete guide to launching the Almondmatcha rover system across all platforms.

## Quick Launch Summary

**Recommended tmux-based launch (organized terminal sessions):**

1. **STM32 boards:** power on and wait for network readiness
2. **Raspberry Pi (ws_rpi):** `./launch_rover_tmux.sh` → 8 active panes + 1 spare
3. **Jetson (ws_jetson):** `./launch_jetson_tmux.sh` → 5 panes (D6 camera, D6 lane, D6->D5 control, D4 monitor, D6 camera recorder)
4. **Base Station (ws_base):** `./launch_base_tmux.sh` (optional)

**Alternative background scripts:**
- Jetson headless: `./launch_headless.sh`
- Jetson with GUI: `./launch_gui.sh`

> **Production note:** `main` branch does not ship POC tracing collectors or memory/bandwidth tracing workflow. Launch only runtime nodes.

## System Overview

- **Control Loop (Domain 5):** ws_rpi, ws_base, STM32 boards, ws_jetson control interface
- **Vision Processing (Domain 6):** ws_jetson camera and lane detection (localhost only)

### Domain 5 Participants (Network-Wide, visible to STM32)

Total: 12 nodes

- ws_rpi: 8 nodes (gnss_spresense_node, gnss_ublox_node, gnss_mission_monitor_node, chassis_controller_node, chassis_imu_node, chassis_sensors_node, mission_monitoring_node_rpi, rover_monitoring_node)
- ws_base: 1 node (mission_command_node)
- ws_jetson: 1 node (rover_kinematic_control)
- STM32: 2 nodes (chassis_controller, sensors_node)

### Domain 4 Participants (Telemetry, NOT visible to STM32)

- ws_base: mission_monitoring_node_pc (telemetry display)
- ws_jetson: rover_local_monitoring_node (CSV logging, future DB)

### Domain 6 Participants (Jetson Localhost Only)

- camera_stream_node
- lane_detection_node
- camera_recorder_node (field-run video logging only, not part of the control path)

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

Expected startup messages on chassis board (192.168.1.2):
```
[MROS2] ========================================
[MROS2]   Firmware: chassis-dynamics
[MROS2]   Board IP: 192.168.1.2
[MROS2]   Platform: NUCLEO_F767ZI
[MROS2]   Domain:   5
[MROS2]   SPDP max: 30  HB: 1000 ms  SPDP: 500 ms
[MROS2]   Node: RoverWithIMU
[MROS2]   Pub:  tpc_chassis_imu (10 Hz)
[MROS2]   Sub:  tpc_chassis_cmd
[MROS2] ========================================
```
Expected startup messages on sensors board (192.168.1.6):
```
[MROS2] ========================================
[MROS2]   Firmware: sensors-gnss
[MROS2]   Board IP: 192.168.1.6
[MROS2]   Platform: NUCLEO_F767ZI
[MROS2]   Domain:   5
[MROS2]   SPDP max: 30  HB: 1000 ms  SPDP: 500 ms
[MROS2]   Node: mros2_node_sensors
[MROS2]   Pub:  tpc_chassis_sensors (4 Hz)
[MROS2]   Sub:  (none)
[MROS2] ========================================
```
> **Note:** Discovery happens automatically once RPi nodes join D5 and send SPDP announcements.
> The firmware waits 3 s internally (inside IMU/sensors tasks, with `spin()` already running)
> before sending its first SPDP announcement — no manual wait needed.

### Step 2: Launch ws_rpi Control System

> **Launch RPi before Jetson.** RPi D5 nodes must be advertising on the network before
> Jetson's `rover_kinematic_control` joins D5, otherwise kinematic control misses SPDP.

**On Raspberry Pi** (`ssh curry@192.168.1.1`):
```bash
cd ~/almondmatcha/ws_rpi
source install/setup.bash
./launch_rover_tmux.sh
```

This creates a 9-pane tmux session (`rover`) running 8 nodes:
- Pane 0: `gnss_spresense_node` — Spresense GPS (D5 pub)
- Pane 1: `chassis_controller_node` — motor command relay (D5 sub/pub)
- Pane 2: `mission_monitoring_node_rpi` — telemetry bridge (D5 sub / D4 pub)
- Pane 3: `gnss_ublox_node` — RTK GNSS centimeter-level (D5 pub)
- Pane 4: `gnss_mission_monitor_node` — waypoint state machine (D5)
- Pane 5: `chassis_imu_node` — IMU data relay (D5 sub)
- Pane 6: `chassis_sensors_node` — encoder/power relay (D5 sub)
- Pane 7: `rover_monitoring_node` — CSV logger (D5 sub)
- Pane 8: spare shell

**Tmux:** `Ctrl+b`+arrows / `z` zoom / `d` detach / `tmux attach -t rover`

Wait 3–5 seconds before launching Jetson.

### Step 3: Launch ws_jetson Vision System

**Option A: Tmux launch (recommended):**
```bash
cd ~/almondmatcha/ws_jetson
source install/setup.bash
./launch_jetson_tmux.sh
```

This creates a 5-pane tmux session (`jetson_vision`):
- Pane 0: `camera_stream_node` — D415 RGB/depth streaming (Domain 6)
- Pane 1: `lane_detection_node` — Lane feature extraction (Domain 6)
- Pane 2: `rover_kinematic_control` — bicycle-model PID (D6 sub / D5 pub)
- Pane 3: `rover_local_monitoring_node` — telemetry CSV logger (Domain 4)
- Pane 4: `camera_recorder_node` — raw video of the RGB stream for offline debugging (Domain 6, throttled to 10 FPS @ 640×360 by default)

**Tmux controls:** `Ctrl+b` + arrows to navigate · `z` to zoom · `d` to detach

**Option B: Background launch script:**
```bash
cd ~/almondmatcha/ws_jetson
source install/setup.bash
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
[rover_kinematic_control]: Rover Kinematic Control node initialized
```

Wait for all nodes to be ready (30 FPS messages flowing).

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
# Expected: /camera_stream_node, /lane_detection_node, /camera_recorder_node

ros2 topic list
# Expected: /tpc_rover_d415_rgb, /tpc_rover_d415_depth, /tpc_rover_nav_lane

ros2 topic hz /tpc_rover_nav_lane
# Expected: ~30 Hz
```

### Domain 5 (Control) - On Any System

```bash
export ROS_DOMAIN_ID=5
ros2 node list
# Expected: 12 nodes total (11 without ws_base, 12 with ws_base)
# /rover_kinematic_control        (Jetson — dual-context D6 sub / D5 pub)
# /chassis_controller_node        (ws_rpi)
# /gnss_mission_monitor_node      (ws_rpi)
# /gnss_spresense_node            (ws_rpi)
# /gnss_ublox_node                (ws_rpi)
# /chassis_imu_node               (ws_rpi)
# /chassis_sensors_node           (ws_rpi)
# /mission_monitoring_node_rpi    (ws_rpi — D5 sub / D4 pub)
# /rover_monitoring_node          (ws_rpi — D5 CSV logger)
# /chassis_controller             (STM32 chassis)
# /sensors_node                   (STM32 sensors)
# /mission_command_node           (ws_base, if launched)

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
export ROS_DOMAIN_ID=6
ros2 topic list | grep nav_lane       # Confirm D6 running
export ROS_DOMAIN_ID=5
ros2 node info /rover_kinematic_control  # Confirm D5 subscription
```

### High STM32 Memory Usage
`[MemoryPool] RESSOURCE LIMIT EXCEEDED` — check D5 participant count:
```bash
export ROS_DOMAIN_ID=5
ros2 node list | wc -l  # Should be 9–11
```
Check `mros2-mbed-*/platform/rtps/config.h` for production participant limits and pool sizing.

### Topics Not Visible
```bash
echo $ROS_DOMAIN_ID          # Verify correct domain
ros2 daemon stop && ros2 daemon start
ping 192.168.1.1             # RPi reachable?
```

---

**See Also:** [DOMAINS.md](DOMAINS.md) · [ARCHITECTURE.md](ARCHITECTURE.md) · [TOPICS.md](TOPICS.md) · [STM32_CHANGES_SUMMARY.md](STM32_CHANGES_SUMMARY.md) · [EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md)
