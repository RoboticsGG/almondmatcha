# System Launch Instructions

Complete guide to launching the Almondmatcha rover system across all platforms.

## Quick Launch Summary

> **Branch: `multi-domain`** — Use the `_multi_domain` scripts on every machine. D4 (telemetry), D5 (control/STM32), D6 (vision).

**Recommended launch order:**

1. **STM32 boards:** power on, wait ~5 s for lwIP network init
2. **Raspberry Pi (ws_rpi):** `./launch_rover_multi_domain.sh`
3. **Jetson (ws_jetson):** `./launch_jetson_multi_domain.sh`
4. **Base Station (ws_base):** `./launch_base_multi_domain.sh` (optional)

## System Overview

- **Domain 5 (control):** RPi (8 nodes), Base (mission_command_node), Jetson (kinematic_control D5 pub), STM32×2 — ~12 participants
- **Domain 6 (vision):** Jetson localhost — camera_stream_node, lane_detection_node (shared memory, 2 nodes)
- **Domain 4 (telemetry):** RPi (D4 pub internal), Base (mission_monitoring_node_pc), Jetson (rover_local_monitoring_node) — 3 nodes
- **Total:** 16 nodes across 5 machines

### Domain 5 Participants (~12)

- ws_rpi: 8 nodes (gnss_spresense_node, gnss_ublox_node, gnss_mission_monitor_node, chassis_controller_node, chassis_imu_node, chassis_sensors_node, mission_monitoring_node_rpi, rover_monitoring_node)
- ws_base: 1 node on D5 (mission_command_node)
- ws_jetson: 1 node on D5 (rover_kinematic_control — D5 pub side)
- STM32: 2 nodes (chassis_controller, sensors_node)

> D6 vision nodes use shared memory on Jetson localhost — invisible to STM32. D4 telemetry nodes don't interact with STM32.

### Prerequisites

1. All systems powered on and connected to network (192.168.1.0/24)
2. STM32 firmware built and flashed (same firmware on both branches)
3. All ROS2 workspaces built (`build_inc.sh` on each machine)

**Network IPs:**

| Device | IP | SSH |
|--------|-----|-----|
| Raspberry Pi | 192.168.1.1 | `curry@192.168.1.1` |
| Jetson Orin | 192.168.1.5 | `yupi@192.168.1.5` |
| STM32 Chassis | 192.168.1.2 | — (mROS2) |
| STM32 Sensors | 192.168.1.6 | — (mROS2) |
| Base Station | 192.168.1.4 | `yupi@192.168.1.4` |

## Launch Sequence

### Step 1: Power On STM32 Boards

Connect STM32 boards to the Ethernet switch and power on. Allow ~5 seconds for lwIP network stack initialization (static IP, UDP socket open). Discovery happens automatically once other nodes start sending SPDP announcements.

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
```

### Step 2: Launch ws_rpi Control System

**On Raspberry Pi** (`ssh curry@192.168.1.1`):
```bash
cd ~/almondmatcha/ws_rpi
./launch_rover_multi_domain.sh
```

This launches all 8 RPi nodes in a tmux session (`rover`):
- GNSS nodes (Spresense, u-blox, mission monitor)
- Chassis nodes (controller, IMU relay, sensors relay)
- Monitoring nodes (mission_monitoring_node_rpi with D5→D4 bridge, rover_monitoring_node CSV logger)

**Tmux:** `Ctrl+b`+arrows / `z` zoom / `d` detach / `tmux attach -t rover`

Wait 3-5 seconds before next system.

### Step 3: Launch ws_jetson Vision System

**On Jetson** (`ssh yupi@192.168.1.5`):
```bash
cd ~/almondmatcha/ws_jetson
./launch_jetson_multi_domain.sh
```

This launches 4 Jetson nodes across 3 domains:
- **D6:** camera_stream_node + lane_detection_node (shared memory)
- **D5:** rover_kinematic_control (D6 sub → D5 pub, dual context)
- **D4:** rover_local_monitoring_node (CSV backup)

**Expected output:**
```
[camera_stream_node]: Starting camera stream at 30 FPS
[lane_detection_node]: Lane detection pipeline initialized
[rover_kinematic_control]: Rover Kinematic Control node initialized on Domain 6
```

Wait 3-5 seconds before next system.

### Step 4: Launch ws_base Mission Control (Optional)

**On Base Station** (`192.168.1.4`):
```bash
cd ~/almondmatcha/ws_base
./launch_base_multi_domain.sh
```

This launches 2 Base nodes:
- **D5:** mission_command_node (action client + service client)
- **D4:** mission_monitoring_node_pc (telemetry display)

## Verification Checklist

### Domain 5 — Control Network

```bash
export ROS_DOMAIN_ID=5
ros2 node list
# Expected (~12 nodes):
# /chassis_controller_node        (ws_rpi)
# /gnss_mission_monitor_node      (ws_rpi)
# /gnss_spresense_node            (ws_rpi)
# /gnss_ublox_node                (ws_rpi)
# /chassis_imu_node               (ws_rpi)
# /chassis_sensors_node           (ws_rpi)
# /mission_monitoring_node_rpi    (ws_rpi, D5 sub side)
# /rover_monitoring_node          (ws_rpi)
# /rover_kinematic_control        (Jetson, D5 pub side)
# /chassis_controller             (STM32 chassis)
# /sensors_node                   (STM32 sensors)
# /mission_command_node           (ws_base, if launched)

ros2 topic hz /tpc_rover_ctrl_cmd     # ~50 Hz
ros2 topic hz /tpc_chassis_imu        # ~10 Hz
ros2 topic hz /tpc_chassis_sensors    # ~4 Hz
```

### Domain 4 — Telemetry

```bash
export ROS_DOMAIN_ID=4
ros2 node list
# Expected (3 nodes):
# /mission_monitoring_domain4_pub   (RPi, internal D4 publisher)
# /mission_monitoring_node_pc       (ws_base)
# /rover_local_monitoring_node      (ws_jetson)

ros2 topic echo /tpc_telemetry_relay --once   # 5 Hz from RPi
```

### Domain 6 — Vision (Jetson only)

```bash
# On Jetson:
export ROS_DOMAIN_ID=6
ros2 node list
# Expected (2-3 nodes):
# /camera_stream_node              (ws_jetson)
# /lane_detection_node             (ws_jetson)
# /rover_kinematic_control         (ws_jetson, D6 sub side)

ros2 topic hz /tpc_rover_d415_rgb    # ~30 FPS
ros2 topic hz /tpc_rover_nav_lane    # ~25-30 FPS
```

## Shutdown

```bash
# On Jetson:
tmux kill-session -t jetson_vision

# On RPi:
tmux kill-session -t rover

# On Base: Ctrl+C or kill tmux session
```

Power down STM32 boards last.

## Common Issues

### STM32 Boards Not Visible
```bash
ping 192.168.1.2  # STM32 chassis
ping 192.168.1.6  # STM32 sensors
minicom -D /dev/ttyACM0  # Check serial console
```

### Vision Data Not Reaching Control
```bash
# On Jetson — verify D6 topics
export ROS_DOMAIN_ID=6
ros2 topic list | grep nav_lane       # Confirm lane topic on D6

# Verify D5 control output
export ROS_DOMAIN_ID=5
ros2 topic hz /tpc_rover_ctrl_cmd     # Should be ~50 Hz
```

### High STM32 Memory Usage
`[MemoryPool] RESSOURCE LIMIT EXCEEDED` — check D5 participant count:
```bash
export ROS_DOMAIN_ID=5
ros2 node list | wc -l  # Should be ~12 (D5 nodes only)
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
