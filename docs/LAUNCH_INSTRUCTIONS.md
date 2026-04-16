# System Launch Instructions

Complete guide to launching the Almondmatcha rover system across all platforms.

## Quick Launch Summary

> **Branch: `single-domain`** — Use the `_single_domain` scripts on every machine. All nodes run on **ROS_DOMAIN_ID=5**.

**Recommended launch order:**

1. **STM32 boards:** power on, wait ~5 s for lwIP network init
2. **Raspberry Pi (ws_rpi):** `./launch_rover_single_domain.sh`
3. **Jetson (ws_jetson):** `./launch_jetson_single_domain.sh`
4. **Base Station (ws_base):** `./launch_base_single_domain.sh` (optional)

## System Overview

- **All communication on Domain 5**
- Total participants on D5: ~16 nodes

### Domain 5 Participants

- ws_rpi: 8 nodes (chassis_controller_node, chassis_imu_node, chassis_sensors_node, gnss_spresense_node, gnss_ublox_node, gnss_mission_monitor_node, mission_monitoring_node_rpi, rover_monitoring_node)
- ws_jetson: 4 nodes (camera_stream_node, lane_detection_node, rover_kinematic_control, rover_local_monitoring_node)
- ws_base: 2 nodes (mission_command_node, mission_monitoring_node_pc)
- STM32: 2 nodes (chassis_controller, sensors_node)

### Prerequisites

1. All systems powered on and connected to network (192.168.1.0/24)
2. STM32 firmware built and flashed
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
./launch_rover_single_domain.sh
```

This launches all 8 RPi nodes in a tmux session (`rover`):
- GNSS nodes (Spresense, u-blox, mission monitor)
- Chassis nodes (controller, IMU relay, sensors relay)
- Monitoring nodes (mission_monitoring_node_rpi, rover_monitoring_node)

**Tmux:** `Ctrl+b`+arrows / `z` zoom / `d` detach / `tmux attach -t rover`

Wait 3-5 seconds before next system.

### Step 3: Launch ws_jetson Vision System

**On Jetson** (`ssh yupi@192.168.1.5`):
```bash
cd ~/almondmatcha/ws_jetson
./launch_jetson_single_domain.sh
```

This launches 4 Jetson nodes (all on D5):
- camera_stream_node — D415 RGB/depth @ 30 FPS
- lane_detection_node — lane feature extraction
- rover_kinematic_control — bicycle-model PID @ 50 Hz
- rover_local_monitoring_node — CSV backup logger

**Expected output:**
```
[camera_stream_node]: Starting camera stream at 30 FPS
[lane_detection_node]: Lane detection pipeline initialized
[rover_kinematic_control]: Rover Kinematic Control node initialized
```

Wait 3-5 seconds before next system.

### Step 4: Launch ws_base Mission Control (Optional)

**On Base Station** (`192.168.1.4`):
```bash
cd ~/almondmatcha/ws_base
./launch_base_single_domain.sh
```

This launches 2 Base nodes:
- mission_command_node — action client + service client
- mission_monitoring_node_pc — telemetry display

## Verification Checklist

### Domain 5 — All Nodes

```bash
export ROS_DOMAIN_ID=5
ros2 node list
# Expected (~16 nodes):
# /chassis_controller_node        (ws_rpi)
# /chassis_imu_node               (ws_rpi)
# /chassis_sensors_node           (ws_rpi)
# /gnss_mission_monitor_node      (ws_rpi)
# /gnss_spresense_node            (ws_rpi)
# /gnss_ublox_node                (ws_rpi)
# /mission_monitoring_node_rpi    (ws_rpi)
# /rover_monitoring_node          (ws_rpi)
# /camera_stream_node             (ws_jetson)
# /lane_detection_node            (ws_jetson)
# /rover_kinematic_control        (ws_jetson)
# /rover_local_monitoring_node    (ws_jetson)
# /chassis_controller             (STM32 chassis)
# /sensors_node                   (STM32 sensors)
# /mission_command_node           (ws_base, if launched)
# /mission_monitoring_node_pc     (ws_base, if launched)

ros2 topic hz /tpc_rover_ctrl_cmd     # ~50 Hz
ros2 topic hz /tpc_chassis_imu        # ~10 Hz
ros2 topic hz /tpc_chassis_sensors    # ~4 Hz
ros2 topic hz /tpc_telemetry_relay    # ~5 Hz
ros2 topic hz /tpc_rover_d415_rgb     # ~30 FPS
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
export ROS_DOMAIN_ID=5
ros2 topic list | grep nav_lane       # Confirm lane topic visible on D5
ros2 topic hz /tpc_rover_nav_lane     # Confirm ~30 Hz
```

### High STM32 Memory Usage
`[MemoryPool] RESSOURCE LIMIT EXCEEDED` — check D5 participant count:
```bash
export ROS_DOMAIN_ID=5
ros2 node list | wc -l  # Should be ~16 (all nodes on D5)
```
See [STM32_RTPS_MEMORY_CALCULATION.md](STM32_RTPS_MEMORY_CALCULATION.md) for config details.

### Topics Not Visible
```bash
echo $ROS_DOMAIN_ID          # Verify = 5
ros2 daemon stop && ros2 daemon start
ping 192.168.1.1             # RPi reachable?
```

---

**See Also:** [DOMAINS.md](DOMAINS.md) · [ARCHITECTURE.md](ARCHITECTURE.md) · [TOPICS.md](TOPICS.md)
