# Rover Bringup

## Overview

Centralized launch system for the Almondmatcha rover, managing all 8 ROS2 nodes
on Raspberry Pi across Domain 5 (control network) and Domain 4 (telemetry relay).

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                Domain 5 — Control Network                    │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ ws_rpi (Raspberry Pi 192.168.1.1):                   │   │
│  │   ├─ gnss_spresense_node        (Standard GPS)       │   │
│  │   ├─ gnss_ublox_node            (RTK GNSS)           │   │
│  │   ├─ gnss_mission_monitor_node  (Waypoint tracking)  │   │
│  │   ├─ chassis_controller_node    (Motor coordination) │   │
│  │   ├─ chassis_imu_node           (IMU data logger)    │   │
│  │   ├─ chassis_sensors_node       (Encoder/power log)  │   │
│  │   ├─ rover_monitoring_node      (full CSV logger)    │   │
│  │   └─ mission_monitoring_node_rpi (D5 sub + D4 relay) │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                              │
│  ws_jetson: rover_kinematic_control (dual-context D6/D5)    │
│  ws_base:   mission_command_node (actions/services to RPi)  │
│  STM32:     chassis_controller + sensors_node (mROS2)       │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                Domain 4 — Telemetry Relay                    │
│  mission_monitoring_node_rpi (RPi) → /tpc_telemetry_relay   │
│  Subscribers: mission_monitoring_node_pc (Base)              │
│               rover_local_monitoring_node (Jetson)           │
└──────────────────────────────────────────────────────────────┘
```

`mission_monitoring_node_rpi` subscribes 9 D5 topics and publishes aggregated
telemetry to D4 at 5 Hz — it does not log CSVs locally. `rover_monitoring_node`
is the separate full-fidelity logger: it subscribes 10 D5 topics (the same
set plus `tpc_chassis_speed_debug`) and writes 7 per-topic CSV files.

## Usage

### Launch All Rover Nodes

```bash
cd ~/almondmatcha/ws_rpi

# Using tmux (recommended)
./launch_rover_tmux.sh

# Or using ros2 launch
source install/setup.bash
ros2 launch rover_bringup rover_startup.launch.py
```

This launches 8 rover nodes on Domain 5 (+ D4 relay context):
- gnss_spresense_node, gnss_ublox_node, gnss_mission_monitor_node
- chassis_controller_node, chassis_imu_node, chassis_sensors_node
- rover_monitoring_node, mission_monitoring_node_rpi

### Launch ws_jetson (Vision System)

On Jetson Orin Nano:
```bash
cd ~/almondmatcha/ws_jetson
source install/setup.bash
# Domain 6 (vision) — localhost only
ros2 launch vision_navigation vision_domain6.launch.py
# Domain 5 (control)
ros2 launch vision_navigation control_domain5.launch.py
# Or single-command tmux launch:
./launch_jetson_tmux.sh
```

### Launch ws_base (Ground Station)

On base station computer:
```bash
cd ~/almondmatcha/ws_base
source install/setup.bash
# Script launches both domains automatically:
# - mission_command_node: Domain 5 (actions/services to RPi)
# - mission_monitoring_node_pc: Domain 4 (telemetry display)
./launch_base_tmux.sh
```

## Node Details

### Domain 5 Nodes (ws_rpi)

| Node | Package | Purpose | Topics |
|------|---------|---------|--------|
| `gnss_spresense_node` | gnss_navigation | Sony Spresense GPS reader | Pub: `tpc_gnss_spresense` |
| `gnss_ublox_node` | gnss_navigation | u-blox RTK GNSS reader | Pub: `tpc_gnss_ublox` |
| `gnss_mission_monitor_node` | gnss_navigation | Waypoint tracking | Sub: `tpc_gnss_spresense`<br>Pub: `tpc_gnss_mission_active`, `tpc_gnss_mission_remain_dist` |
| `chassis_controller_node` | chassis_control | Motor command coordination + closed-loop speed control | Sub: `tpc_rover_ctrl_cmd`, `tpc_gnss_mission_active`, `tpc_chassis_sensors`<br>Pub: `tpc_chassis_cmd`<br>Params: `config/chassis_speed_control_params.yaml` |
| `chassis_imu_node` | chassis_sensors | IMU data logger | Sub: `tpc_chassis_imu` |
| `chassis_sensors_node` | chassis_sensors | Encoder/power logger | Sub: `tpc_chassis_sensors` |
| `rover_monitoring_node` | rover_monitoring | Full-fidelity local logger | Sub: 10 D5 topics<br>CSV: 7 files in ws_rpi/runs/ (no D4 publish) |
| `mission_monitoring_node_rpi` | rover_monitoring | D5 aggregator → D4 relay | Sub: 9 D5 topics<br>Pub: D4 `/tpc_telemetry_relay` @ 5 Hz<br>No local CSV logging |

**`chassis_controller_node` closed-loop speed control:** holds target speed against
`tpc_chassis_sensors` encoder feedback (~4 Hz) instead of applying `spd_msg` PWM duty
open-loop; falls back to open-loop passthrough if the encoder feed goes stale.
Tuned via `chassis_control/config/chassis_speed_control_params.yaml`
(loaded by `rover_startup.launch.py`; `launch_rover_tmux.sh` passes it with
`--ros-args --params-file`). Kill-switch: `use_closed_loop_speed` (default `true`) —
`ros2 param set /chassis_controller_node use_closed_loop_speed false` reverts to
open-loop with no rebuild.

### Domain 5 Nodes (ws_base)

| Node | Package | Purpose |
|------|---------|---------|
| `mission_command_node` | mission_control | Send navigation goals (action `/des_data`) and speed limits (service `/srv_spd_limit`) |

### Domain 4 Nodes (ws_base, ws_jetson)

| Node | Package | Purpose |
|------|---------|---------|
| `mission_monitoring_node_pc` | mission_control | Subscribe `/tpc_telemetry_relay` — telemetry display |
| `rover_local_monitoring_node` | rover_monitoring | Subscribe `/tpc_telemetry_relay` — CSV logging in ws_jetson/runs/ |

## Communication (Domain 5)

**Actions (ws_base → ws_rpi):**
- `/des_data` — Navigation goals (latitude/longitude) → `gnss_mission_monitor_node`

**Services (ws_base → ws_rpi):**
- `/srv_spd_limit` — Speed limit commands (0-100%) → `chassis_controller_node`

**Topics (ws_rpi → rest of D5):**
- `tpc_chassis_imu` — IMU data (10 Hz)
- `tpc_chassis_sensors` — Encoder/power (4 Hz), also consumed by `chassis_controller_node` for closed-loop speed control
- `tpc_gnss_spresense` — GPS position (10 Hz)
- `tpc_gnss_ublox` — RTK GNSS position (10 Hz)
- `tpc_chassis_cmd` — Motor commands (50 Hz)
- `tpc_gnss_mission_active` — Mission status (10 Hz)
- `tpc_gnss_mission_remain_dist` — Distance to waypoint (10 Hz)

**Topics (Domain 4):**
- `tpc_telemetry_relay` — Aggregated state from RPi → Base + Jetson (5 Hz)

## Verification

### Check Domain 5 (All Systems)
```bash
export ROS_DOMAIN_ID=5
ros2 node list
# Expected (RPi running):
#   /gnss_spresense_node
#   /gnss_ublox_node
#   /gnss_mission_monitor_node
#   /chassis_controller_node
#   /chassis_imu_node
#   /chassis_sensors_node
#   /rover_monitoring_node
#   /mission_monitoring_node_rpi
ros2 topic hz /tpc_chassis_imu      # ~10 Hz
ros2 topic hz /tpc_gnss_spresense   # ~10 Hz
ros2 action list                     # Should show /des_data
ros2 service list                    # Should show /srv_spd_limit
```

### Check Domain 4 (Telemetry)
```bash
export ROS_DOMAIN_ID=4
ros2 node list
# Expected: /mission_monitoring_node_pc, /rover_local_monitoring_node
ros2 topic hz /tpc_telemetry_relay  # ~5 Hz
```

## Troubleshooting

### Nodes not appearing
```bash
export ROS_DOMAIN_ID=5
ros2 node list
ps aux | grep ros2
```

### STM32 topics missing
```bash
ping 192.168.1.2  # STM32 chassis
ping 192.168.1.6  # STM32 sensors
ros2 topic list | grep chassis
```

### D4 telemetry not reaching base
```bash
# On RPi: verify mission_monitoring_node_rpi running on D5
ROS_DOMAIN_ID=5 ros2 node list | grep mission_monitoring
# On base: verify domain
echo $ROS_DOMAIN_ID  # Should be 4
```

### Action/Service not working
```bash
export ROS_DOMAIN_ID=5
ros2 action list        # Should show /des_data
ros2 service list       # Should show /srv_spd_limit
```

## Author

Almondmatcha Development Team  
Last updated: August 4, 2026
