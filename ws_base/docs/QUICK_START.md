# ws_base Quick Start

## Overview
Mission control base station (Domain 5 - Unified Architecture) for rover command and telemetry monitoring.

## POC Experiment Launch (full runs)

```bash
cd ~/almondmatcha

# Laboratory run — camera fallback to video file, verbose output
bash ws_base/launch_poc_lab.sh
bash ws_base/launch_poc_lab.sh --duration 600   # 10-minute run

# On-field run — live camera mandatory, Ctrl-C = emergency stop
bash ws_base/launch_poc_field.sh
bash ws_base/launch_poc_field.sh --duration 600
```

Both scripts load parameters from `ws_base/params/lab/` or `ws_base/params/field/`
respectively — edit those files instead of navigating into `src/`.

## On-Field Tuning

```bash
# Edit before each field run:
nano ws_base/params/field/control.yaml          # k_p, k_i, k_d, ema_alpha
nano ws_base/params/field/mission_command.yaml  # des_lat, des_long, rover_spd
```

## Base Station Nodes Only (no collectors)

```bash
cd ~/almondmatcha/ws_base

# Option 1: GNU Screen (recommended)
./launch_base_screen.sh

# Option 2: Tmux
./launch_base_tmux.sh
```

## Nodes

| Node | Purpose | Domain | CSV Output |
|------|---------|--------|------------|
| `mission_command_node` | Generate navigation goals and speed limits | 5 | — |
| `mission_monitoring_node_pc` | Display rover telemetry + write CSV | 5 | `telemetry_relay_YYYYMMDD_HHMMSS.csv` |

The `mission_monitoring_node_pc` automatically writes a CSV of all received
`/tpc_telemetry_relay` messages to `~/almondmatcha_poc/` (default) or to the
run directory when launched via the POC scripts.

## Configuration

For POC runs, edit files in `ws_base/params/` (not `src/`):

```bash
# Lab run params
nano ws_base/params/lab/mission_command.yaml  # destination + speed
nano ws_base/params/lab/camera.yaml           # camera settings
nano ws_base/params/lab/control.yaml          # controller gains

# Field run params (edit before deployment)
nano ws_base/params/field/mission_command.yaml
nano ws_base/params/field/control.yaml
```

The source-of-truth defaults remain at `src/mission_control/config/params.yaml`
(used by non-POC launches).

Example mission_command parameters:

```yaml
mission_command_node:
  ros__parameters:
    rover_spd: 15      # Speed limit (0-100%)
    des_lat: 7.007286  # Target latitude (Thailand)
    des_long: 100.50203 # Target longitude
```

## Controls

### GNU Screen
- `Ctrl+a n/p` - Next/previous window
- `Ctrl+a 0-9` - Jump to window
- `Ctrl+a d` - Detach session
- `screen -r base_station` - Reattach

### Tmux
- `Ctrl+b →/←` - Navigate panes
- `Ctrl+b z` - Zoom pane
- `Ctrl+b d` - Detach session
- `tmux a -t base_station` - Reattach

## Communication

**Published Actions:**
- `/des_data` (action_ifaces/DesData) - Navigation goal

**Published Services:**
- `/spd_limit` (services_ifaces/SpdLimit) - Speed limit

**Subscribed Topics:**
- `tpc_gnss_spresense` (SpresenseGNSS) - GNSS position
- `tpc_gnss_mission_active` (Bool) - Mission status
- `tpc_gnss_mission_remain_dist` (Float64) - Distance remaining
- `tpc_chassis_cmd` (ChassisCtrl) - Chassis commands
- `tpc_rover_dest_coordinate` (Float64MultiArray) - Target coordinates

## Troubleshooting

```bash
# Build workspace
colcon build --packages-select mission_control

# Check ROS 2
echo $ROS_DISTRO
source /opt/ros/humble/setup.bash

# Manual run
export ROS_DOMAIN_ID=5
ros2 run mission_control mission_command_node
ros2 run mission_control mission_monitoring_node

# Check topics
ros2 topic list | grep tpc_
```
