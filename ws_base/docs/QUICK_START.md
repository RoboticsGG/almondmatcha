# ws_base Quick Start

## Overview
Mission control base station (Domain 5 - Unified Architecture) for rover command and telemetry monitoring.

## Launch

```bash
cd ~/almondmatcha/ws_base

# Option 1: GNU Screen (recommended)
./launch_base_screen.sh

# Option 2: Tmux
./launch_base_tmux.sh
```

## Nodes

| Node | Purpose | Domain |
|------|---------|--------|
| `mission_command_node` | Generate navigation goals and speed limits | 5 |
| `mission_monitoring_node` | Display rover telemetry | 5 |

## Configuration

Edit `src/mission_control/config/params.yaml`:

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
