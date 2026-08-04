# ws_base Quick Start

## Overview
Mission control base station — dual-domain: `mission_command_node` on
Domain 5, `mission_monitoring_node_pc` on Domain 4.

## Launch

```bash
cd ~/almondmatcha

# Production — single command, starts RPi + Jetson + base
bash ws_base/launch_field.sh
```

Base-station-only, if RPi/Jetson are already running:
```bash
cd ~/almondmatcha/ws_base

# Tmux (recommended, matches launch_field.sh)
./launch_base_tmux.sh

# GNU Screen (alternative)
./launch_base_screen.sh
```

## Nodes

| Node | Purpose | Domain |
|------|---------|--------|
| `mission_command_node` | Generate navigation goals and speed limits | 5 |
| `mission_monitoring_node_pc` | Display rover telemetry (from the D4 relay) | 4 |

## Configuration

Edit `src/mission_control/config/params.yaml`:

```yaml
mission_command_node:
  ros__parameters:
    rover_spd: 15                        # Speed limit (0-100%)
    des_lat: 8.007286                    # Target latitude (Thailand)
    des_long: 101.90203                  # Target longitude
    action_watchdog_timeout_sec: 20.0    # Cancel goal if no feedback this long
    mission_retry_sec: 5.0               # Retry interval until the mission is established
```

## Controls

### Tmux
- `Ctrl+b →/←` - Navigate panes
- `Ctrl+b z` - Zoom pane
- `Ctrl+b d` - Detach session
- `tmux attach -t base_station` - Reattach

### GNU Screen
- `Ctrl+a n/p` - Next/previous window
- `Ctrl+a 0-9` - Jump to window
- `Ctrl+a d` - Detach session
- `screen -r base_station` - Reattach

## Communication

**Domain 5 — calls:**
- `/des_data` (action_ifaces/DesData) - Navigation goal (action)
- `/srv_spd_limit` (services_ifaces/SpdLimit) - Speed limit (service)

**Domain 4 — subscribed topics:**
- `tpc_telemetry_relay` (`TelemetryRelay.msg`) - the single aggregated
  telemetry topic `mission_monitoring_node_pc` reads (GNSS, chassis
  command/sensor/IMU, lane, destination fields), published by
  `mission_monitoring_node_rpi` on the RPi at 5 Hz

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
# separate terminal:
export ROS_DOMAIN_ID=4
ros2 run mission_control mission_monitoring_node_pc

# Check topics
ros2 topic list | grep tpc_
```
