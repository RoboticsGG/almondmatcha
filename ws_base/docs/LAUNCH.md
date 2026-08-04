# Launch Scripts

## Production entry point

```bash
cd ~/almondmatcha
bash ws_base/launch_field.sh
```

`launch_field.sh` is the single-command production launcher — it SSHes to
the RPi (`launch_rover_tmux.sh`) and Jetson (`launch_jetson_tmux.sh`), then
runs `launch_base_tmux.sh` locally, in order with proper startup delays.
Ctrl-C in that terminal stops all three machines. See the root
[README.md](../../README.md) Quick Start and
[docs/LAUNCH_INSTRUCTIONS.md](../../docs/LAUNCH_INSTRUCTIONS.md) for the
full sequence and timing.

**Emergency stop from any other shell on the base PC** (e.g. while attached
to a tmux pane instead of the launch terminal):

```bash
bash ws_base/emergency_stop.sh
```

## Base-station-only scripts

Use these when the RPi and Jetson are already running and only the base
station needs to (re)start:

| Script | Tool | Notes |
|--------|------|-------|
| `launch_base_tmux.sh` | tmux | What `launch_field.sh` actually uses — recommended |
| `launch_base_screen.sh` | GNU screen | Manual alternative if tmux is unavailable/undesired |
| `launch_monitoring.sh` | — | Starts `mission_monitoring_node_pc` only (D4 display, no commanding) |

```bash
cd ~/almondmatcha/ws_base

# tmux (recommended, matches launch_field.sh)
./launch_base_tmux.sh

# GNU screen (alternative)
./launch_base_screen.sh
```

## Installation

```bash
# tmux
sudo apt install tmux

# GNU Screen
sudo apt install screen
```

## Layout

Both scripts create 2 panes/windows, each on its own domain:

```
Pane/Window 0: mission_command_node       (Domain 5) → sends goals/speed limits
Pane/Window 1: mission_monitoring_node_pc (Domain 4) → shows telemetry
```

## Controls

### Tmux

| Command | Action |
|---------|--------|
| `Ctrl+b →/←` | Navigate panes |
| `Ctrl+b z` | Zoom pane |
| `Ctrl+b [` | Scroll mode (q to exit) |
| `Ctrl+b d` | Detach |

**Reconnect:** `tmux attach -t base_station`
**Kill:** `tmux kill-session -t base_station`

### GNU Screen

| Command | Action |
|---------|--------|
| `Ctrl+a n` | Next window |
| `Ctrl+a p` | Previous window |
| `Ctrl+a 0-1` | Jump to window N |
| `Ctrl+a d` | Detach (keep running) |
| `Ctrl+a [` | Scroll mode (ESC to exit) |

**Reconnect:** `screen -r base_station`
**Kill:** `screen -S base_station -X quit`

## Features

- Auto-sources the workspace
- Sets the correct `ROS_DOMAIN_ID` per pane (5 for command, 4 for monitoring)
- Color-coded pane titles
- Session persistence (detach/reattach)

## Troubleshooting

**Permission denied:**
```bash
chmod +x ~/almondmatcha/ws_base/launch_base_*.sh ~/almondmatcha/ws_base/launch_field.sh
```

**Build missing:**
```bash
cd ~/almondmatcha/ws_base
colcon build
```

**ROS 2 not sourced:**
```bash
source /opt/ros/humble/setup.bash
```

**Manual run (no session manager):**
```bash
cd ~/almondmatcha/ws_base
source install/setup.bash

# Terminal 1 — Domain 5
export ROS_DOMAIN_ID=5
ros2 run mission_control mission_command_node

# Terminal 2 — Domain 4
export ROS_DOMAIN_ID=4
ros2 run mission_control mission_monitoring_node_pc
```
