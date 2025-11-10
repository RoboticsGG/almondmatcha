# Launch Scripts

## Available Scripts

| Script | Tool | Status |
|--------|------|--------|
| `launch_base_screen.sh` | GNU screen | ✅ Recommended |
| `launch_base_tmux.sh` | tmux | ⚠️ Requires tmux install |

## Quick Launch

```bash
cd ~/almondmatcha/ws_base

# Option 1: GNU Screen (most compatible)
./launch_base_screen.sh

# Option 2: Tmux (if installed)
./launch_base_tmux.sh
```

## Installation

```bash
# GNU Screen
sudo apt install screen

# Tmux
sudo apt install tmux
```

## Layout

Both scripts create 2 windows/panes:

```
Window 0: mission_command_node    → Sends goals
Window 1: mission_monitoring_node → Shows telemetry
```

## Controls

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

### Tmux

| Command | Action |
|---------|--------|
| `Ctrl+b →/←` | Navigate panes |
| `Ctrl+b z` | Zoom pane |
| `Ctrl+b [` | Scroll mode (q to exit) |
| `Ctrl+b d` | Detach |

**Reconnect:** `tmux a -t base_station`  
**Kill:** `tmux kill-session -t base_station`

## Features

✅ Auto-sources workspace  
✅ Sets `ROS_DOMAIN_ID=5`  
✅ Pre-flight validation  
✅ Color-coded output  
✅ Session persistence (detach/reattach)

## Troubleshooting

**Permission denied:**
```bash
chmod +x ~/almondmatcha/ws_base/launch_base_*.sh
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
export ROS_DOMAIN_ID=5

# Terminal 1
ros2 run mission_control mission_command_node

# Terminal 2
ros2 run mission_control mission_monitoring_node
```
