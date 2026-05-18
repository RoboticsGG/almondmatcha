# Launch Scripts

## POC Experiment Scripts (full measurement runs)

| Script | Mode | Description |
|--------|------|-------------|
| `launch_poc_lab.sh` | **Laboratory** | Camera fallback to video file; verbose console; lab params from `params/lab/` |
| `launch_poc_field.sh` | **On-Field** | Live D415 camera only; Ctrl-C = emergency stop with full process cleanup; field params from `params/field/` |
| `launch_poc_experiment.sh` | Legacy | Original single-domain POC script (superseded by lab/field split) |

## Base Station Node Launchers

| Script | Tool | Purpose |
|--------|------|---------|
| `launch_base_single_domain.sh` | tmux | Base PC nodes for single-domain POC (`base_poc` session) |
| `launch_base_tmux.sh` | tmux | Base PC nodes for multi-domain baseline |
| `launch_base_screen.sh` | GNU screen | Base PC nodes (screen, no tmux dependency) |
| `launch_monitoring.sh` | bash | Monitoring node only (legacy/standalone) |

## Quick Launch — POC Experiments

```bash
cd ~/almondmatcha

# Laboratory run (camera fallback enabled, verbose output)
bash ws_base/launch_poc_lab.sh
bash ws_base/launch_poc_lab.sh --duration 600   # 10-minute run
bash ws_base/launch_poc_lab.sh --verbose        # extra debug logging

# On-field run (live camera mandatory, Ctrl-C = emergency stop)
bash ws_base/launch_poc_field.sh
bash ws_base/launch_poc_field.sh --duration 600
```

## On-Field Tuning Workflow

1. **Edit gains** before the run (no need to navigate `src/`):
   ```bash
   nano ws_base/params/field/control.yaml          # controller gains
   nano ws_base/params/field/mission_command.yaml  # target GPS coords + speed
   ```
2. **Run:** `bash ws_base/launch_poc_field.sh`
3. **Emergency stop:** `Ctrl-C` — all nodes and collectors cleanly terminated
4. **Results:** `ls ws_base/runs/single_domain/`

See [ws_base/params/README.md](../params/README.md) for full params documentation.

## Quick Launch — Base Station Nodes Only

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
