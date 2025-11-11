# ws_jetson - Vision Navigation Workspace

ROS2 workspace for real-time lane detection and visual navigation on NVIDIA Jetson Orin Nano.

## Hardware

- **Platform:** Jetson Orin Nano 8GB
- **Camera:** Intel RealSense D415 RGB-D (USB 3.0)
- **Network:** 192.168.1.5/24 (Gigabit Ethernet)
- **Domains:** 5 (control output), 6 (vision processing)

## Quick Start

**Single-command tmux launch (recommended):**
```bash
cd ~/almondmatcha/ws_jetson
./build_clean.sh
source install/setup.bash
./launch_jetson_tmux.sh  # Launches all nodes in organized tmux session
```

**Manual multi-terminal launch:**
```bash
# Terminal 1: Vision processing (Domain 6)
export ROS_DOMAIN_ID=6
ros2 launch vision_navigation vision_domain6.launch.py

# Terminal 2: Control interface (Domain 5)
export ROS_DOMAIN_ID=5
ros2 launch vision_navigation control_domain5.launch.py
```

## Architecture

The system uses a multi-domain design to optimize performance:

**Domain 6 (Vision Processing - Localhost):**
- Camera streaming (30 FPS, high bandwidth)
- Lane detection (heavy computation)
- Isolated from network, invisible to STM32 boards

**Domain 5 (Control Interface - Network):**
- Lightweight steering control (50 Hz)
- Subscribes to Domain 6 via localhost DDS
- Publishes to rover control network

This architecture reduces STM32 discovery overhead and enables scalable vision/AI expansion.


## Nodes

| Node | Domain | Function | Rate |
|------|--------|----------|------|
| `camera_stream` | 6 | D415 RGB/depth streaming | 30 FPS |
| `lane_detection` | 6 | Lane parameter extraction | 25-30 FPS |
| `steering_control_domain5` | 5 | PID steering control | 50 Hz |

The steering control node bridges domains: subscribes to Domain 6 vision data, publishes Domain 5 control commands.

## Building

**Clean build (recommended):**
```bash
./build_clean.sh
source install/setup.bash
```

**Incremental build:**
```bash
./build_inc.sh
source install/setup.bash
```

## Running

**Tmux launch (single command - recommended):**

All nodes in organized tmux session:
```bash
./launch_jetson_tmux.sh
```

Tmux controls:
- `Ctrl+b` then arrow keys: Navigate between panes
- `Ctrl+b z`: Zoom current pane (fullscreen toggle)
- `Ctrl+b [`: Scroll mode (q to exit, arrows to navigate)
- `Ctrl+d`: Close current pane
- `tmux kill-session -t jetson_vision`: Kill entire session

**Quick launch scripts:**

Headless mode (no GUI):
```bash
./launch_headless.sh  # Launches both Domain 6 and Domain 5 automatically
```

GUI mode (with visualization):
```bash
./launch_gui.sh  # Launches both Domain 6 (with GUI) and Domain 5 automatically
```

Both scripts handle:
- Automatic domain separation
- Proper startup sequence (vision first, then control)
- Background process management
- Clean shutdown on Ctrl+C

**Manual multi-domain launch (development):**

Terminal 1 - Vision processing:
```bash
export ROS_DOMAIN_ID=6
ros2 launch vision_navigation vision_domain6.launch.py
```

Terminal 2 - Control interface:
```bash
export ROS_DOMAIN_ID=5
ros2 launch vision_navigation control_domain5.launch.py
```

## Configuration

**System configuration:**
- `vision_nav_gui.yaml` - GUI mode (visualization enabled)
- `vision_nav_headless.yaml` - Headless mode (no visualization)

**Control tuning:**
- `steering_control_params.yaml` - PID gains, filtering, limits

Example PID tuning:
```yaml
steering_control:
  ros__parameters:
    k_p: 4.5      # Proportional gain
    k_i: 0.1      # Integral gain
    k_d: 0.15     # Derivative gain
    k_e1: 0.7     # Heading error weight
    k_e2: 0.3     # Lateral offset weight
    ema_alpha: 0.3
```

Changes take effect on next launch (no rebuild required).

## Tmux Session Management

**Launch organized session:**
```bash
./launch_jetson_tmux.sh  # Creates 'jetson_vision' session with 3 panes
```

**Session layout:**
```
┌─────────────────────┬─────────────────────┐
│                     │ [1] Lane Detection  │
│  [0] Camera Stream  │     (Domain 6)      │
│     (Domain 6)      ├─────────────────────┤
│                     │ [2] Steering Ctrl   │
│                     │     (Domain 5)      │
└─────────────────────┴─────────────────────┘
```

**Tmux cheat sheet:**

Navigation:
- `Ctrl+b` → arrow keys: Switch between panes
- `Ctrl+b` → `z`: Zoom/unzoom current pane (fullscreen)
- `Ctrl+b` → `o`: Cycle through panes
- `Ctrl+b` → `q`: Show pane numbers

Scrolling/History:
- `Ctrl+b` → `[`: Enter scroll mode
  - Arrow keys or PgUp/PgDn: Navigate
  - `q`: Exit scroll mode

Session management:
- `Ctrl+b` → `d`: Detach from session (keeps running)
- `tmux attach -t jetson_vision`: Reattach to session
- `tmux ls`: List all sessions
- `tmux kill-session -t jetson_vision`: Kill session

Pane management:
- `Ctrl+d`: Close current pane
- `exit`: Close current pane (alternative)

**Reattaching to running session:**
```bash
tmux attach -t jetson_vision  # Reconnect to existing session
```

## Verification

**Check Domain 6 (vision):**
```bash
export ROS_DOMAIN_ID=6
ros2 node list  # Should show: /camera_stream, /lane_detection
ros2 topic hz /tpc_rover_nav_lane  # Should be ~30 Hz
```

**Check Domain 5 (control):**
```bash
export ROS_DOMAIN_ID=5
ros2 node list  # Should show: /steering_control_domain5, /chassis_controller, etc.
ros2 topic list  # Should NOT show camera topics
ros2 topic hz /tpc_rover_fmctl  # Should be ~50 Hz
```

**Verify cross-domain communication:**
```bash
# Both should show data flowing
ros2 topic echo /tpc_rover_nav_lane  # Domain 6
ros2 topic echo /tpc_rover_fmctl     # Domain 5
```

## Troubleshooting

**Vision data not reaching control node:**
```bash
# Check Domain 6 running
export ROS_DOMAIN_ID=6
ros2 topic list | grep nav_lane

# Verify localhost discovery
ros2 topic info /tpc_rover_nav_lane -v
```

**Control commands not reaching rover:**
```bash
# Check Domain 5 publisher
export ROS_DOMAIN_ID=5
ros2 topic info /tpc_rover_fmctl -v
```

**Camera not detected:**
```bash
lsusb | grep Intel
rs-enumerate-devices
sudo usermod -aG video $USER  # Then logout/login
```

**Low frame rate:**
- Reduce resolution in YAML config
- Disable visualization (`show_window: false`)
- Check CPU usage with `htop`

**Topics not visible:**
```bash
echo $ROS_DOMAIN_ID  # Verify correct domain
ros2 daemon stop && ros2 daemon start
```

## Performance Baselines

| Metric | Target | Typical |
|--------|--------|---------|
| Camera FPS | 30 | 30 |
| Lane detection FPS | 25 | 25-30 |
| Steering update rate | 50 Hz | 50 Hz |
| End-to-end latency | <150 ms | 100-120 ms |
| CPU usage | <60% | 40-50% |

## Topics

**Domain 6 (localhost only):**
- `tpc_rover_d415_rgb` - RGB stream (30 FPS)
- `tpc_rover_d415_depth` - Depth stream (30 FPS)
- `tpc_rover_nav_lane` - Lane parameters [theta, b, detected] (30 FPS)

**Domain 5 (network-wide):**
- `tpc_rover_fmctl` - Steering commands [angle, detected] (50 Hz)

Steering commands are consumed by `node_chassis_controller` (ws_rpi) and converted to motor commands for STM32.

## See Also

- [../docs/DOMAINS.md](../docs/DOMAINS.md) - Multi-domain architecture
- [../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md) - System overview
- [docs/architecture.md](docs/architecture.md) - Detailed vision pipeline
- [docs/configuration.md](docs/configuration.md) - Parameter tuning guide
