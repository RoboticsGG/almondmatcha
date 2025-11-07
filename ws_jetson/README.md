# ws_jetson - Vision Navigation Workspace

ROS2 workspace for real-time lane detection and visual navigation on NVIDIA Jetson Orin Nano.

## Quick Start

```bash
cd ~/almondmatcha/ws_jetson
./build_clean.sh
source install/setup.bash
./launch_headless.sh     # Production
./launch_gui.sh          # Development with visualization
```

## Hardware

- **Platform:** Jetson Orin Nano 8GB Developer Kit
- **Camera:** Intel RealSense D415 RGB-D (USB 3.0)
- **Network:** Static IP 192.168.1.5
- **Domain:** ROS2 Domain 5

## Nodes

| Node | Function | Rate |
|------|----------|------|
| `camera_stream` | D415 RGB/depth streaming | 30 FPS |
| `lane_detection` | Lane parameter extraction | 25-30 FPS |
| `steering_control` | PID-based steering commands | 50 Hz |

## Building

### Clean Build (Recommended)

```bash
cd ~/almondmatcha/ws_jetson
./build_clean.sh
source install/setup.bash
```

Removes all build artifacts (`build/`, `install/`, `log/`) for fresh build.

### Incremental Build

```bash
cd ~/almondmatcha/ws_jetson
./build_inc.sh
source install/setup.bash
```

Faster for minor code changes (preserves previous build artifacts).

### Manual Build

```bash
cd ~/almondmatcha/ws_jetson
colcon build --packages-select vision_navigation
source install/setup.bash
```

## Running

### Launch Scripts

**Headless Mode (Production):**
```bash
./launch_headless.sh
```
- No GUI windows
- Optimized for performance
- Uses `vision_nav_headless.yaml` config

**GUI Mode (Development):**
```bash
./launch_gui.sh
```
- Camera preview windows
- Lane detection visualization
- Uses `vision_nav_gui.yaml` config

### Manual Launch

```bash
export ROS_DOMAIN_ID=5
cd ~/almondmatcha/ws_jetson
source install/setup.bash

# GUI mode
ros2 launch vision_navigation vision_nav_gui.launch.py

# Headless mode
ros2 launch vision_navigation vision_nav_headless.launch.py
```

### Individual Nodes

```bash
export ROS_DOMAIN_ID=5
source install/setup.bash

# Camera stream
ros2 run vision_navigation camera_stream --ros-args \
    --params-file config/vision_nav_gui.yaml

# Lane detection
ros2 run vision_navigation lane_detection --ros-args \
    --params-file config/vision_nav_gui.yaml

# Steering control
ros2 run vision_navigation steering_control --ros-args \
    --params-file config/vision_nav_gui.yaml \
    --params-file config/steering_control_params.yaml
```

## Configuration

### YAML Files

Configuration separated by function:

**System Configuration:**
- `vision_nav_gui.yaml` - GUI mode settings (camera preview on, visualization on)
- `vision_nav_headless.yaml` - Headless mode (all visualization off)

**Control Tuning:**
- `steering_control_params.yaml` - PID gains, filtering, limits

**Example: Tuning PID Controller**

```bash
cd ~/almondmatcha/ws_jetson/vision_navigation/config
nano steering_control_params.yaml
```

```yaml
steering_control:
  ros__parameters:
    # PID gains
    k_p: 4.5          # Proportional gain
    k_i: 0.1          # Integral gain
    k_d: 0.15         # Derivative gain
    
    # Error weighting
    k_e1: 0.7         # Heading error weight
    k_e2: 0.3         # Lateral offset weight
    
    # Filtering
    ema_alpha: 0.3    # Exponential moving average
    
    # Limits
    max_steering: 45.0    # degrees
    min_steering: -45.0
```

After editing, relaunch (no rebuild needed):
```bash
./launch_gui.sh
```

### Network Setup

Set static IP on Jetson:

```bash
# Temporary
sudo ip addr add 192.168.1.5/24 dev eth0
sudo ip link set eth0 up

# Permanent (NetworkManager)
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.5/24
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con up "Wired connection 1"
```

Verify connectivity:

```bash
ping 192.168.1.1    # Raspberry Pi
ping 192.168.1.2    # STM32 chassis
```

## Testing

### Verify Camera

```bash
# List RealSense devices
rs-enumerate-devices

# Expected output:
# Device 0: Intel RealSense D415
# Serial Number: 806312060441
# Firmware Version: 05.12.14.00
```

### Monitor Topics

```bash
export ROS_DOMAIN_ID=5

# Camera stream (should be 30 Hz)
ros2 topic hz tpc_rover_d415_rgb

# Lane detection output
ros2 topic echo tpc_rover_nav_lane

# Steering commands
ros2 topic echo tpc_rover_fmctl
```

### Check Performance

```bash
# Node CPU usage
top -p $(pgrep -f camera_stream)
top -p $(pgrep -f lane_detection)

# Topic latency
ros2 topic delay tpc_rover_nav_lane
```

## Troubleshooting

### Camera Not Detected

**Symptom:** `RuntimeError: Cannot open camera`

**Solutions:**
```bash
# Check USB connection
lsusb | grep Intel

# Reinstall librealsense
sudo apt update
sudo apt install librealsense2-utils
rs-enumerate-devices

# Check permissions
sudo usermod -aG video $USER
# Log out and log back in
```

### Low Frame Rate

**Symptom:** Camera or lane detection < 25 FPS

**Solutions:**
```bash
# Reduce resolution in YAML config
width: 640
height: 480
fps: 30

# Disable visualization
show_window: false

# Check CPU usage
htop
```

### High CPU Usage

**Symptom:** Jetson thermal throttling or lag

**Solutions:**
```bash
# Switch to headless mode
./launch_headless.sh

# Reduce camera FPS
fps: 15

# Monitor temperature
tegrastats
```

### Topics Not Visible

**Symptom:** `ros2 topic list` shows no topics

**Solutions:**
```bash
# Verify domain
echo $ROS_DOMAIN_ID  # Should be 5

# Check network
ping 192.168.1.1  # RPi

# Restart daemon
ros2 daemon stop
ros2 daemon start
```

### Build Failures

**Symptom:** `colcon build` errors

**Solutions:**
```bash
# Clean rebuild
./build_clean.sh

# Check ROS2 environment
source /opt/ros/humble/setup.bash
./build_clean.sh

# Check Python dependencies
pip3 install opencv-python pyrealsense2
```

## Data Logging

Lane detection and steering control log to CSV files:

**Lane Detection:** `lane_pub_log.csv`
```
timestamp,theta,b,detected
2025-11-07T10:30:00.123,5.23,45.67,1.0
```

**Steering Control:** `logs/rover_ctl_log_ver_3.csv`
```
time_sec,theta_ema,b_ema,u,e_sum
1730708000.123,4.85,44.25,19.4,5.12
```

## Directory Structure

```
ws_jetson/
├── README.md                       # This file
├── build_clean.sh                  # Clean build script
├── build_inc.sh                    # Incremental build
├── launch_gui.sh                   # GUI mode launcher
├── launch_headless.sh              # Headless mode launcher
├── docs/                           # Detailed documentation
│   ├── architecture.md
│   ├── nodes.md
│   ├── configuration.md
│   └── troubleshooting.md
└── vision_navigation/              # Main package
    ├── package.xml
    ├── setup.py
    ├── config/                     # YAML configuration
    │   ├── vision_nav_gui.yaml
    │   ├── vision_nav_headless.yaml
    │   └── steering_control_params.yaml
    ├── launch/                     # ROS2 launch files
    │   ├── vision_nav_gui.launch.py
    │   └── vision_nav_headless.launch.py
    └── vision_navigation_pkg/      # Python source
        ├── camera_stream_node.py
        ├── lane_detection_node.py
        ├── steering_control_node.py
        ├── lane_detector.py
        ├── control_filters.py
        ├── helpers.py
        ├── config.py
        └── __init__.py
```

## Detailed Documentation

For in-depth technical details:

- [docs/architecture.md](docs/architecture.md) - System design, data flow, message types
- [docs/nodes.md](docs/nodes.md) - Node parameters, launching, testing
- [docs/configuration.md](docs/configuration.md) - Parameter tuning, custom configs
- [docs/troubleshooting.md](docs/troubleshooting.md) - Common issues, solutions

## Performance Baselines

| Metric | Target | Typical |
|--------|--------|---------|
| Camera FPS | 30 | 30 |
| Lane detection FPS | 25 | 25-30 |
| Steering update rate | 50 Hz | 50 Hz |
| End-to-end latency | <150 ms | 100-120 ms |
| CPU usage | <60% | 40-50% |

## System Integration

**Topics Published:**
- `tpc_rover_d415_rgb` - Camera RGB stream (30 FPS)
- `tpc_rover_d415_depth` - Camera depth stream (30 FPS)
- `tpc_rover_nav_lane` - Lane parameters [theta, b, detected] (30 FPS)
- `tpc_rover_fmctl` - Steering commands [angle, detected] (50 Hz)

**Topics Subscribed:**
- None (vision system is data source)

**Integration with Rover:**
- Steering commands consumed by `node_chassis_controller` (ws_rpi)
- Chassis controller converts to motor commands for STM32

---

**Platform:** Jetson Orin Nano 8GB  
**ROS2:** Humble  
**Domain:** 5  
**Camera:** Intel RealSense D415
