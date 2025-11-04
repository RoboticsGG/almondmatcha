# Jetson Workspace - Visual Navigation System

Real-time visual navigation system for autonomous rover lane detection and steering control running on NVIDIA Jetson platform.

## Overview

The Jetson workspace (`ws_jetson`) provides camera streaming, lane detection, and closed-loop steering control for autonomous mobile rovers. It processes Intel RealSense D415 camera data or video files to detect lane boundaries and commands steering based on lane position feedback.

### Key Components

- **Camera Stream Node**: Streams RGB/depth frames from D415 camera or video files
- **Lane Detection Node**: Processes frames to detect lane markers and compute navigation parameters
- **Steering Control Node**: Implements closed-loop PID steering controller
- **Control Filters**: Reusable low-pass filters and control utilities
- **Lane Detector**: Image processing pipeline for lane boundary detection

## System Architecture

### Data Flow

```
Camera Input (D415 or Video)
    |
    v
[Camera Stream Node] -> /tpc_rover_d415_rgb (RGB frames)
    |                -> /tpc_rover_d415_depth (Depth frames, optional)
    v
[Lane Detection Node] -> /tpc_rover_nav_lane (Lane parameters)
    |                 -> lane_pub_log.csv (Detection logging)
    v
[Steering Control Node] -> /tpc_rover_fmctl (Steering command)
                        -> logs/rover_ctl_log_ver_3.csv (Control logging)
    v
Steering Actuator (Front Module)
```

## Prerequisites

- ROS 2 (tested on Humble)
- Python 3.10+
- Build system: `colcon`
- Dependencies:
  - `python3-opencv`
  - `python3-numpy`
  - `pyrealsense2` (for D415 camera)
  - `rclpy`
  - `cv_bridge`
  - `sensor_msgs`
  - `std_msgs`

### Install Dependencies

```bash
sudo apt update
sudo apt install python3-opencv python3-numpy ros2-dev
pip3 install pyrealsense2
```

## Building

### Quick Build

```bash
cd ~/almondmatcha/ws_jetson
colcon build --packages-select vision_navigation
source install/setup.bash
```

### Clean Rebuild

```bash
cd ~/almondmatcha/ws_jetson
rm -rf build install log
colcon build --packages-select vision_navigation
source install/setup.bash
```

### Build Output

- **Location**: Relative to workspace root (`ws_jetson/`)
- **Build artifacts**: `ws_jetson/build/`
- **Install directory**: `ws_jetson/install/`
- **ROS logs**: `ws_jetson/log/`
- **Data logs**: `ws_jetson/logs/`

## Build Scripts

To simplify building the workspace, two scripts are provided:

### Clean Build Script

Use this script to remove all previous build, install, and log artefacts before building:

```bash
./build_clean.sh
```
- Removes `build/`, `install/`, and `log/` directories before building
- Sources ROS2 setup if available
- Runs `colcon build --symlink-install`
- Recommended for a fresh build after major changes

### Incremental Build Script

Use this script for a regular build that preserves previous build artefacts:

```bash
./build_inc.sh
```
- Does not remove old build artefacts
- Sources ROS2 setup if available
- Runs `colcon build --symlink-install`
- Recommended for quick rebuilds after minor changes

Both scripts are located in the workspace root (`ws_jetson/`). Make sure they are executable:

```bash
chmod +x build_clean.sh build_inc.sh
```

## Running

### GUI Mode vs Headless Mode

The vision navigation system supports two modes of operation:

**Headless Mode (SSH/No Display)**  
Run without display windows when connected via SSH or when no monitor is attached:
```bash
# All visualization disabled (default)
camera_stream
lane_detection
steering_control
```

**GUI Mode (With Monitor)**  
Enable visualization windows for debugging and testing when a monitor is connected:
```bash
# Camera stream with preview
camera_stream --ros-args -p open_cam:=True

# Lane detection with visualization
lane_detection --ros-args -p show_window:=True
```

### Individual Node Execution

#### Camera Stream Node

Stream from D415 camera with default settings (headless):

```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
camera_stream
```

Stream with preview window enabled (GUI mode):

```bash
camera_stream --ros-args -p open_cam:=True
```

Stream from video file:

```bash
camera_stream --ros-args -p video_path:="/path/to/video.mp4" -p open_cam:=True
```

Stream D415 with depth enabled:

```bash
camera_stream --ros-args -p enable_depth:=True
```

#### Lane Detection Node

Process frames with visualization:

```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
lane_detection --ros-args -p show_window:=True
```

#### Steering Control Node

Run with default PID parameters:

```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
steering_control
```

Run with custom control gains:

```bash
steering_control --ros-args \
  -p k_p:=5.0 -p k_i:=0.1 -p k_d:=0.05 \
  -p steer_max_deg:=45.0
```

### Complete System Startup

The system provides three launch files for different use cases:

#### 1. Headless Mode (Production/SSH)

**Recommended for:** Production deployment, SSH sessions, remote operation

```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
ros2 launch vision_navigation vision_nav_headless.launch.py
```

- ✅ No GUI windows (optimized for headless operation)
- ✅ Lower resource usage
- ✅ Ideal for SSH access without X11 forwarding
- ✅ All nodes started with proper timing

#### 2. GUI Mode (Debugging/Testing)

**Recommended for:** Local debugging, testing with monitor connected

```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
ros2 launch vision_navigation vision_nav_gui.launch.py
```

- ✅ Camera preview window (RGB + Depth if enabled)
- ✅ Lane detection visualization
- ✅ Ideal for debugging and parameter tuning
- ✅ All nodes started with proper timing

#### 3. Configurable Mode (Legacy)

**Recommended for:** Custom configuration via command-line arguments

```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
ros2 launch vision_navigation vision_navigation.launch.py
```

Override visualization parameters:

```bash
# Headless via parameters
ros2 launch vision_navigation vision_navigation.launch.py \
  camera_preview:=false lane_visualization:=false

# GUI via parameters  
ros2 launch vision_navigation vision_navigation.launch.py \
  camera_preview:=true lane_visualization:=true
```

Launch all three nodes simultaneously:

```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
ros2 launch vision_navigation vision_navigation.launch.py
```

Or manually in separate terminals:

**Terminal 1 - Camera Stream:**
```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
camera_stream --ros-args -p open_cam:=True
```

**Terminal 2 - Lane Detection:**
```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
lane_detection --ros-args -p show_window:=True
```

**Terminal 3 - Steering Control:**
```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
steering_control
```

**Terminal 4 - Monitor Steering Output (Optional):**
```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
ros2 topic echo /tpc_rover_fmctl
```

## Node Details

### Camera Stream Node (camera_stream)

**Purpose**: Stream camera data to ROS2 topics

**Published Topics**:
- `/tpc_rover_d415_rgb` (sensor_msgs/Image, bgr8): RGB camera frames
- `/tpc_rover_d415_depth` (sensor_msgs/Image, 16UC1): Depth frames (optional)

**Parameters**:
- `width` (int, default: 1280): Frame width in pixels
- `height` (int, default: 720): Frame height in pixels
- `fps` (int, default: 30): Frames per second
- `open_cam` (bool, default: False): Display camera preview window
- `enable_depth` (bool, default: False): Stream depth data (D415 only)
- `video_path` (str, default: ""): Path to video file (if set, uses video instead of camera)
- `loop_video` (bool, default: True): Loop video when finished
- `json_config` (str, default: ""): Path to RealSense advanced mode JSON configuration

### Lane Detection Node (lane_detection)

**Purpose**: Detect lane markers and compute navigation parameters

**Subscribed Topics**:
- `/tpc_rover_d415_rgb` (sensor_msgs/Image): RGB camera stream

**Published Topics**:
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): [theta, b, detected]
  - `theta`: Heading error from lane center (degrees)
  - `b`: Lateral offset from lane center (pixels)
  - `detected`: Detection flag (1.0 = valid, 0.0 = not detected)

**Parameters**:
- `show_window` (bool, default: False): Display lane detection visualization

**Processing Pipeline**:
1. RGB to LAB color space conversion
2. Color-based lane filtering
3. Sobel gradient edge detection
4. Binary image creation
5. Perspective transform (bird's-eye view)
6. Polyfit lane boundary detection
7. Theta and b parameter calculation

**CSV Logging**: `lane_pub_log.csv` (timestamp, theta, b, detected)

### Steering Control Node (steering_control)

**Purpose**: Closed-loop steering control for lane following

**Subscribed Topics**:
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): Lane parameters [theta, b, detected]

**Published Topics**:
- `/tpc_rover_fmctl` (std_msgs/Float32MultiArray): [steer_angle, detected]
  - `steer_angle`: Steering command in degrees (+right, -left)
  - `detected`: Lane detection status flag

**Parameters**:
- `k_e1` (float, default: 1.0): Weight on heading error (theta)
- `k_e2` (float, default: 0.1): Weight on lateral offset (b)
- `k_p` (float, default: 4.0): Proportional gain
- `k_i` (float, default: 0.0): Integral gain
- `k_d` (float, default: 0.0): Derivative gain
- `ema_alpha` (float, default: 0.05): Exponential moving average smoothing factor
- `steer_max_deg` (float, default: 60.0): Maximum steering angle saturation (±degrees)
- `steer_when_lost` (float, default: 0.0): Steering command when lane not detected

**Control Algorithm**:
```
Combined Error: e = k_e1 * theta_ema + k_e2 * b_ema
PID Output: u = k_p * e + k_i * integral(e, dt) + k_d * de/dt
Steering: steer = clamp(u, -steer_max_deg, steer_max_deg)
If lane not detected: steer = steer_when_lost
```

**CSV Logging**: `logs/rover_ctl_log_ver_3.csv` (time_sec, theta_ema, b_ema, u, e_sum)

## Configuration and Tuning

### Lane Detection Tuning

Edit color thresholds in `vision_navigation_pkg/lane_detector.py`:
- `green_mask` thresholds: Filter out green grass
- `red_mask` thresholds: Detect red lane markers
- Sobel gradient thresholds: Edge detection sensitivity
- Magnitude and direction filtering: Edge feature extraction
- Contour area threshold: Noise removal

### Steering Control Tuning

**Smooth lane following (gentle turns)**:
```bash
steering_control --ros-args \
  -p k_e1:=0.8 -p k_e2:=0.05 -p k_p:=3.0 -p steer_max_deg:=45
```

**Aggressive tracking (sharp turns)**:
```bash
steering_control --ros-args \
  -p k_e1:=1.5 -p k_e2:=0.2 -p k_p:=5.0 -p steer_max_deg:=60
```

**With integral control (steady-state correction)**:
```bash
steering_control --ros-args \
  -p k_p:=4.0 -p k_i:=0.1 -p k_d:=0.02
```

## Data Logging

All nodes automatically log CSV data to the `logs/` directory.

### Lane Detection Log

**File**: `lane_pub_log.csv`

```
timestamp,theta,b,detected
2025-11-04T10:30:45.123456,5.23,45.67,1.0
2025-11-04T10:30:45.153456,-3.21,42.11,1.0
```

### Control Loop Log

**File**: `logs/rover_ctl_log_ver_3.csv`

```
time_sec,theta_ema,b_ema,u,e_sum
1730708000.123,4.85,44.25,19.4,5.12
1730708000.153,-2.95,41.80,-11.8,-3.42
```

## Troubleshooting

### Camera Not Detected

**Problem**: "Cannot find RealSense device"

**Solutions**:
1. Check USB connection
2. Verify device: `rs-enumerate-devices`
3. Install librealsense: `sudo apt install librealsense2`
4. Check device serial number in code (current: 806312060441)

### Lane Detection Not Working

**Problem**: Lane not detected or incorrect detection

**Solutions**:
1. Enable visualization: `-p show_window:=True`
2. Check lighting conditions (outdoor, shadows, etc.)
3. Verify lane markers are visible and contrasting
4. Adjust color thresholds in `lane_detector.py`

### Steering Commands Not Received

**Problem**: `/tpc_rover_fmctl` topic empty

**Solutions**:
1. Verify all three nodes running: `ros2 node list`
2. Check topic connectivity: `ros2 topic info /tpc_rover_nav_lane`
3. Monitor lane detection: `ros2 topic echo /tpc_rover_nav_lane`
4. Check control node parameters loaded

### High CPU Usage

**Problem**: Frame rate too high causing CPU overload

**Solutions**:
1. Reduce frame rate: `-p fps:=15`
2. Reduce resolution: `-p width:=640 -p height:=480`
3. Disable visualization: `-p show_window:=False`

### Steering Oscillation

**Problem**: Oscillating or unstable steering

**Solutions**:
1. Increase EMA smoothing: `-p ema_alpha:=0.02`
2. Reduce proportional gain: `-p k_p:=2.0`
3. Add derivative control: `-p k_d:=0.05`

## Performance Specifications

| Metric | Value |
|--------|-------|
| Max camera FPS | 30 |
| Typical end-to-end latency | 100-150 ms |
| EMA filter warmup time | ~1.5 seconds (default alpha=0.05) |
| Memory per node | 150-200 MB |
| Processing resolution | 1280x720 |

## Sign Conventions

### Steering Angle
- **Positive**: Turn RIGHT
- **Negative**: Turn LEFT
- **Units**: Degrees

### Heading Error (theta)
- **Positive**: Lane center to the RIGHT (need right turn)
- **Negative**: Lane center to the LEFT (need left turn)
- **Units**: Degrees

### Lateral Offset (b)
- **Positive**: Camera displaced RIGHT from lane center
- **Negative**: Camera displaced LEFT from lane center
- **Units**: Pixels

## Package Structure

```
ws_jetson/
├── README.md                          (This file)
├── build_clean.sh                     (Clean build script)
├── build_inc.sh                       (Incremental build script)
├── .gitignore
├── vision_navigation/
│   ├── package.xml                    (ROS2 package metadata, v1.0.0)
│   ├── setup.py                       (Python setup configuration)
│   ├── setup.cfg
│   ├── launch/
│   │   ├── vision_nav_headless.launch.py  (Production/SSH launch - no GUI)
│   │   ├── vision_nav_gui.launch.py       (Debug/test launch - with GUI)
│   │   └── vision_navigation.launch.py    (Configurable launch - legacy)
│   └── vision_navigation_pkg/
│       ├── __init__.py
│       ├── camera_stream_node.py      (RGB/depth camera streaming)
│       ├── lane_detection_node.py     (Lane marker detection)
│       ├── steering_control_node.py   (PID-based steering)
│       ├── lane_detector.py           (Lane detection pipeline)
│       ├── control_filters.py         (Filters and utilities)
│       └── demo_lane.py               (Demonstration)
├── build/                             (Build artifacts, auto-generated)
├── install/                           (Install directory, auto-generated)
├── log/                               (ROS2 logs, auto-generated)
└── logs/                              (Data logs, CSV files)
```

## Launch Files Quick Reference

| Launch File | Mode | GUI | Use Case |
|------------|------|-----|----------|
| `vision_nav_headless.launch.py` | Production | ❌ No | SSH/Remote operation |
| `vision_nav_gui.launch.py` | Debug | ✅ Yes | Local debugging with monitor |
| `vision_navigation.launch.py` | Configurable | ⚙️ Params | Custom configuration |

## Building from Source

### Setup Workspace

```bash
mkdir -p ~/almondmatcha/ws_jetson/src
cd ~/almondmatcha/ws_jetson
```

### Verify Package Structure

```bash
cd ~/almondmatcha/ws_jetson
find . -name "package.xml" -type f
```

### Build

```bash
cd ~/almondmatcha/ws_jetson
colcon build --packages-select vision_navigation
source install/setup.bash
```

### Verify Installation

```bash
which camera_stream lane_detection steering_control
ros2 node list  # (while nodes are running)
```

## Future Enhancements

- ROS2 launch file for complete system startup (COMPLETED - see launch/)
- Unit tests for lane detection pipeline
- Performance profiling and optimization
- Support for additional camera types (Raspberry Pi Camera v2)
- Real-time parameter tuning via ROS2 parameter service
- Machine learning-based lane detection (neural network)
- Multi-lane tracking and path planning
- Sensor fusion (IMU, encoders, lidar)

## License

Apache 2.0

## Support

For issues or questions, refer to the vision_navigation_pkg documentation or the main repository documentation.

## Version History

- **v1.0.0** (November 4, 2025)
  - Complete refactoring with modular architecture
  - Professional documentation
  - Type hints throughout
  - Reusable control filters module
  - Comprehensive README
  - ROS2 launch file support

## Related Documentation

- Main project: `/home/yupi/almondmatcha/README.md`
- Rover workspace: `/home/yupi/almondmatcha/ws_rpi/README.md`
- Session notes: `/home/yupi/almondmatcha/copilot-session-note/WORK_SESSION_2025-11-04.md`
