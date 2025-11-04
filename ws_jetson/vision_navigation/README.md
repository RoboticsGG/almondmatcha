# Vision Navigation System

Real-time visual navigation system for autonomous rover lane detection and steering control. This system processes Intel RealSense D415 camera data or video files to detect lane boundaries and compute steering commands via closed-loop PID control.

## System Overview

The Vision Navigation system consists of three coordinated ROS2 nodes:

1. **Camera Stream Node** - Acquires and publishes RGB/depth frames
2. **Lane Detection Node** - Detects lane markers and computes steering parameters
3. **Steering Control Node** - Implements PID-based steering control

Initialization Sequence: Camera (0s) → Lane Detection (2s) → Steering Control (3s)

## Architecture

### Data Flow

```
Camera (D415/Video) → Camera Stream Node → /tpc_rover_d415_rgb
                                           ↓
                     Lane Detection Node ← Frame Input
                                           ↓
                                    /tpc_rover_nav_lane [theta, b, detected]
                                           ↓
                     Steering Control Node ← Lane Parameters
                                           ↓
                                    /tpc_rover_fmctl [steer_angle, detected]
                                           ↓
                                    Steering Actuator
```

### Node Descriptions

#### Camera Stream Node (camera_stream)

Streams RGB and optional depth frames from Intel RealSense D415 or video file.

Published Topics:
- `/tpc_rover_d415_rgb` (sensor_msgs/Image, bgr8): RGB frames
- `/tpc_rover_d415_depth` (sensor_msgs/Image, 16UC1): Depth frames (optional)

Parameters:
- `width` (int, default 1280): Frame width in pixels
- `height` (int, default 720): Frame height in pixels
- `fps` (int, default 30): Frames per second
- `open_cam` (bool, default False): Display preview window
- `enable_depth` (bool, default False): Enable depth streaming
- `video_path` (str, default ""): Video file path (empty = use D415 camera)
- `loop_video` (bool, default True): Loop video when finished
- `json_config` (str, default ""): RealSense advanced mode JSON config path

#### Lane Detection Node (lane_detection)

Detects lane markers and computes steering parameters (theta, b, detected flag).

Subscribed Topics:
- `/tpc_rover_d415_rgb` (sensor_msgs/Image): RGB camera stream

Published Topics:
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): [theta, b, detected]
  - theta: Heading error from lane center (degrees, positive = right)
  - b: Lateral offset from lane center (pixels, positive = right)
  - detected: Detection flag (1.0 = valid, 0.0 = not detected)

Parameters:
- `show_window` (bool, default False): Display lane detection visualization

CSV Output: `lane_pub_log.csv` with columns [timestamp, theta, b, detected]

Processing Pipeline:
1. RGB to LAB color space conversion
2. Color-based lane filtering (green grass removal, lane detection)
3. Sobel gradient edge detection
4. Magnitude and direction filtering
5. White pixel detection
6. Combined binary image creation
7. Contour area filtering
8. Perspective transform (bird's-eye view)
9. Polyfit lane boundary detection
10. Theta and b parameter calculation

#### Steering Control Node (steering_control)

Implements closed-loop PID steering control based on lane detection parameters.

Subscribed Topics:
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): Lane parameters [theta, b, detected]

Published Topics:
- `/tpc_rover_fmctl` (std_msgs/Float32MultiArray): [steer_angle, detected]
  - steer_angle: Steering command in degrees (positive = right, negative = left)
  - detected: Lane detection status flag

Parameters:
- `k_e1` (float, default 1.0): Weight on heading error (theta)
- `k_e2` (float, default 0.1): Weight on lateral offset (b)
- `k_p` (float, default 4.0): Proportional gain
- `k_i` (float, default 0.0): Integral gain
- `k_d` (float, default 0.0): Derivative gain
- `ema_alpha` (float, default 0.05): Exponential moving average smoothing (0-1)
- `steer_max_deg` (float, default 60.0): Maximum steering angle saturation
- `steer_when_lost` (float, default 0.0): Steering command when lane not detected

Control Algorithm:
```
Combined Error: e = k_e1 * theta_ema + k_e2 * b_ema
PID Output: u = k_p * e + k_i * integral(e, dt) + k_d * de/dt
Steering: steer = clamp(u, -steer_max_deg, steer_max_deg)
If not detected: steer = steer_when_lost
```

CSV Output: `logs/rover_ctl_log_ver_3.csv` with columns [time_sec, theta_ema, b_ema, u, e_sum]

## Configuration and Tuning

### Centralized Configuration (config.py)

All system parameters are centralized in `vision_navigation_pkg/config.py`:

- CameraConfig: Resolution, FPS, camera modes
- LaneDetectionConfig: Color thresholds, gradient parameters, window settings
- ControlConfig: PID gains, error weights, saturation limits
- LoggingConfig: File paths, CSV headers
- TopicConfig: ROS2 topic names (tpc_* convention)
- SystemConfig: Node names, initialization timing, QoS settings

Launch file automatically reads defaults from config.py, enabling:
- Single source of truth for all parameters
- No manual sync between config and launch file
- Environment variable override support

### Quick Tuning Workflow

1. Test parameters during session (no rebuild):
   ```bash
   ros2 launch vision_navigation vision_navigation.launch.py \
     k_p:=4.5 k_i:=0.1 k_d:=0.15
   ```

2. Save best values to config.py:
   ```python
   class ControlConfig:
       K_P = 4.5    # Updated from testing
       K_I = 0.1
       K_D = 0.15
   ```

3. Rebuild:
   ```bash
   colcon build --packages-select vision_navigation
   ```

4. Next session uses new defaults:
   ```bash
   ros2 launch vision_navigation vision_navigation.launch.py
   ```

### Tuning Examples

Smooth lane following (gentle turns):
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  k_e1:=0.8 k_e2:=0.05 k_p:=3.0 steer_max_deg:=45
```

Aggressive tracking (sharp turns):
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  k_e1:=1.5 k_e2:=0.2 k_p:=5.0 steer_max_deg:=60
```

With integral/derivative control (steady-state correction and damping):
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  k_p:=4.0 k_i:=0.1 k_d:=0.05 ema_alpha:=0.08
```

## Installation and Build

### Prerequisites

- ROS 2 (Humble or newer)
- Python 3.10+
- OpenCV (python3-opencv)
- NumPy (python3-numpy)
- pyrealsense2 (for D415 camera)
- rclpy, cv_bridge, sensor_msgs, std_msgs

### Build Steps

```bash
# Navigate to workspace
cd /path/to/ws_jetson

# Build the package
colcon build --packages-select vision_navigation

# Source the workspace
source install/setup.bash

# Verify installation
ros2 pkg executables vision_navigation
```

## Running the System

### Individual Node Execution

Camera stream from D415:
```bash
ros2 run vision_navigation camera_stream
```

Camera stream with preview:
```bash
ros2 run vision_navigation camera_stream --ros-args -p open_cam:=True
```

From video file:
```bash
ros2 run vision_navigation camera_stream --ros-args \
  -p video_path:="/path/to/video.mp4"
```

Lane detection with visualization:
```bash
ros2 run vision_navigation lane_detection --ros-args -p show_window:=True
```

Steering control:
```bash
ros2 run vision_navigation steering_control
```

### Complete System Startup

Terminal 1 - Camera:
```bash
ros2 run vision_navigation camera_stream --ros-args -p open_cam:=True
```

Terminal 2 - Lane detection:
```bash
ros2 run vision_navigation lane_detection --ros-args -p show_window:=True
```

Terminal 3 - Steering control:
```bash
ros2 run vision_navigation steering_control
```

Terminal 4 - Monitor steering output:
```bash
ros2 topic echo /tpc_rover_fmctl
```

Or use launch file (automated sequencing):
```bash
ros2 launch vision_navigation vision_navigation.launch.py
```

## Helper Modules

### control_filters.py

Provides low-pass filters and control utilities:

Classes:
- `MovingAverageLPF`: Simple moving average filter
- `ExponentialMovingAverageLPF`: EMA filter with configurable smoothing and warmup detection

Functions:
- `clamp()`: Saturate value to [min, max] range
- `pid_controller()`: PID calculation with anti-windup support

### lane_detector.py

Lane detection pipeline functions:
- `preprocess_frame()`: Color filtering and edge detection
- `perspective_transform()`: Bird's-eye view transformation
- `find_center_line()`: Sliding window lane tracking
- `compute_lane_params()`: Theta and b parameter calculation
- `process_frame()`: Complete lane detection pipeline
- `plot_lane_lines()`: Visualization (for debugging)

### helpers.py

Reusable utility functions (50+ total):
- Conversion: degrees_to_radians, radians_to_degrees, normalize_angle
- Validation: is_valid_number, validate_image, validate_roi_points
- Math: clamp, lerp, smooth_step, exponential_moving_average, calculate_distance
- Image: resize_image, crop_image, draw_crosshair, draw_text_box
- Logging: setup_csv_logging, log_csv_row, get_timestamp_string
- Timing: Timer class for performance measurement
- ROS: get_message_timestamp, create_float_array_message

## Data Output

### Lane Detection Log (lane_pub_log.csv)

Columns: timestamp, theta, b, detected

Example:
```
2025-11-04T10:30:45.123456,5.23,45.67,1.0
2025-11-04T10:30:45.153456,-3.21,42.11,1.0
```

### Steering Control Log (logs/rover_ctl_log_ver_3.csv)

Columns: time_sec, theta_ema, b_ema, u, e_sum

Example:
```
1730708000.123,4.85,44.25,19.4,5.12
1730708000.153,-2.95,41.80,-11.8,-3.42
```

## Performance Specifications

- Maximum camera FPS: 30
- End-to-end latency: 100-150 ms (typical)
- EMA filter warmup time: 1.5 seconds (default alpha=0.05)
- Memory per node: 150-200 MB
- Typical processing resolution: 1280x720

## Troubleshooting

### D415 Camera Not Detected

1. Check USB connection
2. Verify device: `rs-enumerate-devices`
3. Install librealsense: `sudo apt install librealsense2`
4. Check device serial number (current: 806312060441)

### Lane Detection Not Working

1. Enable visualization: `-p show_window:=True`
2. Check lighting conditions (outdoor, shadows, etc.)
3. Verify lane markers are visible and contrasting
4. Adjust color thresholds in config.py LaneDetectionConfig
5. Use video file mode for testing: `-p video_path:="/path/to/video.mp4"`

### Steering Commands Not Received

1. Verify all nodes running: `ros2 node list`
2. Check topic connectivity: `ros2 topic info /tpc_rover_nav_lane`
3. Monitor lane detection: `ros2 topic echo /tpc_rover_nav_lane`
4. Check steering control node parameters loaded correctly

### High CPU Usage

1. Reduce frame rate: `-p fps:=15`
2. Reduce resolution: `-p width:=640 -p height:=480`
3. Disable visualization: `-p show_window:=False`
4. Use video file mode instead of live camera

### Oscillating or Unstable Steering

1. Increase EMA smoothing: `-p ema_alpha:=0.02`
2. Reduce proportional gain: `-p k_p:=2.0`
3. Add derivative control: `-p k_d:=0.05`
4. Adjust error weights: reduce `k_e1` or `k_e2`

## Sign Conventions

### Steering Angle
- Positive: Turn RIGHT
- Negative: Turn LEFT
- Units: Degrees
- Range: [-steer_max_deg, +steer_max_deg]

### Heading Error (theta)
- Positive: Lane center is to the RIGHT (need to turn right)
- Negative: Lane center is to the LEFT (need to turn left)
- Units: Degrees

### Lateral Offset (b)
- Positive: Camera is displaced to the RIGHT from lane center
- Negative: Camera is displaced to the LEFT from lane center
- Units: Pixels

## File Structure

```
vision_navigation/
├── README.md                           # This file
├── LICENSE                             # Apache 2.0
├── package.xml                         # ROS2 metadata
├── setup.py                            # Python setup
├── setup.cfg                           # Setup config
├── launch/
│   └── vision_navigation.launch.py     # ROS2 launch file
├── vision_navigation_pkg/
│   ├── __init__.py
│   ├── config.py                       # Centralized configuration
│   ├── helpers.py                      # Utility functions
│   ├── camera_stream_node.py           # Camera streaming node
│   ├── lane_detection_node.py          # Lane detection node
│   ├── steering_control_node.py        # Steering control node
│   ├── lane_detector.py                # Lane detection pipeline
│   └── control_filters.py              # Filters and utilities
├── resource/                           # Package resources
└── test/                               # Unit tests
```

## Version History

### v1.0.0 (November 4, 2025)

Code Quality:
- 100% type hint coverage
- Professional docstrings for all functions and classes
- Comprehensive documentation

Architecture:
- Centralized configuration (config.py) with 6 config classes
- Reusable helper module (helpers.py) with 50+ functions
- Auto-sync launch file that reads from config.py
- Proper node initialization sequencing (2s camera, cascading startup)

Improvements:
- Launch file auto-reads defaults from config.py (no manual sync)
- Environment variable override support
- Enhanced PID controller with anti-windup
- Professional code organization and naming
- Removed all educational/tutorial comments

## Dependencies

### Runtime
- rclpy (ROS2 Python client)
- cv_bridge (ROS2 OpenCV bridge)
- sensor_msgs (ROS2 sensor messages)
- std_msgs (ROS2 standard messages)
- opencv-python / python3-opencv
- numpy / python3-numpy
- pyrealsense2 (Intel RealSense SDK)

### Build
- ament_python (ROS2 build system)

## License

Apache 2.0

## Future Enhancements

- Unit tests for lane detection pipeline
- Real-time parameter tuning via ROS2 parameter service
- Performance profiling and optimization
- Support for additional camera types
- Machine learning-based lane detection
- Multi-lane tracking and path planning
- Sensor fusion (IMU, encoders, lidar)
