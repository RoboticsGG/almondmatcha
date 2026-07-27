# Vision Navigation System

Real-time visual navigation system for autonomous rover lane detection and steering control. This system processes Intel RealSense D415 camera data or video files to detect lane boundaries and compute steering commands via closed-loop PID control.

## System Overview

The Vision Navigation system consists of three coordinated ROS2 nodes:

1. **Camera Stream Node** - Acquires and publishes RGB/depth frames
2. **Lane Detection Node** - Detects lane markers and computes steering parameters
3. **Rover Kinematic Control Node** - Implements PID-based steering + speed control

Initialization Sequence: Camera (0s) → Lane Detection (2s) → Steering Control (3s)

## Architecture

### Data Flow

```
Camera (D415/Video) → Camera Stream Node → /tpc_rover_d415_rgb
                                           ↓
                     Lane Detection Node ← Frame Input
                                           ↓
                                    /tpc_rover_nav_lane [curvature, theta, b, detected]
                                           ↓
                     Steering Control Node ← Lane Parameters
                                           ↓
                                    /tpc_rover_ctrl_cmd [steer_angle, speed_cmd, detected]
                                           ↓
                                    Chassis Controller (RPi)
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

Detects lane markers and computes steering parameters (curvature, theta, b, detected flag).

Subscribed Topics:
- `/tpc_rover_d415_rgb` (sensor_msgs/Image): RGB camera stream

Published Topics:
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): [curvature, theta, b, detected]
  - curvature: Parabola coefficient A (x = A*y^2 + B*y + C) in the rover-centered warped frame; positive = curves right ahead, negative = curves left ahead
  - theta: Heading error from lane center (degrees, positive = right)
  - b: Lateral offset from lane center (pixels, positive = right)
  - detected: Detection flag (1.0 = valid, 0.0 = not detected)

Parameters:
- `show_window` (bool, default False): Display lane detection visualization
- `roi_base_points` (double array, 8 values): ROI trapezoid corners [x0,y0,...,x3,y3] at (roi_base_width x roi_base_height)
- `roi_base_width` / `roi_base_height` (float, default 1280.0 / 720.0): reference resolution roi_base_points were authored against
- `crop_margin_px` (float, default 20.0): margin around the ROI bounding box before cropping (in roi_base_width/height units, scaled to actual frame size)
- `sliding_windows` (int, default 9): number of vertical search windows
- `window_margin` (int, default 100): search window half-width in pixels
- `min_window_pixels` (int, default 50): minimum pixels to recenter a search window
- `min_lane_pixels` (int, default 50): minimum total detected pixels required for a valid fit

CSV Output: columns [timestamp, curvature, theta, b, detected]. Written on a background thread (queue-based) so file I/O never blocks the image callback.

Processing Pipeline:
1. Crop frame to the ROI bounding box (plus margin) -- cuts CPU on the steps below without reducing pixel density inside the ROI
2. LAB color space filtering (green background removal)
3. Sobel gradient edge detection (CV_32F + cv2.cartToPolar for magnitude/direction)
4. White pixel detection
5. Combined binary image creation
6. Morphological opening (vectorized noise-blob removal)
7. Perspective transform (bird's-eye view)
8. 2nd-degree polyfit lane boundary detection (parabola, not a straight line)
9. Curvature, theta, and b parameter calculation (in a rover-centered frame)

#### Rover Kinematic Control Node (rover_kinematic_control)

Implements closed-loop PID steering + speed control based on lane detection parameters.

Subscribed Topics:
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): Lane parameters [curvature, theta, b, detected]

Published Topics:
- `/tpc_rover_ctrl_cmd` (std_msgs/Float32MultiArray): [steer_angle, speed_cmd, detected]
  - steer_angle: Steering command in degrees (positive = right, negative = left)
  - speed_cmd: Speed command (0–100% PWM duty cycle)
  - detected: Lane detection status flag

Parameters:
- `k_e1` (float, default 1.0): Weight on heading error (theta)
- `k_e2` (float, default 0.1): Weight on lateral offset (b)
- `k_p` (float, default 4.0): Proportional gain
- `k_i` (float, default 0.0): Integral gain
- `k_d` (float, default 0.0): Derivative gain
- `k_ff` (float, default 1000.0): Feedforward gain on filtered curvature -- anticipates the curve ahead instead of waiting for heading/offset error to build up
- `speed_ref` (float, default 50.0): Base speed when lane detected (0–100% duty cycle)
- `speed_lost_ratio` (float, default 0.5): Speed multiplier when lane temporarily lost
- `detection_timeout_sec` (float, default 10.0): Seconds before speed drops to 0 on sustained lane loss
- `ema_alpha` (float, default 0.05): Exponential moving average smoothing (0-1), applied to theta, b, and curvature
- `steer_max_deg` (float, default 60.0): Maximum steering angle saturation
- `steer_when_lost` (float, default 0.0): Steering command when lane not detected

Control Algorithm:
```
Combined Error:    e       = k_e1 * theta_ema + k_e2 * b_ema
Feedback (PID):    u_pid   = k_p * e + k_i * integral(e, dt) + k_d * de/dt
Feedforward:       u_ff    = k_ff * curvature_ema
Combined Output:   u_total = u_pid + u_ff
Steering:          steer   = clamp(u_total, -steer_max_deg, steer_max_deg)
If not detected:   steer   = steer_when_lost
```

CSV Output: columns [time_sec, theta_ema, b_ema, curvature_ema, pid_u, e_sum, steer_angle, speed_cmd, detected]

## Configuration and Tuning

### Centralized Configuration (config.py)

All system parameters are centralized in `vision_navigation/config.py`:

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

Curve anticipation (raise k_ff if the rover steers in late on curves; lower it if it steers in before the curve is actually there):
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  k_ff:=1500.0
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
ros2 run vision_navigation camera_stream_node
```

Camera stream with preview:
```bash
ros2 run vision_navigation camera_stream_node --ros-args -p open_cam:=True
```

From video file:
```bash
ros2 run vision_navigation camera_stream_node --ros-args \
  -p video_path:="/path/to/video.mp4"
```

Lane detection with visualization:
```bash
ros2 run vision_navigation lane_detection_node --ros-args -p show_window:=True
```

Steering control:
```bash
ros2 run vision_navigation rover_kinematic_control
```

### Complete System Startup

Terminal 1 - Camera:
```bash
ros2 run vision_navigation camera_stream_node --ros-args -p open_cam:=True
```

Terminal 2 - Lane detection:
```bash
ros2 run vision_navigation lane_detection_node --ros-args -p show_window:=True
```

Terminal 3 - Rover kinematic control:
```bash
ros2 run vision_navigation rover_kinematic_control
```

Terminal 4 - Monitor control output:
```bash
ros2 topic echo /tpc_rover_ctrl_cmd
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
- `get_scaled_roi_points()`: Scale ROI corners from base to actual frame resolution
- `crop_to_roi()`: Crop frame to the ROI bounding box before preprocessing
- `perspective_transform()`: Bird's-eye view transformation
- `find_center_line()`: Sliding window lane tracking
- `compute_lane_params()`: Curvature, theta, and b parameter calculation (2nd-degree fit)
- `process_frame()`: Complete lane detection pipeline (crop -> preprocess -> transform -> detect -> params)
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

### Lane Detection Log (`<ws_jetson>/runs/logs/ws_jetson_lane_detection_TIMESTAMP.csv`)

Columns: timestamp, curvature, theta, b, detected

Example:
```
2025-11-04T10:30:45.123456,0.000412,5.23,45.67,1.0
2025-11-04T10:30:45.153456,-0.000298,-3.21,42.11,1.0
```

### Steering Control Log (`<ws_jetson>/runs/logs/ws_jetson_kinematic_ctrl_TIMESTAMP.csv`)

Columns: time_sec, theta_ema, b_ema, curvature_ema, pid_u, e_sum, steer_angle, speed_cmd, detected

Example:
```
1730708000.123,4.85,44.25,0.000412,19.4,5.12,20.0,50,1
1730708000.153,-2.95,41.80,-0.000298,-11.8,-3.42,-15.3,50,1
```

## Performance Specifications

- Maximum camera FPS: 30
- End-to-end latency: 100-150 ms (typical)
- EMA filter warmup time: 1.5 seconds (default alpha=0.05)
- Memory per node: 150-200 MB
- Camera resolution: 960x540 (configured in vision_nav_headless.yaml / vision_nav_gui.yaml)
- Preprocessing runs on the ROI crop (~960x255 at default roi_base_points/crop_margin_px), not the full frame

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

### Curvature
- Positive: Lane curves RIGHT ahead (and behind -- it's the coefficient A of a symmetric parabola)
- Negative: Lane curves LEFT ahead
- Units: pixels^-1 (coefficient A in x = A*y^2 + B*y + C, rover-centered warped frame)
- Typical magnitude: ~1e-4 (gentle curve) to ~1e-2 (tight curve)

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
├── vision_navigation/
│   ├── __init__.py
│   ├── config.py                       # Centralized configuration
│   ├── helpers.py                      # Utility functions
│   ├── camera_stream_node.py           # Camera streaming node
│   ├── lane_detection_node.py          # Lane detection node
│   ├── rover_kinematic_control_node.py # Kinematic control node
│   ├── lane_detector.py                # Lane detection pipeline
│   └── control_filters.py              # Filters and utilities
├── resource/                           # Package resources
└── test/                               # Unit tests
```

## Version History

### v1.1.0 (July 23, 2026)

Addresses field-tested curve-approach steering lag (rover reacting too late on curves).

- Lane fit upgraded from a straight line (degree-1) to a parabola (degree-2), adding a
  `curvature` output; `tpc_rover_nav_lane` grows from `[theta, b, detected]` to
  `[curvature, theta, b, detected]`
- Rover kinematic control adds a feedforward term (`k_ff * curvature_ema`) combined with
  the existing PID feedback, so steering anticipates the curve instead of only reacting
  to heading/offset error after the fact
- ROI-bounding-box crop added before preprocessing (`get_scaled_roi_points` / `crop_to_roi`),
  cutting CPU without sacrificing pixel density inside the ROI; camera resolution changed
  1280x720 -> 960x540
- Fixed a pre-existing width/height swap bug in `perspective_transform` that produced an
  incorrectly-scaled, transposed warp even at native resolution -- this changes the
  warped-space geometry, so `k_p`/`k_e1`/`k_e2`/`k_ff`/`steer_max_deg` were re-tuned
- `preprocess_frame` optimized: dead code removed (unused blurred-RGB pass, unused
  `red_mask`), Sobel switched to `CV_32F` + `cv2.cartToPolar`, per-contour noise-filter
  loop replaced with vectorized morphological opening
- Lane detection CSV logging moved off the image callback onto a background thread
  (`queue.Queue`), since synchronous file I/O was dropping frames
- ROI points, crop margin, and sliding-window/pixel-threshold constants (previously
  hardcoded and disconnected from `config.py`) moved to ROS parameters sourced from
  `vision_nav_headless.yaml` / `vision_nav_gui.yaml`

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
