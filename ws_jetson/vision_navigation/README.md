# Vision Navigation System# Visual Navigation Package



Real-time visual navigation system for autonomous rover lane detection and steering control.Real-time visual navigation system for autonomous rover lane detection and steering control.



## Overview## Overview



The Vision Navigation package provides camera streaming, lane detection, and closed-loop steering control for autonomous mobile rovers. It processes Intel RealSense D415 camera data or video files to detect lane boundaries and commands steering based on lane position feedback.The Visual Navigation package provides camera streaming, lane detection, and closed-loop steering control for autonomous mobile rovers. It processes Intel RealSense D415 camera data or video files to detect lane boundaries and commands steering based on lane position feedback.



## Code Organization## System Architecture



### Directory Structure### Data Flow



``````

vision_navigation/Camera (D415/Video)

├── README.md                      # This file    |

├── LICENSE                        # Apache 2.0 license    v

├── package.xml                    # ROS2 package metadata[Camera Stream Node] -> /tpc_rover_d415_rgb

├── setup.py                       # Python setup configuration    |

├── setup.cfg                      # Python setup config    v

├── resource/                      # Package resources (auto-generated)[Nav Process Node] -> Lane Detection -> /tpc_rover_nav_lane

├── test/                          # Unit tests directory    |

│    v

├── Core Nodes:[Rover Control Node] -> PID Steering -> /tpc_rover_fmctl

├── camera_stream_node.py          # RGB/depth camera streaming (Intel D415 or video)    |

├── lane_detection_node.py         # Lane marker detection and parameter calculation    v

├── steering_control_node.py       # PID-based steering controlSteering Actuator (Front Module)

│```

├── Utility Modules:

├── lane_detector.py               # Lane detection pipeline (image processing)### Nodes

├── control_filters.py             # Filters and control utilities

│#### Camera Stream Node (node_cam_stream)

└── Demonstration:

    └── demo_lane.py               # Lane detection demonstration**Purpose:** Stream camera data to ROS2 topics

```

**Responsibilities:**

### Module Purposes- Initialize Intel RealSense D415 camera OR video file playback

- Stream RGB frames at configurable frame rate

#### Core Nodes- Optional depth streaming (D415 only)

- Frame resizing and synchronization

**camera_stream_node.py**

- Streams RGB and depth frames from Intel RealSense D415 camera**Published Topics:**

- Supports video file playback for testing- `/tpc_rover_d415_rgb` (sensor_msgs/Image, bgr8): RGB camera frames

- Configurable resolution, frame rate, and advanced settings- `/tpc_rover_d415_depth` (sensor_msgs/Image, 16UC1): Depth frames (optional)

- Publishes to `/tpc_rover_d415_rgb` and `/tpc_rover_d415_depth` topics

**Parameters:**

**lane_detection_node.py**- `width` (int, 1280): Frame width in pixels

- Processes RGB frames to detect lane markers- `height` (int, 720): Frame height in pixels

- Uses color filtering, edge detection, and perspective transform- `fps` (int, 30): Frames per second

- Computes steering angle (theta) and lateral offset (b) parameters- `json_config` (str, ""): Path to RealSense advanced mode configuration

- Publishes detection results to `/tpc_rover_nav_lane` topic- `open_cam` (bool, False): Display camera preview in OpenCV window

- Logs results to `lane_pub_log.csv`- `enable_depth` (bool, False): Stream depth data (D415 only)

- `video_path` (str, ""): Path to video file (if set, uses video instead of camera)

**steering_control_node.py**- `loop_video` (bool, True): Loop video when finished

- Implements closed-loop PID steering controller

- Filters noisy lane detection with exponential moving average**Modes:**

- Computes steering command from lane parameters1. **D415 Camera Mode:** Streams directly from Intel RealSense D415 USB camera

- Publishes steering angle to `/tpc_rover_fmctl` topic2. **Video Playback Mode:** Streams from MP4/AVI video file for testing

- Logs control loop data to `logs/rover_ctl_log_ver_3.csv`

#### Navigation Process Node (node_nav_process)

#### Utility Modules

**Purpose:** Detect lane markers and compute navigation parameters

**lane_detector.py**

- Lane detection image processing pipeline**Responsibilities:**

- Functions:- Subscribe to RGB camera frames

  - `preprocess_frame()`: Color filtering and edge detection (LAB color space)- Apply color filtering, edge detection, and perspective transform

  - `perspective_transform()`: Bird's-eye view transformation- Detect lane boundaries using polyfit

  - `process_frame()`: Complete lane detection pipeline- Calculate steering angle (theta) and lateral offset (b)

- Handles color-based filtering, gradient detection, and contour processing- Publish lane parameters and detection status

- Log results to CSV

**control_filters.py**

- Reusable filter and control components**Subscribed Topics:**

- Classes:- `/tpc_rover_d415_rgb` (sensor_msgs/Image): RGB camera stream

  - `MovingAverageLPF`: Simple moving average filter

  - `ExponentialMovingAverageLPF`: EMA filter with warmup detection**Published Topics:**

- Functions:- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): [theta, b, detected]

  - `clamp()`: Value saturation utility  - theta: Heading error from lane center (degrees)

  - `pid_controller()`: Standalone PID calculation  - b: Lateral offset from lane center (pixels)

  - detected: Detection flag (1.0 = valid, 0.0 = not detected)

**demo_lane.py**

- Demonstration script for lane detection testing**Parameters:**

- Supports video file input and visualization- `show_window` (bool, False): Display lane detection visualization



## System Architecture**Processing Pipeline:**

1. RGB to LAB color space conversion

### Data Flow2. Color-based lane filtering (remove non-lane colors)

3. Sobel gradient edge detection

```4. Magnitude and direction filtering

Camera Input (D415 or Video)5. White pixel detection

    |6. Combined binary image creation

    v7. Contour area filtering

[camera_stream_node]8. Perspective transform (bird's-eye view)

    |9. Polyfit lane boundary detection

    +---> /tpc_rover_d415_rgb (RGB frames)10. Theta and b parameter calculation

    +---> /tpc_rover_d415_depth (Depth frames, optional)

    |**CSV Logging:** `lane_pub_log.csv`

    v- timestamp, theta, b, detected

[lane_detection_node]

    |#### Rover Control Node (node_rover_ctl)

    +---> /tpc_rover_nav_lane (Lane parameters)

    +---> lane_pub_log.csv (Detection logging)**Purpose:** Closed-loop steering control for lane following

    |

    v**Responsibilities:**

[steering_control_node]- Subscribe to lane detection parameters

    |- Apply exponential moving average filtering

    +---> /tpc_rover_fmctl (Steering command)- Compute combined error from heading and lateral offset

    +---> logs/rover_ctl_log_ver_3.csv (Control logging)- Execute PID control algorithm

    |- Publish steering command

    v- Log control loop data

Steering Actuator (Front Module)

```**Subscribed Topics:**

- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): Lane parameters [theta, b, detected]

### Topic Interface

**Published Topics:**

| Topic | Message Type | Direction | Description |- `/tpc_rover_fmctl` (std_msgs/Float32MultiArray): [steer_angle, detected]

|-------|--------------|-----------|-------------|  - steer_angle: Steering command in degrees (+right, -left)

| `/tpc_rover_d415_rgb` | sensor_msgs/Image (bgr8) | Out | RGB camera frames |  - detected: Lane detection status flag

| `/tpc_rover_d415_depth` | sensor_msgs/Image (16UC1) | Out | Depth frames (optional) |

| `/tpc_rover_nav_lane` | std_msgs/Float32MultiArray | Out | Lane params [theta, b, detected] |**Parameters:**

| `/tpc_rover_fmctl` | std_msgs/Float32MultiArray | Out | Steering command [angle, detected] |- `k_e1` (float, 1.0): Weight on heading error (theta)

- `k_e2` (float, 0.1): Weight on lateral offset (b)

### Processing Pipeline- `k_p` (float, 4.0): Proportional gain

- `k_i` (float, 0.0): Integral gain

#### Lane Detection- `k_d` (float, 0.0): Derivative gain

1. RGB frame input- `ema_alpha` (float, 0.05): Exponential moving average smoothing factor

2. LAB color space conversion- `steer_max_deg` (float, 60.0): Maximum steering angle saturation (±degrees)

3. Color-based filtering (remove grass, detect lanes)- `steer_when_lost` (float, 0.0): Steering command when lane not detected

4. Sobel gradient edge detection

5. Magnitude and direction filtering**Control Algorithm:**

6. White pixel detection

7. Binary image combination```

8. Contour noise removalCombined Error: e = k_e1 * theta_ema + k_e2 * b_ema

9. Perspective transform to bird's-eye viewPID Output: u = k_p * e + k_i * integral(e, dt) + k_d * de/dt

10. Polyfit lane boundary detectionSteering: steer = clamp(u, -steer_max_deg, steer_max_deg)

11. Theta (angle) and b (offset) calculation```



#### Steering ControlIf lane not detected: steer = steer_when_lost (safety default)

1. Lane parameters input [theta, b, detected]

2. Exponential moving average filtering (warmup period)**CSV Logging:** `logs/rover_ctl_log_ver_3.csv`

3. Combined error calculation: e = k_e1*theta + k_e2*b- time_sec, theta_ema, b_ema, u, e_sum

4. PID control calculation

5. Steering saturation (±steer_max_deg)### Utility Modules

6. Safety fallback if lane not detected

7. Publish steering command#### control_filters.py



## Installation and BuildingProvides low-pass filters and control utilities:



### Prerequisites**Classes:**

- ROS 2 (Humble or newer)- `MovingAverageLPF`: Simple moving average filter

- Python 3.10+- `ExponentialMovingAverageLPF`: EMA filter with configurable smoothing

- Dependencies:

  - `python3-opencv`**Functions:**

  - `python3-numpy`- `clamp()`: Saturate value to [min, max] range

  - `pyrealsense2` (for D415 camera)- `pid_controller()`: PID calculation utility

  - `rclpy`, `cv_bridge`, `sensor_msgs`, `std_msgs`

#### lane_detector.py

### Build Steps

Lane detection pipeline functions:

```bash- `preprocess_frame()`: Color filtering and edge detection

# Build the package- `perspective_transform()`: Bird's-eye view transformation

cd /path/to/ws_jetson- `process_frame()`: Complete lane detection pipeline

colcon build --packages-select vision_navigation

## Building

# Source the workspace

source install/setup.bash### Prerequisites

```- ROS 2 (tested on Humble)

- Python 3.10+

### Verify Installation- OpenCV (python3-opencv)

- NumPy (python3-numpy)

```bash- pyrealsense2 (for D415 camera)

# List installed nodes- rclpy, cv_bridge

ros2 pkg executables vision_navigation

### Build Steps

# Output should show:

# vision_navigation camera_stream```bash

# vision_navigation lane_detection# Build the package

# vision_navigation steering_controlcd /path/to/ws_jetson

# vision_navigation demo_lanecolcon build --packages-select pkg_imagproc

```

# Source the workspace

## Running the Systemsource install/setup.bash

```

### Individual Node Execution

### Verification

#### Camera Stream Node

```bash

```bash# Check installed nodes

# Stream from D415 camera with default settingsros2 pkg executables pkg_imagproc

ros2 run vision_navigation camera_stream

# Should output:

# Stream with preview window# pkg_imagproc node_cam_stream

ros2 run vision_navigation camera_stream --ros-args -p open_cam:=True# pkg_imagproc node_nav_process

# pkg_imagproc node_rover_ctl

# Stream with depth enabled# pkg_imagproc node_demo_lane

ros2 run vision_navigation camera_stream --ros-args -p enable_depth:=True```



# Stream from video file## Usage

ros2 run vision_navigation camera_stream --ros-args \

  -p video_path:="/path/to/video.mp4"### Running Individual Nodes

```

#### Camera Stream Node

#### Lane Detection Node

```bash

```bash# Stream from D415 camera

# Process frames without visualizationros2 run pkg_imagproc node_cam_stream

ros2 run vision_navigation lane_detection

# Stream from D415 with preview

# Process with lane detection visualizationros2 run pkg_imagproc node_cam_stream --ros-args -p open_cam:=True

ros2 run vision_navigation lane_detection --ros-args -p show_window:=True

```# Stream from D415 with depth

ros2 run pkg_imagproc node_cam_stream --ros-args -p enable_depth:=True

#### Steering Control Node

# Stream from video file

```bashros2 run pkg_imagproc node_cam_stream --ros-args \

# Run with default control parameters  -p video_path:="/path/to/video.mp4" -p loop_video:=True

ros2 run vision_navigation steering_control```



# Run with custom PID gains#### Navigation Process Node

ros2 run vision_navigation steering_control --ros-args \

  -p k_p:=5.0 -p k_i:=0.1 -p k_d:=0.05```bash

```# Process frames with visualization disabled

ros2 run pkg_imagproc node_nav_process

### Complete System Startup

# Process with lane detection visualization

Terminal 1 - Camera streaming:ros2 run pkg_imagproc node_nav_process --ros-args -p show_window:=True

```bash```

ros2 run vision_navigation camera_stream --ros-args -p open_cam:=True

```#### Rover Control Node



Terminal 2 - Lane detection:```bash

```bash# Run with default parameters

ros2 run vision_navigation lane_detection --ros-args -p show_window:=Trueros2 run pkg_imagproc node_rover_ctl

```

# Run with custom control gains

Terminal 3 - Steering control:ros2 run pkg_imagproc node_rover_ctl --ros-args \

```bash  -p k_p:=5.0 -p k_i:=0.1 -p k_d:=0.05 \

ros2 run vision_navigation steering_control  -p steer_max_deg:=45.0

``````



Terminal 4 - Monitor output (optional):### Running Complete System

```bash

ros2 topic echo /tpc_rover_fmctl```bash

```# Terminal 1: Camera stream

ros2 run pkg_imagproc node_cam_stream --ros-args -p open_cam:=True

## Configuration and Tuning

# Terminal 2: Lane detection

### Camera Stream Parametersros2 run pkg_imagproc node_nav_process --ros-args -p show_window:=True



| Parameter | Default | Description |# Terminal 3: Rover control

|-----------|---------|-------------|ros2 run pkg_imagproc node_rover_ctl

| width | 1280 | Frame width in pixels |

| height | 720 | Frame height in pixels |# Terminal 4: Monitor steering output

| fps | 30 | Frames per second |ros2 topic echo /tpc_rover_fmctl

| enable_depth | False | Enable depth streaming (D415 only) |```

| open_cam | False | Display preview window |

| video_path | "" | Video file path (empty = use D415) |### With ROS2 Launch File (if available)

| loop_video | True | Loop video when finished |

| json_config | "" | RealSense advanced mode config JSON |```bash

ros2 launch pkg_imagproc vision_navigation.launch.py

### Lane Detection Parameters```



| Parameter | Default | Description |## Configuration

|-----------|---------|-------------|

| show_window | False | Display lane detection visualization |### Lane Detection Tuning



### Steering Control ParametersEdit parameters in `lane_detector.py`:

- `green_mask` thresholds: Filter out green grass

| Parameter | Default | Description |- `red_mask` thresholds: Detect red lane markers

|-----------|---------|-------------|- Sobel gradient thresholds: Edge detection sensitivity

| k_e1 | 1.0 | Heading error weight |- Magnitude and direction filtering: Edge feature extraction

| k_e2 | 0.1 | Lateral offset weight |- Contour area threshold: Noise removal

| k_p | 4.0 | Proportional gain |

| k_i | 0.0 | Integral gain |### Control Tuning

| k_d | 0.0 | Derivative gain |

| ema_alpha | 0.05 | EMA smoothing factor (0-1) |Example configuration for different scenarios:

| steer_max_deg | 60.0 | Maximum steering angle |

| steer_when_lost | 0.0 | Steering when lane lost |**Gentle lane following (smooth turns):**

```bash

### Control Tuning Examples-p k_e1:=0.8 -p k_e2:=0.05 -p k_p:=3.0

```

**Smooth lane following (gentle turns):**

```bash**Aggressive tracking (sharp turns):**

ros2 run vision_navigation steering_control --ros-args \```bash

  -p k_e1:=0.8 -p k_e2:=0.05 -p k_p:=3.0 -p steer_max_deg:=45-p k_e1:=1.5 -p k_e2:=0.2 -p k_p:=5.0

``````



**Aggressive tracking (sharp turns):****With integral control (steady-state correction):**

```bash```bash

ros2 run vision_navigation steering_control --ros-args \-p k_p:=4.0 -p k_i:=0.1 -p k_d:=0.02

  -p k_e1:=1.5 -p k_e2:=0.2 -p k_p:=5.0 -p steer_max_deg:=60```

```

## Troubleshooting

**With integral/derivative control:**

```bash### D415 Camera Not Detected

ros2 run vision_navigation steering_control --ros-args \

  -p k_p:=4.0 -p k_i:=0.1 -p k_d:=0.05**Problem:** "Cannot find RealSense device"

```

**Solutions:**

## Data Logging1. Check USB connection

2. Verify device is recognized: `rs-enumerate-devices`

### Lane Detection Log (lane_pub_log.csv)3. Check device serial number matches in code (current: 806312060441)

```4. Install librealsense: `sudo apt install librealsense2`

timestamp,theta,b,detected

2025-11-04T10:30:45.123456,5.23,45.67,1.0### Lane Detection Not Working

2025-11-04T10:30:45.153456,-3.21,42.11,1.0

```**Problem:** Lane not detected or incorrect detection



### Control Loop Log (logs/rover_ctl_log_ver_3.csv)**Solutions:**

```1. Check camera focus and exposure (adjust with JSON config)

time_sec,theta_ema,b_ema,u,e_sum2. Ensure lane markers are visible and contrasting

1730708000.123,4.85,44.25,19.4,5.123. Verify lighting conditions (outdoor, shadows, etc.)

1730708000.153,-2.95,41.80,-11.8,-3.424. Enable visualization: `-p show_window:=True`

```5. Adjust color thresholds in lane_detector.py



## Troubleshooting### Steering Commands Not Received



### Camera Issues**Problem:** `/tpc_rover_fmctl` topic empty



**Problem: "Cannot find RealSense device"****Solutions:**

- Check USB connection1. Verify all three nodes are running: `ros2 node list`

- Verify device: `rs-enumerate-devices`2. Check topic connectivity: `ros2 topic info /tpc_rover_nav_lane`

- Install librealsense: `sudo apt install librealsense2`3. Monitor lane detection: `ros2 topic echo /tpc_rover_nav_lane`

- Check device serial matches code (current: 806312060441)4. Check control node parameters loaded correctly



**Problem: "pyrealsense2 not found"**### High CPU Usage

- Install: `pip install pyrealsense2`

- Or use video mode: `-p video_path:="/path/to/video.mp4"`**Problem:** Frame rate too high causing CPU overload



### Lane Detection Issues**Solutions:**

1. Reduce frame rate: `-p fps:=15`

**Problem: Lane not detected or incorrect detection**2. Reduce image resolution: `-p width:=640 -p height:=480`

- Enable visualization: `-p show_window:=True`3. Disable visualization: `-p show_window:=False`

- Check lighting conditions4. Use video file mode instead of live camera

- Verify lane markers are visible and contrasting

- Adjust color thresholds in `lane_detector.py`## Performance



**Problem: High false detections**### Frame Rate

- Reduce EMA alpha: `-p ema_alpha:=0.02`- Camera stream: Configurable (default 30 FPS)

- Enable buffer warmup by checking is_full()- Lane detection: Real-time (typically 25-30 FPS @ 1280x720)

- Adjust gradient thresholds in lane_detector.py- Control loop: Depends on lane detection rate



### Steering Control Issues### Latency

- Camera to steering command: ~100-150 ms (typical)

**Problem: "/tpc_rover_fmctl" topic not published**- EMA filter warm-up: ~1.5 seconds (with alpha=0.05, maxlen=30)

- Verify all nodes running: `ros2 node list`

- Check lane detection: `ros2 topic echo /tpc_rover_nav_lane`### Memory

- Monitor control node: `ros2 topic echo /tpc_rover_fmctl`- Typical process memory: 150-200 MB per node



**Problem: Oscillating or unstable steering**## Data Output

- Increase EMA smoothing: `-p ema_alpha:=0.02`

- Reduce proportional gain: `-p k_p:=2.0`### CSV Logs

- Add derivative control: `-p k_d:=0.05`

**lane_pub_log.csv:**

### Performance Issues```

timestamp,theta,b,detected

**Problem: High CPU usage**2025-11-04T10:30:45.123456,5.23,45.67,1.0

- Reduce frame rate: `-p fps:=15`2025-11-04T10:30:45.153456,-3.21,42.11,1.0

- Reduce resolution: `-p width:=640 -p height:=480````

- Disable preview: `-p open_cam:=False`

- Disable visualization: `-p show_window:=False`**logs/rover_ctl_log_ver_3.csv:**

```

**Problem: High latency (steering lag)**time_sec,theta_ema,b_ema,u,e_sum

- Reduce EMA window: Edit `maxlen` in `ExponentialMovingAverageLPF`1730708000.123,4.85,44.25,19.4,5.12

- Reduce frame processing time with lower resolution1730708000.153,-2.95,41.80,-11.8,-3.42

- Ensure sufficient CPU resources available```



## Sign Conventions## Sign Conventions



### Steering Angle### Steering Angle

- **Positive:** Turn RIGHT- **Positive:** Turn RIGHT

- **Negative:** Turn LEFT- **Negative:** Turn LEFT

- **Units:** Degrees- **Units:** Degrees



### Heading Error (theta)### Heading Error (theta)

- **Positive:** Lane center RIGHT of camera (need right turn)- **Positive:** Lane center is to the RIGHT (need to turn right)

- **Negative:** Lane center LEFT of camera (need left turn)- **Negative:** Lane center is to the LEFT (need to turn left)

- **Units:** Degrees- **Units:** Degrees



### Lateral Offset (b)### Lateral Offset (b)

- **Positive:** Camera RIGHT of lane center- **Positive:** Camera is displaced to the RIGHT from lane center

- **Negative:** Camera LEFT of lane center- **Negative:** Camera is displaced to the LEFT from lane center

- **Units:** Pixels- **Units:** Pixels



## Performance Specifications## Dependencies



| Metric | Value |### Runtime

|--------|-------|- rclpy (ROS2 Python client)

| Max camera FPS | 30 |- cv_bridge (ROS2 OpenCV bridge)

| Typical latency (end-to-end) | 100-150 ms |- sensor_msgs (ROS2 sensor message types)

| EMA filter warmup time | 1.5 seconds (default alpha=0.05) |- std_msgs (ROS2 standard message types)

| Memory per node | 150-200 MB |- opencv-python / python3-opencv

| Processing resolution | 1280x720 |- numpy / python3-numpy

- pyrealsense2 (Intel RealSense SDK)

## Dependencies

### Development

### Runtime- ament_python (ROS2 build system)

- rclpy (ROS2 Python client library)- ament_copyright, ament_flake8, ament_pep257 (code quality)

- cv_bridge (ROS2 OpenCV bridge)

- sensor_msgs (ROS2 sensor messages)## File Structure

- std_msgs (ROS2 standard messages)

- opencv-python / python3-opencv```

- numpy / python3-numpypkg_imagproc/

- pyrealsense2 (Intel RealSense SDK)├── package.xml              # Package metadata (v1.0.0)

├── setup.py                 # Python setup configuration

### Build├── setup.cfg                # Python setup config

- ament_python (ROS2 build system)├── pkg_imagproc/

- ament_copyright, ament_flake8, ament_pep257 (code quality tools)│   ├── __init__.py

│   ├── node_cam1_d415_stream.py    # Camera streaming node

## Future Enhancements│   ├── node_cam1_nav_process.py    # Lane detection node

│   ├── node_rover_ctl.py           # Steering control node

- ROS2 launch file for complete system startup│   ├── lane_detector.py            # Lane detection pipeline

- Unit tests and integration tests│   ├── control_filters.py          # Filters and utilities

- Performance profiling and optimization│   ├── demo_lane.py               # Demo/testing node

- Support for additional camera models│   ├── command.txt

- Real-time parameter tuning service│   └── preprocess_img/

- Machine learning-based lane detection├── launch/                  # ROS2 launch files

- Multi-lane tracking and path planning├── resource/               # Package resources

- Sensor fusion (IMU, encoders, lidar)└── test/                   # Unit tests

- Web dashboard for monitoring```



## Improvements in v1.0.0## Improvements in v1.0.0



### Code Organization### Code Organization

- Simplified directory structure (removed nested pkg_imagproc)- Separated filter classes into dedicated module (control_filters.py)

- Clearer node names for easy identification- Added comprehensive type hints throughout

- Removed unused temporary files and directories- Better method organization with clear sections (initialization, callbacks, utilities)

- Organized utility modules for reusability- Professional docstrings for all classes and methods



### Naming Convention### Node Refactoring

- Package: `vision_navigation` (descriptive, semantic)- node_cam1_d415_stream.py: Better error handling, cleaner initialization, type hints

- Nodes:- node_cam1_nav_process.py: Improved frame validation, better logging, visualization helper

  - `camera_stream` (instead of node_cam1_d415_stream)- node_rover_ctl.py: Clearer PID implementation, buffer warmup logic, better status output

  - `lane_detection` (instead of node_cam1_nav_process)

  - `steering_control` (instead of node_rover_ctl)### Documentation

- Entry points reflect clear functionality- Professional README with complete architecture overview

- Parameter documentation and tuning guide

### Code Quality- Troubleshooting section with common issues

- Type hints throughout- CSV log format documentation

- Comprehensive docstrings- Sign convention documentation

- Professional documentation

- Consistent error handling### Package Metadata

- Resource cleanup and lifecycle management- Updated version to 1.0.0

- Improved package description

### Maintainability- Corrected entry point naming

- Logical file organization- Consistent Python 3 usage

- Extracted reusable components

- Better separation of concerns## Future Enhancements

- Clear code sections and comments

- ROS2 launch file for complete system startup

## License- Unit tests for lane detection pipeline

- Performance profiling and optimization

Apache 2.0- Support for additional camera types (Raspberry Pi Camera v2)

- Real-time parameter tuning via ROS2 parameter service

## Support- Machine learning-based lane detection (neural network)

- Multi-lane tracking and path planning

For issues or documentation updates, refer to WORK_SESSION files in workspace root.- Sensor fusion (IMU, encoders, lidar)


## License

Apache 2.0

## Support

For issues or questions, refer to workspace documentation in WORK_SESSION files.
