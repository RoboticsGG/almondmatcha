# Node Details

## Camera Stream Node (camera_stream)

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

**Run individually**:
```bash
source ~/almondmatcha/ws_jetson/install/setup.bash

# From D415 camera (headless)
camera_stream

# With preview window
camera_stream --ros-args -p open_cam:=True

# From video file
camera_stream --ros-args -p video_path:="/path/to/video.mp4" -p open_cam:=True

# With depth streaming
camera_stream --ros-args -p enable_depth:=True
```

---

## Lane Detection Node (lane_detection)

**Purpose**: Detect lane markers and compute navigation parameters

**Subscribed Topics**:
- `/tpc_rover_d415_rgb` (sensor_msgs/Image): RGB camera stream

**Published Topics**:
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): [curvature, theta, b, detected]

**Parameters**:
- `show_window` (bool, default: False): Display lane detection visualization
- `roi_base_points` (double array, 8 values): ROI trapezoid corners at (roi_base_width x roi_base_height)
- `roi_base_width` / `roi_base_height` (float, default: 1280.0 / 720.0)
- `crop_margin_px` (float, default: 20.0): margin around the ROI bounding box before cropping
- `sliding_windows` (int, default: 9), `window_margin` (int, default: 100)
- `min_window_pixels` (int, default: 50), `min_lane_pixels` (int, default: 50)

**CSV Logging**: columns (timestamp, curvature, theta, b, detected), written on a background thread

**Run individually**:
```bash
source ~/almondmatcha/ws_jetson/install/setup.bash

# Headless mode
lane_detection

# With visualization
lane_detection --ros-args -p show_window:=True
```

---

## Steering Control Node (steering_control)

**Purpose**: Closed-loop steering control for lane following

**Subscribed Topics**:
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): Lane parameters [curvature, theta, b, detected]

**Published Topics**:
- `/tpc_rover_ctrl_cmd` (std_msgs/Float32MultiArray): [steer_angle, speed_cmd, detected]

**Parameters**:
- `k_e1` (float, default: 1.0): Weight on heading error (theta)
- `k_e2` (float, default: 0.1): Weight on lateral offset (b)
- `k_p` (float, default: 4.0): Proportional gain
- `k_i` (float, default: 0.0): Integral gain
- `k_d` (float, default: 0.0): Derivative gain
- `k_ff` (float, default: 1000.0): Feedforward gain on filtered curvature
- `ema_alpha` (float, default: 0.05): Exponential moving average smoothing factor (theta, b, curvature)
- `steer_max_deg` (float, default: 60.0): Maximum steering angle saturation (±degrees)
- `speed_ref` (float, default: 50.0): Base speed command when lane is detected (0–100% duty cycle)
- `speed_lost_ratio` (float, default: 0.5): Speed multiplier when lane is temporarily lost
- `detection_timeout_sec` (float, default: 10.0): Seconds before speed drops to 0 on sustained lane loss

**CSV Logging**: `logs/ws_jetson_kinematic_ctrl_TIMESTAMP.csv` (time_sec, theta_ema, b_ema, curvature_ema, pid_u, e_sum, steer_angle, speed_cmd, detected)

**Run individually**:
```bash
source ~/almondmatcha/ws_jetson/install/setup.bash

# Default parameters
steering_control

# Custom control gains
steering_control --ros-args \
  -p k_p:=5.0 -p k_i:=0.1 -p k_d:=0.05 \
  -p steer_max_deg:=45.0
```

---

## Manual Testing

**Monitor Camera Stream**:
```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
ros2 topic hz /tpc_rover_d415_rgb
```

**Monitor Lane Detection**:
```bash
ros2 topic echo /tpc_rover_nav_lane
```

**Monitor Control Commands**:
```bash
ros2 topic hz /tpc_rover_ctrl_cmd
```

**View All Topics**:
```bash
ros2 topic list
```

---

## Node Initialization Sequence

Proper timing ensures each node is ready before the next starts:
1. **0s**: Camera stream starts, hardware initialization
2. **2s**: Lane detection starts (camera stream ready)
3. **3s**: Steering control starts (lane detection ready)
