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
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): [theta, b, detected]

**Parameters**:
- `show_window` (bool, default: False): Display lane detection visualization

**CSV Logging**: `lane_pub_log.csv` (timestamp, theta, b, detected)

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
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): Lane parameters [theta, b, detected]

**Published Topics**:
- `/tpc_rover_fmctl` (std_msgs/Float32MultiArray): [steer_angle, detected]

**Parameters**:
- `k_e1` (float, default: 1.0): Weight on heading error (theta)
- `k_e2` (float, default: 0.1): Weight on lateral offset (b)
- `k_p` (float, default: 4.0): Proportional gain
- `k_i` (float, default: 0.0): Integral gain
- `k_d` (float, default: 0.0): Derivative gain
- `ema_alpha` (float, default: 0.05): Exponential moving average smoothing factor
- `steer_max_deg` (float, default: 60.0): Maximum steering angle saturation (±degrees)
- `steer_when_lost` (float, default: 0.0): Steering command when lane not detected

**CSV Logging**: `logs/rover_ctl_log_ver_3.csv` (time_sec, theta_ema, b_ema, u, e_sum)

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

**Monitor Steering Commands**:
```bash
ros2 topic hz /tpc_rover_fmctl
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
