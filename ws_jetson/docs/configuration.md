# Configuration Guide

## Overview

Configuration follows ROS2 best practices using YAML files located in `vision_navigation/config/`.

**No parameter duplication**: System config and control tuning are separate files.

## Configuration Files

### System Configuration (Camera & Lane Detection)

**For headless mode (production/SSH)**:
```yaml
# vision_nav_headless.yaml
camera_stream:
  ros__parameters:
    width: 960
    height: 540
    fps: 30
    open_cam: false        # GUI disabled
    enable_depth: false
    video_path: ""
    loop_video: true
    json_config: ""

lane_detection:
  ros__parameters:
    show_window: false     # No visualization
    # ROI trapezoid [x0,y0,...,x3,y3] authored at roi_base_width x roi_base_height,
    # scaled to the actual camera_stream resolution at runtime
    roi_base_points: [0.0, 500.0, 1280.0, 500.0, 900.0, 200.0, 400.0, 200.0]
    roi_base_width: 1280.0
    roi_base_height: 720.0
    crop_margin_px: 20.0   # Margin around the ROI bounding box before cropping
    sliding_windows: 9     # Tied to the cropped canvas size -- retune if ROI/resolution change
    window_margin: 100
    min_window_pixels: 50
    min_lane_pixels: 50
```

**For GUI mode (debugging/testing)**:
```yaml
# vision_nav_gui.yaml
camera_stream:
  ros__parameters:
    width: 960
    height: 540
    fps: 30
    open_cam: true         # GUI enabled
    enable_depth: false
    video_path: ""
    loop_video: true
    json_config: ""

lane_detection:
  ros__parameters:
    show_window: true      # Visualization enabled
    # Keep in sync with vision_nav_headless.yaml so GUI debugging reflects
    # production ROI/crop behavior
    roi_base_points: [0.0, 500.0, 1280.0, 500.0, 900.0, 200.0, 400.0, 200.0]
    roi_base_width: 1280.0
    roi_base_height: 720.0
    crop_margin_px: 20.0
    sliding_windows: 9
    window_margin: 100
    min_window_pixels: 50
    min_lane_pixels: 50
```

### Steering Control Parameters (Separate File)

Keep this separate for easy tuning without changing system config:

```yaml
# rover_kinematic_control_params.yaml
rover_kinematic_control:
  ros__parameters:
    k_e1: 1.0              # Weight on heading error (theta)
    k_e2: 0.1              # Weight on lateral offset (b)
    k_p: 4.0               # Proportional gain
    k_i: 0.0               # Integral gain
    k_d: 0.0               # Derivative gain
    k_ff: 1000.0           # Feedforward gain on filtered curvature
    ema_alpha: 0.05        # EMA smoothing (0.0-1.0)
    steer_max_deg: 60.0    # Max steering angle
    steer_when_lost: 0.0   # Steering when lane lost
```

## Modifying Configuration

### Tune Control Parameters (Most Common)

```bash
cd ~/almondmatcha/ws_jetson
nano vision_navigation/config/rover_kinematic_control_params.yaml
# Edit: k_p, k_i, k_d, k_ff, ema_alpha, etc.
./build_inc.sh
```

### Change System Configuration

**For production**:
```bash
nano vision_navigation/config/vision_nav_headless.yaml
./build_inc.sh
```

**For debugging**:
```bash
nano vision_navigation/config/vision_nav_gui.yaml
./build_inc.sh
```

### Override Parameters at Launch

```bash
# Override steering gains at runtime
ros2 launch vision_navigation vision_nav_gui.launch.py \
  k_p:=5.0 k_i:=0.15 k_d:=0.2

# Override camera settings
ros2 launch vision_navigation vision_nav_gui.launch.py \
  width:=640 height:=480 fps:=15

# Enable depth + visualization
ros2 launch vision_navigation vision_nav_gui.launch.py \
  enable_depth:=true show_window:=true
```

### Dynamic Parameter Update (While Running)

```bash
# Get current value
ros2 param get /rover_kinematic_control k_p

# Update value
ros2 param set /rover_kinematic_control k_p 5.0

# List all parameters
ros2 param list
```

## Creating Custom Configurations

### For Control Tuning

```bash
cd ~/almondmatcha/ws_jetson/vision_navigation/config

# Create copy with your tuning
cp rover_kinematic_control_params.yaml my_aggressive_tune.yaml

# Edit your copy
nano my_aggressive_tune.yaml

# Rebuild
cd ~/almondmatcha/ws_jetson
./build_inc.sh
```

### For System Configuration

```bash
cd ~/almondmatcha/ws_jetson/vision_navigation/config

# Create copy
cp vision_nav_gui.yaml my_custom_system.yaml

# Edit your copy
nano my_custom_system.yaml

# Update launch file to use my_custom_system.yaml
cd ~/almondmatcha/ws_jetson
./build_inc.sh
```

## Verifying Configuration

**Check installed configs**:
```bash
ls $(ros2 pkg prefix vision_navigation)/share/vision_navigation/config/
```

**Check loaded parameters (while node running)**:
```bash
ros2 param list /camera_stream
ros2 param get /camera_stream width

ros2 param list /rover_kinematic_control
ros2 param get /rover_kinematic_control k_p
```

## Tuning Examples

### Smooth Lane Following (Gentle Turns)

Edit `rover_kinematic_control_params.yaml`:
```yaml
k_e1: 0.8              # Reduce heading error weight
k_e2: 0.05             # Reduce offset weight
k_p: 3.0               # Lower proportional gain
steer_max_deg: 45      # Reduce max angle
ema_alpha: 0.05        # Keep default smoothing
```

### Aggressive Tracking (Sharp Turns)

Edit `rover_kinematic_control_params.yaml`:
```yaml
k_e1: 1.5              # Increase heading error weight
k_e2: 0.2              # Increase offset weight
k_p: 5.0               # Higher proportional gain
steer_max_deg: 60      # Increase max angle
ema_alpha: 0.02        # Reduce smoothing (faster response)
```

### With Integral Control (Steady-State Correction)

Edit `rover_kinematic_control_params.yaml`:
```yaml
k_p: 4.0
k_i: 0.1               # Add integral term
k_d: 0.02              # Add derivative damping
```

### Curve Anticipation (Feedforward)

Edit `rover_kinematic_control_params.yaml`:
```yaml
k_ff: 1500.0            # Raise if the rover steers in late on curves;
                         # lower if it steers in before the curve is there
```

## Lane Detection Tuning

**Via config (no rebuild needed to test, restart node to apply)** -- edit `vision_nav_headless.yaml` / `vision_nav_gui.yaml` under `lane_detection.ros__parameters`:
- `roi_base_points` / `roi_base_width` / `roi_base_height`: ROI trapezoid location
- `crop_margin_px`: margin around the ROI bounding box before cropping
- `sliding_windows` / `window_margin` / `min_window_pixels` / `min_lane_pixels`: sliding-window search tuning -- these are tied to the cropped canvas size, so retune them if you change `roi_base_points`, `crop_margin_px`, or the camera resolution

**Via code (requires rebuild)** -- edit thresholds in `vision_navigation/lane_detector.py` `preprocess_frame()`:
- `green_mask` thresholds: Filter out green background
- Sobel gradient thresholds: Edge detection sensitivity
- Magnitude and direction filtering: Edge feature extraction
- Noise filtering uses a fixed 3x3 morphological opening (not a separately tunable area threshold)

## Camera Settings

### For Low Light
```bash
ros2 launch vision_navigation vision_nav_gui.launch.py \
  json_config:="/path/to/lowlight.json"
```

### For High Speed
```bash
ros2 launch vision_navigation vision_nav_gui.launch.py \
  fps:=30 width:=1280 height:=720
```

### For Low Power / SSH
```bash
ros2 launch vision_navigation vision_nav_headless.launch.py \
  fps:=15 width:=640 height:=480
```

## Data Logging

All nodes automatically log CSV data to the `logs/` directory.

### Lane Detection Log
```
timestamp,curvature,theta,b,detected
2025-11-04T10:30:45.123456,0.000412,5.23,45.67,1.0
```

### Control Loop Log
```
time_sec,theta_ema,b_ema,curvature_ema,pid_u,e_sum,steer_angle,speed_cmd,detected
1730708000.123,4.85,44.25,0.000412,19.4,5.12,20.0,50,1
```

Check logs:
```bash
ls -lh ~/almondmatcha/ws_jetson/logs/
tail -f ~/almondmatcha/ws_jetson/logs/rover_ctl_log_ver_3.csv
```
