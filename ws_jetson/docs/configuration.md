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
    width: 1280
    height: 720
    fps: 30
    open_cam: false        # GUI disabled
    enable_depth: false
    video_path: ""
    loop_video: true
    json_config: ""

lane_detection:
  ros__parameters:
    show_window: false     # No visualization
```

**For GUI mode (debugging/testing)**:
```yaml
# vision_nav_gui.yaml
camera_stream:
  ros__parameters:
    width: 1280
    height: 720
    fps: 30
    open_cam: true         # GUI enabled
    enable_depth: false
    video_path: ""
    loop_video: true
    json_config: ""

lane_detection:
  ros__parameters:
    show_window: true      # Visualization enabled
```

### Steering Control Parameters (Separate File)

Keep this separate for easy tuning without changing system config:

```yaml
# steering_control_params.yaml
steering_control:
  ros__parameters:
    k_e1: 1.0              # Weight on heading error (theta)
    k_e2: 0.1              # Weight on lateral offset (b)
    k_p: 4.0               # Proportional gain
    k_i: 0.0               # Integral gain
    k_d: 0.0               # Derivative gain
    ema_alpha: 0.05        # EMA smoothing (0.0-1.0)
    steer_max_deg: 60.0    # Max steering angle
    steer_when_lost: 0.0   # Steering when lane lost
```

## Modifying Configuration

### Tune Control Parameters (Most Common)

```bash
cd ~/almondmatcha/ws_jetson
nano vision_navigation/config/steering_control_params.yaml
# Edit: k_p, k_i, k_d, ema_alpha, etc.
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
ros2 param get /steering_control k_p

# Update value
ros2 param set /steering_control k_p 5.0

# List all parameters
ros2 param list
```

## Creating Custom Configurations

### For Control Tuning

```bash
cd ~/almondmatcha/ws_jetson/vision_navigation/config

# Create copy with your tuning
cp steering_control_params.yaml my_aggressive_tune.yaml

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

ros2 param list /steering_control
ros2 param get /steering_control k_p
```

## Tuning Examples

### Smooth Lane Following (Gentle Turns)

Edit `steering_control_params.yaml`:
```yaml
k_e1: 0.8              # Reduce heading error weight
k_e2: 0.05             # Reduce offset weight
k_p: 3.0               # Lower proportional gain
steer_max_deg: 45      # Reduce max angle
ema_alpha: 0.05        # Keep default smoothing
```

### Aggressive Tracking (Sharp Turns)

Edit `steering_control_params.yaml`:
```yaml
k_e1: 1.5              # Increase heading error weight
k_e2: 0.2              # Increase offset weight
k_p: 5.0               # Higher proportional gain
steer_max_deg: 60      # Increase max angle
ema_alpha: 0.02        # Reduce smoothing (faster response)
```

### With Integral Control (Steady-State Correction)

Edit `steering_control_params.yaml`:
```yaml
k_p: 4.0
k_i: 0.1               # Add integral term
k_d: 0.02              # Add derivative damping
```

## Lane Detection Tuning

Edit color thresholds in `vision_navigation_pkg/lane_detector.py`:
- `green_mask` thresholds: Filter out green grass
- `red_mask` thresholds: Detect red lane markers
- Sobel gradient thresholds: Edge detection sensitivity
- Magnitude and direction filtering: Edge feature extraction
- Contour area threshold: Noise removal

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
**File**: `lane_pub_log.csv`
```
timestamp,theta,b,detected
2025-11-04T10:30:45.123456,5.23,45.67,1.0
```

### Control Loop Log
**File**: `logs/rover_ctl_log_ver_3.csv`
```
time_sec,theta_ema,b_ema,u,e_sum
1730708000.123,4.85,44.25,19.4,5.12
```

Check logs:
```bash
ls -lh ~/almondmatcha/ws_jetson/logs/
tail -f ~/almondmatcha/ws_jetson/logs/rover_ctl_log_ver_3.csv
```
