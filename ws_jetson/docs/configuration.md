# Configuration Guide

## Overview

Configuration follows ROS2 best practices using YAML files located in
`src/vision_navigation/config/`.

**No parameter duplication**: System config and control tuning are separate files.

## Configuration Files

### System Configuration (Camera & Lane Detection)

**For headless mode (production/SSH)**:
```yaml
# vision_nav_headless.yaml
camera_stream_node:
  ros__parameters:
    width: 1280
    height: 720
    fps: 30
    open_cam: false        # GUI disabled
    enable_depth: false
    video_path: ""
    loop_video: true
    json_config: ""

lane_detection_node:
  ros__parameters:
    show_window: false     # No visualization
    # ROI trapezoid [x0,y0,...,x3,y3] authored at roi_base_width x roi_base_height,
    # scaled to the actual camera_stream resolution at runtime. Derived from
    # the physical camera mount (50 cm high, 15 deg tilt +-3 deg tolerance,
    # lateral centreline, 8 cm behind the front axle) -- see the source
    # comment for the ground-plane rectangle these points project, and
    # regenerate_roi.py to recompute if the mount changes.
    roi_base_points: [39.0, 458.0, 1241.0, 458.0, 915.0, 270.0, 365.0, 270.0]
    roi_base_width: 1280.0
    roi_base_height: 720.0
    crop_margin_px: 20.0   # Margin around the ROI bounding box before cropping
    # Bird's-eye output canvas -- fixed size, not the crop size, so both axes
    # carry the same metres-per-pixel (200 px/m). This is what makes theta a
    # real heading angle, b a real cross-track offset, and curvature a real 1/m arc.
    bev_width_px: 720
    bev_height_px: 340
    sliding_windows: 9     # Tied to the cropped canvas size -- retune if ROI/resolution change
    # MUST stay below half the spacing between adjacent painted lines (61 px
    # on this track), or a window captures two lines at once.
    window_margin: 40
    min_window_pixels: 50
    min_lane_pixels: 50
    # Bounds where the search may start relative to last frame -- stops the
    # detector locking onto the wrong painted line. Also the per-frame jump limit.
    search_band_px: 45.0
    # Absolute plausibility bound on the fitted offset (100 px = 0.50 m, the
    # same clamp used downstream in rover_kinematic_control). Past it, the
    # reading can't be a real measurement -- reject the lock instead of
    # reporting "Detected" on a bogus value.
    max_abs_b_px: 100.0
```

**For GUI mode (debugging/testing)**: `vision_nav_gui.yaml` uses the same
`camera_stream_node` / `lane_detection_node` keys and the same lane-detection
values as headless (kept in sync so GUI debugging reflects production ROI/crop
behavior); it differs only in `open_cam: true` and `show_window: true`.

### Steering Control Parameters (Separate File)

Keep this separate for easy tuning without changing system config:

```yaml
# rover_kinematic_control_params.yaml
rover_kinematic_control:
  ros__parameters:
    k_e1: 1.0              # Weight on heading error (theta)
    k_e2: 0.1              # Weight on lateral offset (b)
    k_p: 4.0               # Proportional gain
    k_i: 0.01              # Integral gain
    k_d: 0.01              # Derivative gain
    # Derivable from the metric bird's-eye view (k_ff = 2*S*L*(180/pi) = 11172.7
    # for this rover's 48.75 cm wheelbase). Shipped at 0.0: on this track the
    # correct feedforward is under 1 deg, below the curvature fit's noise
    # floor, while a bad fit times 11172.7 would inject a large steering kick.
    k_ff: 0.0
    ema_alpha: 0.05        # EMA smoothing (0.0-1.0)
    # Target cruise duty (0-100% PWM) when the lane is detected. 16 is the
    # measured runnable cruise duty on this rover (11 stalls on the ramp).
    speed_ref: 16
    steer_max_deg: 60.0    # Max steering angle
    steer_when_lost: 0.0   # Steering when lane lost
```

## Modifying Configuration

### Tune Control Parameters (Most Common)

```bash
cd ~/almondmatcha/ws_jetson
nano src/vision_navigation/config/rover_kinematic_control_params.yaml
# Edit: k_p, k_i, k_d, k_ff, ema_alpha, speed_ref, etc.
./build_inc.sh
```

### Change System Configuration

**For production**:
```bash
nano src/vision_navigation/config/vision_nav_headless.yaml
./build_inc.sh
```

**For debugging**:
```bash
nano src/vision_navigation/config/vision_nav_gui.yaml
./build_inc.sh
```

### Override Parameters at Launch

**Only `vision_navigation.launch.py` wires launch arguments into node
parameters** — `vision_nav_gui.launch.py` and `vision_nav_headless.launch.py`
declare the same-named arguments but only ever pass the YAML file path as
`parameters=[system_config]`, so those arguments are silently ignored by the
node. Use `vision_navigation.launch.py` for runtime overrides:

```bash
# Override steering gains at runtime
ros2 launch vision_navigation vision_navigation.launch.py \
  k_p:=5.0 k_i:=0.15 k_d:=0.2

# Override camera settings
ros2 launch vision_navigation vision_navigation.launch.py \
  camera_width:=640 camera_height:=480 camera_fps:=15

# Enable depth + visualization
ros2 launch vision_navigation vision_navigation.launch.py \
  enable_depth:=true lane_visualization:=true
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
cd ~/almondmatcha/ws_jetson/src/vision_navigation/config

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
cd ~/almondmatcha/ws_jetson/src/vision_navigation/config

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
ros2 param list /camera_stream_node
ros2 param get /camera_stream_node width

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
                         # (derived full value for this rover is 11172.7 -- see
                         # docs/CONTROL_LAW.md §1.7)
```

## Lane Detection Tuning

**Via config (no rebuild needed to test, restart node to apply)** -- edit `vision_nav_headless.yaml` / `vision_nav_gui.yaml` under `lane_detection_node.ros__parameters`:
- `roi_base_points` / `roi_base_width` / `roi_base_height`: ROI trapezoid location
- `crop_margin_px`: margin around the ROI bounding box before cropping
- `bev_width_px` / `bev_height_px`: bird's-eye canvas size (keep both axes at the same px/m scale)
- `sliding_windows` / `window_margin` / `min_window_pixels` / `min_lane_pixels`: sliding-window search tuning -- these are tied to the cropped canvas size, so retune them if you change `roi_base_points`, `crop_margin_px`, or the camera resolution
- `search_band_px` / `max_abs_b_px`: per-frame jump limit and absolute plausibility bound on the fitted lateral offset

**Via code (requires rebuild)** -- edit thresholds in `vision_navigation/lane_detector.py` `preprocess_frame()`:
- `green_mask` thresholds: Filter out green background
- Sobel gradient thresholds: Edge detection sensitivity
- Magnitude and direction filtering: Edge feature extraction
- Noise filtering uses a fixed 3x3 morphological opening (not a separately tunable area threshold)

## Camera Settings

### For Low Light
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  json_config:="/path/to/lowlight.json"
```

### For High Speed
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  camera_fps:=30 camera_width:=1280 camera_height:=720
```

### For Low Power / SSH
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  camera_fps:=15 camera_width:=640 camera_height:=480
```

## Data Logging

`lane_detection_node` and `rover_kinematic_control` each log their own CSV
to a shared per-run directory: `<ws_jetson>/runs/run_NNN_<stamp>/`, one
directory per launch. `camera_stream_node` does not write a CSV.

### Lane Detection Log (`lane_detection.csv`)
```
timestamp,curvature,theta,b,detected,fps
2026-08-04T10:30:45.123456,0.000412,5.23,45.67,1.0,27.3
```

`fps` is a rolling-window measurement of achieved throughput — not the
configured target — recorded alongside each detection.

### Control Loop Log (`kinematic_control.csv`)
```
time_sec,theta_ema,b_ema,curvature_ema,pid_u,e_sum,steer_angle,speed_cmd,detected
1754300000.123,4.85,44.25,0.000412,19.4,5.12,20.0,16,1
```

Check logs:
```bash
ls -lh ~/almondmatcha/ws_jetson/runs/
tail -f ~/almondmatcha/ws_jetson/runs/run_NNN_<stamp>/kinematic_control.csv
```
