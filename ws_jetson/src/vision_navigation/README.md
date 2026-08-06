# Vision Navigation System

Real-time visual navigation system for autonomous rover lane detection and steering control. This system processes Intel RealSense D415 camera data or video files to detect lane boundaries and compute steering commands via closed-loop static-gain control.

## System Overview

The Vision Navigation system consists of three coordinated ROS2 nodes:

1. **Camera Stream Node** - Acquires and publishes RGB/depth frames
2. **Lane Detection Node** - Detects lane markers and computes steering parameters
3. **Rover Kinematic Control Node** - Implements static-gain steering + speed control

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
  - curvature: Parabola coefficient A (x = A*y^2 + B*y + C); BEV pixels (1/px), NOT converted to a real 1/m arc on the wire -- radius R = 1/(2*A*200); positive = curves right ahead, negative = curves left ahead
  - theta: Heading error from lane center (real degrees, positive = right)
  - b: Lateral offset from lane center, **metres** (converted from the fit's native BEV pixels before publishing; positive = right)

  theta and b are measured at a 1.22 m lookahead ahead of the front axle, not at the rover.
  - detected: Detection flag (1.0 = valid, 0.0 = not detected)

Parameters:
- `show_window` (bool, default False): Display lane detection visualization
- `roi_base_points` (double array, 8 values): ROI trapezoid corners [x0,y0,...,x3,y3] at (roi_base_width x roi_base_height)
- `roi_base_width` / `roi_base_height` (float, default 1280.0 / 720.0): reference resolution roi_base_points were authored against
- `crop_margin_px` (float, default 20.0): margin around the ROI bounding box before cropping (in roi_base_width/height units, scaled to actual frame size)
- `bev_width_px` / `bev_height_px` (int, default 720 / 340): fixed bird's-eye canvas, sets the 200 px/m metric scale
- `sliding_windows` (int, default 9): number of vertical search windows
- `window_margin` (int, default 40): search window half-width in pixels. **Hard ceiling 75** -- half the smaller adjacent-line gap (measured track: 2.50 m wide, lines 5 cm, centre line offset ±0.50 m from the track's own midpoint -> asymmetric gaps of 0.75 m / 1.75 m -> 150 px at 200 px/m)
- `search_band_px` (float, default 45.0): how far from the previous frame's result the search may start. **Hard ceiling 75**, same basis as `window_margin` -- stops the detector locking onto a neighbouring painted line
- `max_abs_b_m` (float, default 0.50): absolute plausibility bound on the fitted lateral offset in metres, matching the downstream clamp in rover_kinematic_control -- a fit beyond this is rejected instead of reported as a detection
- `min_window_pixels` (int, default 50): minimum pixels to recenter a search window
- `min_lane_pixels` (int, default 50): minimum total detected pixels required for a valid fit

CSV Output: columns [timestamp, curvature, theta, b, detected, fps]. Written on a background thread (queue-based) so file I/O never blocks the image callback.

Processing Pipeline:
1. Crop frame to the ROI bounding box (plus margin) -- cuts CPU on the steps below without reducing pixel density inside the ROI
2. Adaptive (per-frame Otsu) red-track / white-line color segmentation -- LAB a* for red, chroma distance from neutral for white; see `segment_track_colors()`
3. Perspective transform (bird's-eye view)
4. Shape filter on line candidates in BEV -- rejects broad/short blobs (e.g. sun glare) that pass the color stage but aren't shaped like a painted line; see `filter_line_candidates_bev()`
5. 2nd-degree polyfit lane boundary detection (parabola, not a straight line)
6. Curvature, theta, and b parameter calculation (lookahead-centered frame)

See [docs/VISION_PIPELINE.md](../../../docs/VISION_PIPELINE.md) for the full
derivation: camera geometry, why the ROI is what it is, the metric bird's-eye
scale, and the two parameters with hard correctness ceilings.

#### Rover Kinematic Control Node (rover_kinematic_control)

Implements closed-loop static-gain steering + speed control based on lane detection parameters.

Subscribed Topics:
- `/tpc_rover_nav_lane` (std_msgs/Float32MultiArray): Lane parameters [curvature, theta, b, detected]

Published Topics:
- `/tpc_rover_ctrl_cmd` (std_msgs/Float32MultiArray): [steer_angle, speed_cmd, detected]
  - steer_angle: Steering command in degrees (positive = right, negative = left)
  - speed_cmd: Speed command (0–100% PWM duty cycle)
  - detected: Lane detection status flag

Parameters:
- `k_lat` (float, default 181.17): Gain on lateral offset b (deg/metre)
- `k_head` (float, default 2.024): Gain on heading error theta (deg/deg)
- `wheelbase_m` (float, default 0.4875): Front-to-rear axle distance, metres (Ackermann feedforward)
- `bev_px_per_m` (float, default 200.0): BEV pixel scale, converts curvature from 1/px to 1/m
- `speed_ref` (float, default 50.0): Base speed when lane detected (0–100% duty cycle)
- `speed_lost_ratio` (float, default 0.5): Speed multiplier when lane temporarily lost
- `detection_timeout_sec` (float, default 10.0): Seconds before speed drops to 0 on sustained lane loss
- `ema_alpha` (float, default 0.05): Exponential moving average smoothing (0-1), applied to theta, b, and curvature
- `steer_max_deg` (float, default 45.0): Maximum steering angle saturation (conservative margin below the ±60° mechanical limit)
- `steer_when_lost` (float, default 0.0): Steering command when lane not detected

Control Algorithm (static-gain feedback + Ackermann feedforward, single-step
MPC form -- `k_lat`/`k_head` are a fixed discrete-time LQR solution on the
linearized error model `b_dot = v*theta, theta_dot = -(v/L)*delta` (v ~
0.15-0.20 m/s, L = wheelbase_m, dt ~ 1/20 s measured control-loop period),
solved offline as `k1 = 3.162 rad/m, k2 = 2.024 rad/rad` then converted to
this codebase's units: `k_lat = k1*(180/pi) = 181.17` (b is metres, steer
is degrees, so this ratio needs the rad->deg conversion) and `k_head = k2 =
2.024` (theta and steer are both angles, so the ratio is unit-invariant)):
```
Feedback:          u_fb    = k_lat * b_ema + k_head * theta_ema
Feedforward:       u_ff    = atan(wheelbase_m * curvature_m_ema)   [curvature_m_ema = 2 * bev_px_per_m * curvature_ema]
Combined Output:   u_total = u_fb + u_ff
Steering:          steer   = clamp(u_total, -steer_max_deg, steer_max_deg)
If not detected:   steer   = steer_when_lost
```
theta, b, and steer_angle all use "+ = correct by steering right" in this
codebase, so both feedback terms are ADDED. Do not copy a "-k1*e_lat -
k2*e_heading" form from a textbook without checking its sign convention
against this one first -- it typically assumes positive steer = left.

CSV Output: columns [time_sec, theta_ema, b_ema, curvature_ema, u_fb, u_ff, steer_angle, speed_cmd, detected]

## Configuration and Tuning

### Centralized Configuration (config.py)

All system parameters are centralized in `vision_navigation/config.py`:

- CameraConfig: Resolution, FPS, camera modes
- LaneDetectionConfig: Color thresholds, gradient parameters, window settings
- ControlConfig: static feedback gains, wheelbase, saturation limits
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
     k_lat:=150.0 k_head:=1.5
   ```

2. Save best values to config.py:
   ```python
   class ControlConfig:
       K_LAT = 150.0   # Updated from testing
       K_HEAD = 1.5
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

Smooth lane following (gentle turns, gentler than the shipped LQR default):
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  k_lat:=100.0 k_head:=1.5 steer_max_deg:=30
```

Aggressive tracking (sharp turns, at/above the shipped LQR default):
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  k_lat:=181.17 k_head:=2.5 steer_max_deg:=45
```

There is no integral or derivative term in this control law (pure
proportional static gain + feedforward) -- steady-state offset and damping
are addressed by re-tuning `k_lat`/`k_head` or `ema_alpha`, not by adding I/D
terms.

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

### lane_detector.py

Lane detection pipeline functions:
- `segment_track_colors()`: Adaptive (per-frame Otsu) red-track / white-line color segmentation
- `filter_line_candidates_bev()`: Shape filter (tall & narrow) that rejects glare/blob false positives in the warped BEV view
- `get_scaled_roi_points()`: Scale ROI corners from base to actual frame resolution
- `crop_to_roi()`: Crop frame to the ROI bounding box before preprocessing
- `perspective_transform()`: Bird's-eye view transformation
- `find_center_line()`: Sliding window lane tracking
- `compute_lane_params()`: Curvature, theta, and b parameter calculation (2nd-degree fit)
- `process_frame()`: Complete lane detection pipeline (crop -> segment colors -> transform to BEV -> shape-filter -> detect -> params)
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

### Lane Detection Log (`<ws_jetson>/runs/run_NNN_<stamp>/lane_detection.csv`)

Columns: timestamp, curvature, theta, b, detected, fps

`fps` is a rolling-window measurement of achieved throughput, not the configured target.

Example:
```
2026-08-04T10:30:45.123456,0.000412,5.23,45.67,1.0,27.3
2026-08-04T10:30:45.153456,-0.000298,-3.21,42.11,1.0,27.1
```

### Steering Control Log (`<ws_jetson>/runs/run_NNN_<stamp>/kinematic_control.csv`)

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
- Camera resolution: 1280x720 (configured in vision_nav_headless.yaml / vision_nav_gui.yaml)
- Preprocessing runs on the ROI crop (~1234x229 at default roi_base_points/crop_margin_px), not the full frame

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
2. Reduce the feedback gains: `-p k_lat:=100.0 -p k_head:=1.0`
3. Confirm sign convention was not inverted after edits (see Sign Conventions below) --
   a flipped sign turns feedback into positive feedback, which reads as unbounded oscillation/runaway, not gentle instability

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
- Units: Metres

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
├── config/
│   ├── vision_nav_gui.yaml             # Camera + lane detection, GUI mode
│   ├── vision_nav_headless.yaml        # Camera + lane detection, headless mode
│   └── rover_kinematic_control_params.yaml  # Steering/speed control gains
├── launch/
│   ├── vision_navigation.launch.py     # Single-domain launch; only file that wires launch args into node parameters
│   ├── vision_nav_gui.launch.py        # GUI mode (loads vision_nav_gui.yaml)
│   ├── vision_nav_headless.launch.py   # Headless mode (loads vision_nav_headless.yaml)
│   ├── vision_domain6.launch.py        # Vision-only nodes, Domain 6
│   └── control_domain5.launch.py       # Kinematic control only, Domain 5
├── vision_navigation/
│   ├── __init__.py
│   ├── config.py                       # Centralized configuration
│   ├── helpers.py                      # Utility functions
│   ├── camera_stream_node.py           # Camera streaming node
│   ├── camera_recorder_node.py         # Field-run video recording (debug)
│   ├── lane_detection_node.py          # Lane detection node
│   ├── rover_kinematic_control_node.py # Kinematic control node
│   ├── lane_detector.py                # Lane detection pipeline
│   ├── control_filters.py              # Filters and utilities
│   └── demo_lane.py                    # Standalone lane-detection demo/debug script
├── resource/                           # Package resources
└── test/                               # Unit tests
```

## Version History

### v1.3.0 (August 6, 2026)

Replaces the PID steering controller with a static-gain feedback + Ackermann
feedforward law (single-step MPC form): `steer = k_lat*b_ema + k_head*theta_ema
+ atan(wheelbase_m*curvature_m_ema)`. `k_lat`/`k_head` are a fixed
discrete-time LQR solution, not re-solved online each step -- solved offline
against the linearized error model `b_dot = v*theta, theta_dot =
-(v/L)*delta` (v ~ 0.15-0.20 m/s measured cruise speed, L = wheelbase_m,
dt ~ 1/20 s measured control-loop period) as `k1 = 3.162 rad/m, k2 = 2.024
rad/rad`, then converted to this codebase's units: `k_lat = k1*(180/pi) =
181.17` (b is metres, steer is degrees -- needs the rad->deg conversion),
`k_head = k2 = 2.024` (theta and steer are both angles -- the ratio is
unit-invariant, no conversion needed).

- Removed `k_p`/`k_i`/`k_d`/`k_e1`/`k_e2`/`k_ff` and the integral/derivative
  state; replaced with `k_lat`, `k_head`, `wheelbase_m`, `bev_px_per_m`
- Feedforward switched from the small-angle linear approximation
  (`k_ff * curvature_ema`) to the exact Ackermann angle
  (`atan(wheelbase_m * curvature_m_ema)`) -- numerically near-identical on
  this track (curvature is small enough that atan(x) ≈ x), but removes the
  approximation entirely for a negligible extra cost
- `steer_max_deg` default lowered from 60.0 to 45.0 as a conservative margin
  while the new law is unvalidated on the track. Note: at `k_lat=181.17`,
  even a modest lookahead offset saturates this clamp on its own (b=0.15m ->
  27 deg from the b-term alone) -- expect the commanded angle to sit at/near
  the clamp often on this track, not only on sharp turns
- EMA low-pass filtering on theta/b/curvature is unchanged -- only the
  feedback stage (PID -> static gain) and feedforward form changed
- theta/b/steer_angle all use "+ = correct by steering right" in this
  codebase, so both feedback terms are ADDED, not subtracted -- see Sign
  Conventions below before copying a control law from a source that may
  assume the opposite convention

### v1.2.0 (August 5, 2026)

Replaces `preprocess_frame()` (fixed LAB green mask + Sobel gradient +
`gray > 180` white threshold) after an audit against a mobile-phone photo of
the actual track found it did no positive red-surface segmentation at all,
and that a sun-glare patch on the track reads at nearly the same HSV
brightness/saturation as real white paint, so no fixed brightness threshold
can separate them.

- `segment_track_colors()` added: red-vs-background and white-vs-red splits
  are computed per frame with Otsu's method (LAB `a*` for red, chroma
  distance from neutral for white) instead of fixed constants, so the split
  re-fits to whatever the current camera/lighting produced rather than one
  hand-picked number. Fixes a real bug found while building it: taking "the
  single largest red blob" silently dropped half the track, because the
  white centre line cuts it into two disconnected halves — connectivity is
  now computed on `(red | white)` together first.
- `filter_line_candidates_bev()` added: color alone cannot separate a
  sun-glare patch from real paint when they're genuinely close in color, so
  this rejects candidates by shape instead, in the metric bird's-eye view —
  real lines are tall and narrow and persist over the ROI's depth; glare
  blobs are not. Uses a per-row run-width prune (not just a whole-component
  test) so a glare patch touching a real line doesn't drag the line's own
  pixels down with it when rejected.
- `perspective_transform()` now warps with `INTER_NEAREST`: linear
  interpolation was blending 0/1 mask values into fractional pixels at
  edges, which the new shape filter's connected-component analysis needs to
  not happen.
- Removed dead config constants that the old pipeline never read outside
  `config.py` itself: `LAB_GREEN_A_MAX`, `LAB_GREEN_B_MIN`, `LAB_RED_A_MIN`,
  `LAB_RED_B_MAX`, `GRADIENT_SOBEL_*`, `MAGNITUDE_*`, `DIRECTION_*`,
  `WHITE_THRESHOLD`, `MIN_CONTOUR_AREA`. Removed the resulting unused
  `matplotlib` import from `lane_detector.py`.
- `capture_roi_debug.py` updated to exercise the new pipeline (it calls the
  segmentation and shape-filter functions directly, not through
  `process_frame()`, so it needed its own update to stay representative of
  node behavior).
- **Not yet validated against the D415.** Designed and tested against a
  phone photo only (no D415 footage existed at the time) — the architecture
  (adaptive per-frame splits, BEV shape filter) is camera-independent by
  construction, but needs re-running against real D415 captures across a
  spread of lighting before being trusted unattended. See
  `docs/VISION_PIPELINE.md` section 2.

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
