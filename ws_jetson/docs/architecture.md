# System Architecture

## Data Flow

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
[Rover Kinematic Control Node] -> /tpc_rover_ctrl_cmd (Kinematic control cmd)
                               -> logs/ws_jetson_kinematic_ctrl_TIMESTAMP.csv (Control logging)
    v
Steering Actuator (Front Module)
```

## Node Communication

| Node | Publishes | Subscribes | Rate |
|------|-----------|-----------|------|
| **camera_stream** | `/tpc_rover_d415_rgb`, `/tpc_rover_d415_depth` | - | 30 FPS |
| **lane_detection** | `/tpc_rover_nav_lane` | `/tpc_rover_d415_rgb` | 25-30 FPS |
| **rover_kinematic_control** | `/tpc_rover_ctrl_cmd` | `/tpc_rover_nav_lane` | 50 Hz |

## Message Types

### Camera Stream
- **RGB frames**: `sensor_msgs/Image` (bgr8 format)
- **Depth frames**: `sensor_msgs/Image` (16UC1 format, optional)

### Lane Detection
- **Lane parameters**: `std_msgs/Float32MultiArray` with [curvature, theta, b, detected]
  - `curvature`: Parabola coefficient A (x = A*y^2 + B*y + C), rover-centered warped frame
  - `theta`: Heading error from lane center (degrees)
  - `b`: Lateral offset from lane center (pixels)
  - `detected`: Detection flag (1.0 = valid, 0.0 = not detected)

### Steering Control
- **Steering command**: `std_msgs/Float32MultiArray` with [steer_angle, detected]
  - `steer_angle`: Steering command in degrees (+right, -left)
  - `detected`: Lane detection status flag

## Sign Conventions

- **Steering Angle**: Positive = RIGHT, Negative = LEFT (degrees)
- **Heading Error (theta)**: Positive = lane RIGHT (turn right), Negative = lane LEFT (turn left)
- **Lateral Offset (b)**: Positive = camera RIGHT of center, Negative = camera LEFT of center (pixels)

## Lane Detection Pipeline

1. Crop frame to the ROI bounding box (plus margin) before the steps below
2. LAB color space filtering (green background removal)
3. Sobel gradient edge detection (CV_32F + cv2.cartToPolar)
4. Binary image creation
5. Morphological opening (noise removal)
6. Perspective transform (bird's-eye view)
7. 2nd-degree polyfit lane boundary detection (parabola)
8. Curvature, theta, and b parameter calculation (rover-centered frame)

## Control Algorithm

```
Combined Error:  e       = k_e1 * theta_ema + k_e2 * b_ema
Feedback (PID):  u_pid   = k_p * e + k_i * integral(e, dt) + k_d * de/dt
Feedforward:     u_ff    = k_ff * curvature_ema
Combined Output: u_total = u_pid + u_ff
Steering:        steer   = clamp(u_total, -steer_max_deg, steer_max_deg)
If lane not detected: steer = steer_when_lost
```

## Performance Specifications

| Metric | Value |
|--------|-------|
| Max camera FPS | 30 |
| Typical end-to-end latency | 100-150 ms |
| EMA filter warmup time | ~1.5 seconds (default alpha=0.05) |
| Memory per node | 150-200 MB |
| Camera resolution | 960x540 |
