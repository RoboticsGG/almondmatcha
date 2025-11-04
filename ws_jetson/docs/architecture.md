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
[Steering Control Node] -> /tpc_rover_fmctl (Steering command)
                        -> logs/rover_ctl_log_ver_3.csv (Control logging)
    v
Steering Actuator (Front Module)
```

## Node Communication

| Node | Publishes | Subscribes | Rate |
|------|-----------|-----------|------|
| **camera_stream** | `/tpc_rover_d415_rgb`, `/tpc_rover_d415_depth` | - | 30 FPS |
| **lane_detection** | `/tpc_rover_nav_lane` | `/tpc_rover_d415_rgb` | 25-30 FPS |
| **steering_control** | `/tpc_rover_fmctl` | `/tpc_rover_nav_lane` | 50 Hz |

## Message Types

### Camera Stream
- **RGB frames**: `sensor_msgs/Image` (bgr8 format)
- **Depth frames**: `sensor_msgs/Image` (16UC1 format, optional)

### Lane Detection
- **Lane parameters**: `std_msgs/Float32MultiArray` with [theta, b, detected]
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

1. RGB to LAB color space conversion
2. Color-based lane filtering
3. Sobel gradient edge detection
4. Binary image creation
5. Perspective transform (bird's-eye view)
6. Polyfit lane boundary detection
7. Theta and b parameter calculation

## Control Algorithm

```
Combined Error: e = k_e1 * theta_ema + k_e2 * b_ema
PID Output: u = k_p * e + k_i * integral(e, dt) + k_d * de/dt
Steering: steer = clamp(u, -steer_max_deg, steer_max_deg)
If lane not detected: steer = steer_when_lost
```

## Performance Specifications

| Metric | Value |
|--------|-------|
| Max camera FPS | 30 |
| Typical end-to-end latency | 100-150 ms |
| EMA filter warmup time | ~1.5 seconds (default alpha=0.05) |
| Memory per node | 150-200 MB |
| Processing resolution | 1280x720 |
