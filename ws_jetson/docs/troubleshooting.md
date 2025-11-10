# Troubleshooting & FAQ

## Camera Issues

### Camera Not Detected

**Problem**: "Cannot find RealSense device"

**Solutions**:
1. Check USB connection - ensure cable is properly connected
2. Verify device: `rs-enumerate-devices`
3. Install librealsense: `sudo apt install librealsense2`
4. Check device serial number matches code (current: 806312060441)
5. Check permissions: `sudo usermod -a -G video $USER` (then reboot)

### No Frames Received

**Problem**: Camera stream node runs but no frames on topic

**Check connectivity**:
```bash
ros2 topic list | grep d415
ros2 topic hz /tpc_rover_d415_rgb
```

**Verify camera is working**:
```bash
which realsense-viewer
realsense-viewer  # Visual test
```

---

## Lane Detection Issues

### Lane Not Detected

**Problem**: Lane detection not working or inconsistent

**Quick checks**:
1. Enable visualization: `lane_detection --ros-args -p show_window:=True`
2. Check lighting conditions (shadows, reflections)
3. Verify lane markers are visible and contrasting
4. Ensure camera is properly mounted and focused

**Tuning**:
- Adjust color thresholds in `vision_navigation_pkg/lane_detector.py`
- Check Sobel gradient sensitivity
- Verify ROI (region of interest) is correct

### High False Detections

**Problem**: Lane detected when it shouldn't be

**Solutions**:
1. Increase `green_mask` threshold to filter more grass
2. Tighten `red_mask` thresholds for lane markers
3. Increase contour area minimum threshold
4. Check lighting and shadows

---

## Steering Control Issues

### Steering Commands Not Received

**Problem**: `/tpc_rover_fmctl` topic empty

**Check system status**:
```bash
ros2 node list
ros2 topic list | grep fmctl
ros2 topic info /tpc_rover_fmctl
```

**Verify dependencies**:
1. Camera stream running: `ros2 topic hz /tpc_rover_d415_rgb`
2. Lane detection running: `ros2 topic hz /tpc_rover_nav_lane`
3. Steering control running: `ros2 node list | grep steering`

### Steering Oscillation

**Problem**: Oscillating or unstable steering

**Quick fixes**:
1. Increase EMA smoothing: `ema_alpha:=0.02`
2. Reduce proportional gain: `k_p:=2.0`
3. Add derivative damping: `k_d:=0.05`
4. Reduce error weights: `k_e1:=0.5 k_e2:=0.05`

### Steering Too Slow

**Problem**: Slow steering response

**Solutions**:
1. Increase proportional gain: `k_p:=6.0`
2. Decrease EMA smoothing: `ema_alpha:=0.1`
3. Increase error weights: `k_e1:=1.5 k_e2:=0.2`
4. Add integral control: `k_i:=0.1`

---

## Performance Issues

### High CPU Usage

**Problem**: Frame rate drops, CPU at 100%

**Solutions**:
1. Reduce frame rate: `fps:=15`
2. Reduce resolution: `width:=640 height:=480`
3. Disable visualization: `show_window:=False`
4. Disable depth stream: `enable_depth:=False`
5. Check monitor tasks: `htop`, look for background processes

### High Latency

**Problem**: End-to-end latency > 200ms

**Causes and fixes**:
1. High CPU usage (see above)
2. Network congestion (check ROS_DOMAIN_ID)
3. GPU memory full (check with `nvidia-smi`)
4. Thermal throttling (check temperature: `cat /sys/class/thermal/thermal_zone0/temp`)

### Memory Issues

**Problem**: OOM killer activating, nodes crashing

**Check memory**:
```bash
free -h
nvidia-smi
```

**Solutions**:
1. Reduce resolution / frame rate
2. Stop other processes
3. Check for memory leaks in nodes
4. Restart system if degradation over time

---

## Build Issues

### Build Fails

**Problem**: `colcon build` fails with errors

**Check logs**:
```bash
cat log/latest_build/vision_navigation/stdout_stderr.log
```

**Common causes**:
1. Missing dependencies: `pip3 install pyrealsense2 numpy opencv-python`
2. Stale build artifacts: `./build_clean.sh`
3. Python version mismatch: Requires Python 3.10+

### Scripts Not Found After Build

**Problem**: `camera_stream: command not found`

**Solution**:
```bash
source ~/almondmatcha/ws_jetson/install/setup.bash
```

---

## Launch File Issues

### Launch File Not Found

**Problem**: `package 'vision_navigation' not found`

**Solutions**:
1. Source setup: `source ~/almondmatcha/ws_jetson/install/setup.bash`
2. Rebuild package: `./build_clean.sh`
3. Check package.xml is in correct location

### Wrong Mode Launching

**Problem**: Headless launch showing GUI windows (or vice versa)

**Check config**:
```bash
cat vision_navigation/config/vision_nav_headless.yaml | grep -A2 camera_stream
```

**Should show**: `open_cam: false`

**Fix**: Edit YAML file and rebuild

---

## Network/Communication Issues

### Topics Not Visible Across Nodes

**Problem**: Topic publishers and subscribers can't see each other

**Check ROS_DOMAIN_ID**:
```bash
echo $ROS_DOMAIN_ID
```

Launch files set Domain 5 automatically. Verify in launch file output.

**Test connectivity**:
```bash
ros2 topic list
ros2 topic pub /test std_msgs/String "data: hello"  # In one terminal
ros2 topic echo /test                                 # In another
```

### Firewall Issues

**Problem**: Nodes on different machines can't communicate

**Solutions**:
1. Check firewall: `sudo ufw status`
2. Allow DDS ports: `sudo ufw allow 7400:7409/udp`
3. Disable firewall temporarily: `sudo ufw disable`
4. Check network connectivity: `ping <other_node_ip>`

---

## Common Warnings (Safe to Ignore)

### "logging was initialized more than once"

This is expected when using multiple ROS2 domains. Safe to ignore.

### "Failed to find data type"

Usually appears during launch - safe to ignore if nodes start.

### Parameter Already Declared

Can occur if node is restarted - safe to ignore.

---

## Getting Help

1. Check current running processes: `ros2 node list`
2. Check all topics: `ros2 topic list`
3. Monitor topic data: `ros2 topic echo /tpc_rover_d415_rgb`
4. Check parameter values: `ros2 param list`
5. View recent logs: `cat log/latest_build/vision_navigation/stdout_stderr.log`
6. Check system resources: `free -h`, `nvidia-smi`, `htop`

## Performance Baseline

For reference, expected performance on Jetson Orin Nano:
- Camera FPS: 30
- Lane detection: 25-30 FPS
- Control loop: 50 Hz
- End-to-end latency: 100-150 ms
- CPU usage: 40-60% (GUI disabled)
- Memory: 150-200 MB per node
