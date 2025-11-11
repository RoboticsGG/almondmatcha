# ROS2 Domain Architecture

The rover system uses a multi-domain architecture to optimize performance and scalability. Vision processing runs isolated on Domain 6 (Jetson localhost), while the control loop operates on Domain 5 (network-wide).

## Domain Assignment

| Domain ID | Purpose | Nodes | Network Scope |
|-----------|---------|-------|---------------|
| **5** | Control Loop | ws_rpi (5), ws_base (2), ws_jetson (1), STM32 (2) = 10 nodes | Entire rover network |
| **6** | Vision Processing | ws_jetson camera, lane detection = 2 nodes | Jetson localhost only |

## Architecture Overview

```
Domain 6: Vision Processing (Jetson localhost)
┌──────────────────────────────────────────┐
│ Camera → Lane Detection                  │
│         (30 FPS, high bandwidth)         │
└────────────┬─────────────────────────────┘
             │ tpc_rover_nav_lane
             │ (localhost DDS discovery)
             │
Domain 5: Control Loop (Network-wide)
┌────────────▼─────────────────────────────┐
│ Steering Control (Jetson)                │
│ • Sub: tpc_rover_nav_lane (Domain 6)     │
│ • Pub: tpc_rover_fmctl (Domain 5)        │
└────────────┬─────────────────────────────┘
             │
┌────────────▼─────────────────────────────┐
│ ws_rpi → STM32 Boards                    │
│ Chassis control, GNSS, sensors           │
└──────────────────────────────────────────┘
             │
┌────────────▼─────────────────────────────┐
│ ws_base (Base Station)                   │
│ Mission command and monitoring           │
└──────────────────────────────────────────┘
```

## Design Rationale

**Domain 6 (Vision):**
- Isolates high-bandwidth camera streams (RGB/Depth at 30 FPS)
- Heavy computation (lane detection, future AI/ML)
- Network scope: Jetson localhost only
- Invisible to resource-constrained STM32 boards

**Domain 5 (Control):**
- Real-time control loop (10 nodes: ws_rpi=5, ws_base=2, ws_jetson=1, STM32=2)
- Low-frequency control messages
- Network scope: Entire rover network
- Manageable memory usage for STM32 boards (optimized for 60% free RAM)

**Benefits:**
- Reduced STM32 discovery overhead (10 participants vs 12+ without isolation)
- Scalable vision/AI expansion without affecting control loop
- Network bandwidth optimization (camera data stays local)
- No bridge nodes (native ROS2 localhost discovery)


## Domain Configuration

### Domain 5: Control Loop

**ws_rpi (Raspberry Pi):**
```bash
export ROS_DOMAIN_ID=5
cd ~/almondmatcha/ws_rpi
source install/setup.bash
./launch_rover_tmux.sh
```

**ws_base (Base Station):**
```bash
export ROS_DOMAIN_ID=5
cd ~/almondmatcha/ws_base
source install/setup.bash
ros2 launch mission_control mission_control.launch.py
```

**ws_jetson Control Interface:**
```bash
export ROS_DOMAIN_ID=5
cd ~/almondmatcha/ws_jetson
source install/setup.bash
ros2 launch vision_navigation control_domain5.launch.py
```

**STM32 Firmware:**
```cpp
// In platform/rtps/config.h
const uint8_t DOMAIN_ID = 5;
```

### Domain 6: Vision Processing

**ws_jetson Vision Nodes:**
```bash
export ROS_DOMAIN_ID=6
cd ~/almondmatcha/ws_jetson
source install/setup.bash
ros2 launch vision_navigation vision_domain6.launch.py
```

## Cross-Domain Communication

The Jetson runs nodes on both domains simultaneously using native DDS localhost discovery. No bridge nodes are required.

**Process:**
1. Domain 6 nodes publish vision data (camera, lane detection)
2. Domain 5 control node subscribes to Domain 6 topics via localhost DDS
3. Control node publishes steering commands to Domain 5 (network-wide)

**Launch Sequence:**

Terminal 1 (Domain 6):
```bash
export ROS_DOMAIN_ID=6
ros2 launch vision_navigation vision_domain6.launch.py
```

Terminal 2 (Domain 5):
```bash
export ROS_DOMAIN_ID=5
ros2 launch vision_navigation control_domain5.launch.py
```

## Verifying Domain Configuration

### Check Domain 5 Nodes (Control Loop)

```bash
export ROS_DOMAIN_ID=5
ros2 node list

# Expected output (visible to all systems including STM32):
/steering_control_domain5       # ws_jetson (control interface only)
/node_chassis_controller        # ws_rpi
/node_gnss_mission_monitor      # ws_rpi
/node_gnss_spresense            # ws_rpi
/node_chassis_imu               # ws_rpi
/node_chassis_sensors           # ws_rpi
/rover_node                     # STM32 chassis-dynamics
/sensors_node                   # STM32 sensors-gnss
/mission_command_node           # ws_base (if running)
/mission_monitoring_node        # ws_base (if running)
```

### Check Domain 6 Nodes (Vision Processing)

```bash
export ROS_DOMAIN_ID=6
ros2 node list

# Expected output (Jetson localhost only):
/camera_stream
/lane_detection
```

### Check Cross-Domain Communication

**From Jetson:**
```bash
# Check Domain 6 vision topics
export ROS_DOMAIN_ID=6
ros2 topic echo /tpc_rover_nav_lane

# Check Domain 5 control output
export ROS_DOMAIN_ID=5
ros2 topic echo /tpc_rover_fmctl
```

## Scalability

### Adding Vision/AI Nodes

New vision or AI nodes run on Domain 6 without impacting STM32 memory:

```bash
export ROS_DOMAIN_ID=6
ros2 run ai_perception object_detection_node
ros2 run ai_perception semantic_segmentation_node
```

These nodes remain invisible to the control loop and STM32 boards.

### Adding Control Nodes

New control nodes run on Domain 5 and participate in the control loop:

```bash
export ROS_DOMAIN_ID=5
ros2 run advanced_control ekf_fusion_node
```

These nodes are visible to all systems including STM32.

## Troubleshooting

### Vision Data Not Reaching Control Node

**Check Domain 6 is running:**
```bash
export ROS_DOMAIN_ID=6
ros2 topic list | grep nav_lane
```

**Check localhost DDS discovery:**
```bash
# Both domains should be visible on Jetson
export ROS_DOMAIN_ID=6
ros2 topic info /tpc_rover_nav_lane -v
```

### Control Commands Not Reaching Rover

**Check Domain 5 publisher:**
```bash
export ROS_DOMAIN_ID=5
ros2 topic info /tpc_rover_fmctl -v
# Should show 1 publisher (Jetson) and 1 subscriber (ws_rpi)
```

### STM32 Still Showing Memory Errors

**Check participant count:**
```bash
export ROS_DOMAIN_ID=5
ros2 node list | wc -l
# Should be 8-10 nodes (8 without ws_base, 10 with ws_base)
```

If still showing errors, increase STM32 config:
```cpp
// In platform/rtps/config.h
const uint8_t MAX_NUM_UNMATCHED_REMOTE_WRITERS = 40;
const uint8_t MAX_NUM_UNMATCHED_REMOTE_READERS = 50;
```

### Topics Not Visible (General)

**Verify domain ID matches:**
```bash
echo $ROS_DOMAIN_ID  # Should be 5 for control loop, 6 for vision
```

**Check network connectivity:**
```bash
ping 192.168.1.1  # RPi
ping 192.168.1.5  # Jetson
ping 192.168.1.2  # Chassis STM32
ping 192.168.1.6  # Sensors STM32
ping 192.168.1.10 # Base station
```

**Restart ROS2 daemon:**
```bash
ros2 daemon stop
ros2 daemon start
```

## See Also

- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture overview
- [TOPICS.md](TOPICS.md) - Topic reference
- `ws_jetson/README.md` - Jetson launch instructions
- `ws_rpi/README.md` - RPi documentation
