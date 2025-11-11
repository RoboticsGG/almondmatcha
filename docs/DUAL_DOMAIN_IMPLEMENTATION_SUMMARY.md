# Multi-Domain Architecture Implementation

## Overview

Implementation of a multi-domain architecture to address STM32 memory constraints and improve system scalability.

**Date:** November 11, 2025

## Architecture

### Domain Assignment

| Domain | Purpose | Network Scope |
|--------|---------|---------------|
| 5 | Control loop: ws_rpi (5), ws_base (2), ws_jetson (1), STM32 (2) = 10 nodes | Network-wide |
| 6 | Vision processing: camera, lane detection = 2 nodes | Localhost only |

### Communication Flow

```
Domain 6 (Vision - Localhost)
  Camera → Lane Detection
        ↓ tpc_rover_nav_lane
        
Domain 5 (Control - Network)
  Steering Control (subscribes D6, publishes D5)
        ↓ tpc_rover_fmctl
  ws_rpi → STM32 Boards
```

### Benefits

- Reduced STM32 discovery overhead (10 participants optimized vs 12+ without isolation)
- Isolated high-bandwidth camera streams from control network
- Scalable vision/AI expansion without STM32 impact
- Native ROS2 cross-domain communication (no bridges)
- Optimized memory: ~180-200 KB used, 300+ KB free (60% headroom on 512 KB STM32)

## Implementation

### Files Created

1. `ws_jetson/vision_navigation/vision_navigation_pkg/steering_control_domain5.py`
   - Cross-domain control node (subscribes D6, publishes D5)
   - PID control, EMA filtering, CSV logging

2. `ws_jetson/vision_navigation/launch/vision_domain6.launch.py`
   - Launch file for Domain 6 vision processing

3. `ws_jetson/vision_navigation/launch/control_domain5.launch.py`
   - Launch file for Domain 5 control interface

### Files Modified

1. `ws_jetson/vision_navigation/setup.py`
   - Added `steering_control_domain5` entry point

2. `docs/DOMAINS.md`
   - Updated for multi-domain architecture
   - Removed historical context
   - Added cross-domain communication details

3. `ws_jetson/README.md`
   - Updated with multi-domain launch instructions
   - Simplified to essential information
   - Removed migration notes

### STM32 Configuration (Previously Completed)

1. Motor control fixes committed:
   - Servo PWM period: 20ms
   - Differential drive: RIGHT motor direction swapped
   - Direction codes: 1=forward, 2=backward, 0=stop

2. Memory limits configured:
   - `MAX_NUM_PARTICIPANTS=12` (10 actual + 2 headroom)
   - `MAX_NUM_UNMATCHED_REMOTE_WRITERS=30`
   - `MAX_NUM_UNMATCHED_REMOTE_READERS=40`
   - Memory usage: ~180-200 KB (60% free RAM)
   - See [STM32_MEMORY_CONFIG.md](STM32_MEMORY_CONFIG.md) for details

## Deployment

### Launch Sequence

**ws_jetson (Terminal 1 - Domain 6):**
```bash
export ROS_DOMAIN_ID=6
ros2 launch vision_navigation vision_domain6.launch.py
```

**ws_jetson (Terminal 2 - Domain 5):**
```bash
export ROS_DOMAIN_ID=5
ros2 launch vision_navigation control_domain5.launch.py
```

**ws_rpi:**
```bash
export ROS_DOMAIN_ID=5
./launch_rover_tmux.sh
```

**ws_base:**
```bash
export ROS_DOMAIN_ID=5
ros2 launch mission_control mission_control.launch.py
```

## Verification

**Check Domain 6 nodes:**
```bash
export ROS_DOMAIN_ID=6
ros2 node list  # Expected: /camera_stream, /lane_detection
ros2 topic hz /tpc_rover_nav_lane  # Expected: ~30 Hz
```

**Check Domain 5 nodes:**
```bash
export ROS_DOMAIN_ID=5
ros2 node list  # Expected: 8-10 nodes (without/with ws_base)
ros2 topic hz /tpc_rover_fmctl  # Expected: ~50 Hz
```

**Verify STM32 memory:**
- Serial console (115200 baud)
- Should NOT show `[MemoryPool]` errors
- Participant count: 10 (or 8 if ws_base not running)
- Free RAM: 300+ KB (60% headroom)

## Next Steps

1. Rebuild ws_jetson: `./build_clean.sh`
2. Test multi-domain launch
3. Verify cross-domain communication
4. Check STM32 memory usage via serial console

---

**Status:** Implementation complete, testing pending
