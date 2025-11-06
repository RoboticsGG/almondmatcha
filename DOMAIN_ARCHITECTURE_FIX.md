# Domain Architecture Fix for Sensor Fusion

**Date:** November 6, 2025  
**Issue:** Incorrect domain separation preventing future EKF sensor fusion  
**Status:** ✅ Fixed

---

## Problem Identified

The previous domain consolidation (Nov 6, 2025 earlier) had an architectural flaw:

### Previous Architecture (INCORRECT for EKF):
```
Domain 2: ws_base + node_chassis_controller + GNSS nodes + ws_jetson vision
Domain 5: STM32 chassis + STM32 sensors + sensor processing nodes
```

**Critical Issue:** Sensor fusion (EKF) requires ALL sensor data on the SAME domain. The previous design split:
- **Vision data** (ws_jetson) → Domain 2
- **GNSS data** (node_gnss_*) → Domain 2  
- **Chassis controller** (node_chassis_controller) → Domain 2
- **IMU/Encoder data** (STM32s) → Domain 5

This made it impossible to implement EKF sensor fusion efficiently because sensors were on different domains.

---

## Corrected Architecture

### New Architecture (CORRECT for EKF):
```
Domain 2: ONLY ws_base ↔ rover bridge (future implementation)
Domain 5: ALL rover nodes (chassis, GNSS, vision, STM32s, sensors, future EKF)
```

### Domain 5 - Complete Rover Ecosystem:
- **STM32 Chassis Dynamics** (192.168.1.2)
  - Motor control
  - IMU sensor (LSM6DSV16X @ 100 Hz)
  
- **STM32 Sensors** (192.168.1.6)
  - Wheel encoders
  - Battery/power monitoring (INA226)
  - GNSS (SimpleRTK2b)
  
- **ws_rpi Nodes:**
  - `node_gnss_spresense` - GNSS position processing
  - `node_gnss_mission_monitor` - Mission waypoint tracking
  - `node_chassis_controller` - Motor command coordinator
  - `node_chassis_imu` - IMU data aggregation
  - `node_chassis_sensors` - Encoder/power data aggregation
  - **Future:** `node_ekf_fusion` - Extended Kalman Filter sensor fusion
  
- **ws_jetson Nodes:**
  - `camera_stream` - RealSense D415 RGB/depth
  - `lane_detection` - Lane parameter detection
  - `steering_control` - PID-based steering

### Domain 2 - Base Station Communication (Future):
- **Future Node:** `node_base_bridge` on ws_rpi
  - Relays telemetry from Domain 5 → Domain 2 (to ws_base)
  - Relays commands from Domain 2 → Domain 5 (from ws_base)
  - Selective topic forwarding (not all internal rover data)

---

## Why This Matters for EKF

Extended Kalman Filter (EKF) sensor fusion requires:

1. **IMU Data** (100 Hz) - Acceleration and gyroscope
2. **GNSS Data** (10 Hz) - Absolute position
3. **Wheel Encoders** (20 Hz) - Relative odometry
4. **Vision Odometry** (Optional, 25-30 Hz) - Visual odometry from camera

**EKF Algorithm Needs:**
- Real-time access to all sensor streams (low latency)
- Synchronized timestamps across all sensors
- No inter-domain message relay overhead

**With All Sensors on Domain 5:**
```
          ┌──────────────────────────────────────┐
          │     EKF Sensor Fusion (Domain 5)     │
          │                                      │
          │  State Estimate: [x, y, θ, vx, vy]  │
          └──────────────┬───────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
    IMU (100Hz)      GNSS (10Hz)    Encoders (20Hz)
    Domain 5         Domain 5         Domain 5
    
    ✅ All sensors accessible directly
    ✅ Low latency, no domain crossing
    ✅ Perfect for real-time fusion
```

**Previous Design Problem:**
```
          ┌──────────────────────────────────────┐
          │     EKF Sensor Fusion (Domain ??)    │
          │                                      │
          │  Where to run? Domain 2 or 5?        │
          └──────────────┬───────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
    IMU (100Hz)      GNSS (10Hz)    Vision (30Hz)
    Domain 5         Domain 2         Domain 2
    
    ❌ Sensors split across domains
    ❌ Requires domain bridge overhead
    ❌ Increased latency, complexity
```

---

## Changes Made

### 1. node_chassis_controller (ws_rpi)
**File:** `ws_rpi/src/pkg_chassis_control/src/node_chassis_controller.cpp`

**Changes:**
- Operates entirely in Domain 5 (was Domain 2)
- Publishes directly to `/tpc_chassis_cmd` (Domain 5)
- Removed Domain 2 publisher (`pub_rocon_d2_`)
- Updated comments to reflect rover-internal operation

**Rationale:** Chassis controller needs direct access to all rover sensors for coordination. Must be on Domain 5 for future EKF integration.

### 2. ws_jetson Vision Navigation
**Files:**
- `ws_jetson/vision_navigation/launch/vision_nav_gui.launch.py`
- `ws_jetson/vision_navigation/launch/vision_nav_headless.launch.py`

**Changes:**
- Changed `SetEnvironmentVariable('ROS_DOMAIN_ID', '2')` → `'5'`
- Updated comments to reflect sensor fusion purpose

**Rationale:** Vision data (lane detection, visual odometry) must be available to EKF on Domain 5.

### 3. rover_startup.launch.py
**File:** `ws_rpi/src/rover_launch_system/launch/rover_startup.launch.py`

**Changes:**
- Consolidated all nodes into single `domain_5_group`
- Removed `domain_2_group` entirely
- All 5 rover nodes now launch with `ROS_DOMAIN_ID=5`

**Nodes in Domain 5:**
1. node_gnss_spresense
2. node_gnss_mission_monitor
3. node_chassis_controller
4. node_chassis_imu
5. node_chassis_sensors

### 4. launch_rover_tmux.sh
**File:** `ws_rpi/launch_rover_tmux.sh`

**Changes:**
- All panes now set `export ROS_DOMAIN_ID=5`
- Updated pane titles to show "Domain 5"
- Reserved pane 6 labeled "Future: EKF Sensor Fusion (Domain 5)"

---

## Build Results

```bash
✅ pkg_chassis_control    [1min 6s]
✅ rover_launch_system    [12.6s]
```

No compilation errors, ready for deployment.

---

## Testing Plan

### 1. Verify Domain 5 Communication
```bash
# On Raspberry Pi
export ROS_DOMAIN_ID=5
ros2 topic list

# Should see ALL rover topics:
# /tpc_chassis_cmd          (chassis controller → STM32)
# /tpc_chassis_imu          (STM32 → chassis sensors)
# /tpc_chassis_sensors      (STM32 → chassis sensors)
# /tpc_gnss_spresense       (GNSS node)
# /tpc_gnss_mission_active  (mission monitor)
# /tpc_rover_fmctl          (vision → chassis)
# /tpc_rover_nav_lane       (lane detection → steering)
# /tpc_rover_d415_rgb       (camera → lane detection)
```

### 2. Test Vision → Chassis Flow
```bash
# Terminal 1: Monitor chassis commands (Domain 5)
export ROS_DOMAIN_ID=5
ros2 topic echo /tpc_chassis_cmd

# Terminal 2: Monitor vision output (Domain 5)
export ROS_DOMAIN_ID=5
ros2 topic echo /tpc_rover_fmctl

# Terminal 3: Launch vision system
cd ~/almondmatcha/ws_jetson
ros2 launch vision_navigation vision_nav_headless.launch.py

# Verify: Vision steering commands should flow to chassis controller
```

### 3. Verify STM32 Communication
```bash
# Domain 5: Should see STM32 data
export ROS_DOMAIN_ID=5
ros2 topic hz /tpc_chassis_imu      # Should see ~10 Hz
ros2 topic hz /tpc_chassis_sensors  # Should see ~10 Hz

# Domain 2: Should see nothing (no rover nodes there yet)
export ROS_DOMAIN_ID=2
ros2 topic list  # Should be empty (no bridge yet)
```

### 4. Test End-to-End Rover Operation
```bash
# Launch all rover nodes
cd ~/almondmatcha/ws_rpi
./launch_rover_tmux.sh

# Verify all 5 nodes running
# Check logs for "Domain 5" initialization messages
```

---

## Future Work: ws_base Bridge

To enable base station (Domain 2) telemetry without breaking sensor fusion:

### Create node_base_bridge.cpp
**Purpose:** Bridge Domain 5 ↔ Domain 2 for ws_base communication

**Architecture:**
```cpp
class BaseBridge {
private:
    // Domain 5 context (rover side)
    rclcpp::Node::SharedPtr rover_node_;
    
    // Domain 2 context (ws_base side)
    rclcpp::Node::SharedPtr base_node_;
    
public:
    // Subscribe to rover telemetry (Domain 5)
    // Publish to base station (Domain 2)
    void relayTelemetry() {
        // /tpc_gnss_spresense (D5) → /tpc_base_gnss (D2)
        // /tpc_chassis_sensors (D5) → /tpc_base_sensors (D2)
        // /tpc_chassis_imu (D5) → /tpc_base_imu (D2)
    }
    
    // Subscribe to base commands (Domain 2)
    // Forward to rover (Domain 5)
    void relayCommands() {
        // /tpc_base_mission_cmd (D2) → /tpc_rover_mission (D5)
        // /srv_base_spd_limit (D2) → /srv_rover_spd_limit (D5)
    }
};
```

**Selective Forwarding:**
- Only forward **telemetry** and **high-level commands**
- Do NOT forward internal sensor data (IMU, encoders, vision)
- Keeps Domain 2 bandwidth low for remote base station

---

## Benefits of This Architecture

### 1. **Enables EKF Sensor Fusion**
- All sensor data (IMU, GNSS, encoders, vision) on Domain 5
- EKF node can subscribe to all sensors without domain crossing
- Low latency, high-frequency fusion (100+ Hz)

### 2. **Clean Separation of Concerns**
- **Domain 5:** Internal rover operation (sensor fusion, control, navigation)
- **Domain 2:** External communication (base station telemetry/commands only)

### 3. **Scalability**
- Easy to add new sensors (lidar, cameras) - just add to Domain 5
- EKF automatically has access to new sensor data
- No need to reconfigure domain bridges

### 4. **Network Efficiency**
- Internal rover communication (high bandwidth) stays on Domain 5
- Only filtered telemetry crosses to Domain 2 (low bandwidth)
- Reduces network load for remote base station connection

### 5. **Development Workflow**
- Rover development and testing entirely on Domain 5
- Base station integration is separate concern
- Can test rover without base station running

---

## Domain Assignments - Final

| Domain | Purpose | Nodes | Bandwidth | Latency |
|--------|---------|-------|-----------|---------|
| **5** | Rover internal (sensor fusion) | All rover nodes (10+) | High (100+ MB/s) | Low (<10ms) |
| **2** | Base station (telemetry) | ws_base + future bridge | Low (1-10 MB/s) | Medium (50-200ms) |

---

## Verification Checklist

Before deploying to rover:

- [x] node_chassis_controller publishes to `/tpc_chassis_cmd` on Domain 5
- [x] ws_jetson launches all nodes with Domain 5
- [x] rover_startup.launch.py sets Domain 5 for all nodes
- [x] launch_rover_tmux.sh exports Domain 5 for all panes
- [x] All packages rebuild successfully
- [ ] STM32 boards receive commands from Domain 5
- [ ] Vision data flows to chassis controller
- [ ] All sensor topics visible on Domain 5
- [ ] No rover topics leak to Domain 2 (until bridge implemented)

---

## Migration Notes

**Breaking Change:** This is a breaking change from previous architecture.

**Required Actions:**
1. Flash STM32 boards (already on Domain 5, no change needed)
2. Deploy updated ws_rpi code to Raspberry Pi
3. Deploy updated ws_jetson code to Jetson
4. Update any manual launch scripts to use Domain 5
5. ws_base cannot communicate with rover until bridge node implemented

**Rollback:** To revert, checkout commit before this change:
```bash
git revert <this_commit_hash>
colcon build
```

---

**Fixed by:** GitHub Copilot  
**Reviewed by:** User  
**Deployed:** [Pending]  
**Next Step:** Test rover operation with all nodes on Domain 5, then implement node_base_bridge for ws_base telemetry
