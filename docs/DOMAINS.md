# ROS2 Domain Architecture

The rover implements a tri-domain architecture: Domain 4 (telemetry), Domain 5 (control), Domain 6 (vision). The design minimizes STM32 DDS participant count while isolating high-bandwidth vision traffic and unidirectional monitoring traffic from the control network.

## Domain Assignment

| Domain | Purpose | Network Scope | Participants | Key Characteristics |
|--------|---------|---------------|--------------|---------------------|
| **4** | Telemetry | Base + Jetson | **2 nodes:**<br>• mission_monitoring_node_pc (Base)<br>• node_rover_local_monitoring (Jetson) | • Subscribes to /tpc_telemetry_relay (5 Hz)<br>• No D5 participation (no STM32 RAM cost)<br>• Jetson logs CSV; future DB backend |
| **5** | Control Network | All rover systems + base command | **11 nodes:**<br>• RPi: 7 nodes<br>• Base: 1 (mission_command_node)<br>• Jetson: 1 (rover_kinematic_control)<br>• STM32: 2 (chassis, sensors) | • Bidirectional command/control<br>• Action/service communication<br>• Real-time control loops (50 Hz)<br>• STM32 optimized (~60% free RAM) |
| **6** | Vision Processing | Jetson localhost | **2 nodes:**<br>• camera_stream<br>• lane_detection | • RGB/Depth streams (30 FPS, 1280×720)<br>• Network isolated (not visible to other hosts)<br>• Lane params relayed to D5 via steering_control |

## Architecture Overview

```
Domain 6: Vision Processing (Jetson localhost)
┌──────────────────────────────────────────┐
│ camera_stream → lane_detection          │
│ (30 FPS RGB/Depth, localhost only)       │
└────────────┤ tpc_rover_nav_lane ├──────────┘
                 │
                 ▼
Domain 5: Rover Control (Network-wide)
┌──────────────┤ Jetson ├────────────────────┐
│ rover_kinematic_control (D6→D5 bridge) │
│ Pub: tpc_rover_ctrl_cmd                 │
└──────────────┤ RPi ├───────────────────────┘
                 │
┌────────────────▼───────────────────────────┐
│ mission_monitoring_node_rpi             │
│ Aggregates: All D5 topics               │
│ Pub: tpc_telemetry_relay (to D4)       │
└────────────────┬───────────────────────────┘
                 │
                 │ Cross-domain relay
                 │
                 ▼
Domain 4: Base Telemetry (Base station)
┌─────────────────────────────────────────┐
│ mission_monitoring_node_pc              │
│ Sub: tpc_telemetry_relay (read-only)   │
└─────────────────────────────────────────┘

Domain 5: Base Command (Base station)
┌─────────────────────────────────────────┐
│ mission_command_node                    │
│ Pub: Commands/goals to RPi action servers│
└─────────────────────────────────────────┘
```

## Design Rationale

**Problem 1:** STM32 boards (512 KB SRAM) consume RAM per DDS participant for discovery and message tracking.  
**Solution 1:** Isolate vision traffic to Domain 6 (Jetson localhost), reducing D5 participants from 13 to 10.  
**Result:** ~60% free RAM on STM32 vs OOM without isolation.

**Problem 2:** Base station telemetry node adds an unnecessary D5 participant.  
**Solution 2:** Cross-domain relay — `mission_monitoring_node_rpi` aggregates D5 topics and publishes to D4 at 5 Hz.  
**Result:** D5 count at 10; monitoring traffic completely isolated from control domain.

**Problem 3:** Separate CSV logger (`node_rover_monitoring`) consumed a D5 participant slot.  
**Solution 3:** CSV logging merged into `mission_monitoring_node_rpi`; secondary Jetson logger added on D4.  
**Result:** One fewer D5 participant; dual-tier logging with no STM32 overhead.

**Cross-domain links:**
- Vision to control: `rover_kinematic_control` (D6 sub → D5 pub)
- Telemetry relay: `mission_monitoring_node_rpi` (D5 sub → D4 pub + CSV)

**Benefits:** 10 D5 participants; network bandwidth optimized; scalable vision/AI expansion without STM32 impact; Jetson logger ready for database migration.


## Domain Configuration

### Domain 5: Control Loop

**ws_rpi (Raspberry Pi):**
```bash
export ROS_DOMAIN_ID=5
cd ~/almondmatcha/ws_rpi
source install/setup.bash
./launch_rover_tmux.sh
```

**ws_base (Base Station - Dual Domain):**
```bash
cd ~/almondmatcha/ws_base
source install/setup.bash
# Script automatically handles both domains:
# - mission_command_node: Domain 5 (commands/actions to RPi)
# - mission_monitoring_node_pc: Domain 4 (telemetry display)
./launch_base_tmux.sh
```

**ws_jetson (Multi-Domain):**
```bash
cd ~/almondmatcha/ws_jetson
source install/setup.bash
# Script handles both Domain 6 (vision) and Domain 5 (control) automatically
./launch_headless.sh
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

# Expected output (11 nodes visible to all D5 systems):
/rover_kinematic_control        # ws_jetson (D6→D5 bridge)
/node_chassis_controller        # ws_rpi
/node_gnss_mission_monitor      # ws_rpi
/node_gnss_spresense            # ws_rpi
/node_gnss_ublox                # ws_rpi
/node_chassis_imu               # ws_rpi
/node_chassis_sensors           # ws_rpi
/mission_monitoring_node_rpi    # ws_rpi (D5 sub → D4 pub + CSV logging)
/chassis_controller             # STM32 chassis-dynamics
/sensors_node                   # STM32 sensors-gnss
/mission_command_node           # ws_base (commands/actions)

# Note: Domain 4 nodes not visible here
```

### Check Domain 4 Nodes (Telemetry)

```bash
export ROS_DOMAIN_ID=4
ros2 node list

# Expected output:
/mission_monitoring_node_pc      # ws_base (telemetry display, D4 subscriber)
/node_rover_local_monitoring     # ws_jetson (telemetry CSV logger, D4 subscriber)
/mission_monitoring_domain4_pub  # internal publisher from mission_monitoring_node_rpi (RPi)
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
ros2 topic echo /tpc_rover_ctrl_cmd
```

---

**See Also:** [ARCHITECTURE.md](ARCHITECTURE.md) · [TOPICS.md](TOPICS.md)
