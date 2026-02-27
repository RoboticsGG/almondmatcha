# ROS2 Domain Architecture

The rover implements a tri-domain architecture: Domain 4 (telemetry), Domain 5 (control), Domain 6 (vision). The design minimizes STM32 DDS participant count while isolating high-bandwidth vision traffic and unidirectional monitoring traffic from the control network.

## Domain Assignment

| Domain | Purpose | Network Scope | Participants | Key Characteristics |
|--------|---------|---------------|--------------|---------------------|
| **4** | Telemetry | Base + Jetson | **2 nodes:**<br>• mission_monitoring_node_pc (Base)<br>• rover_local_monitoring_node (Jetson) | • Subscribes to /tpc_telemetry_relay (5 Hz)<br>• No D5 participation (no STM32 RAM cost)<br>• Jetson logs CSV; future DB backend |
| **5** | Control Network | All rover systems + base command | **11 nodes:**<br>• RPi: 7 nodes<br>• Base: 1 (mission_command_node)<br>• Jetson: 1 (rover_kinematic_control)<br>• STM32: 2 (chassis, sensors) | • Bidirectional command/control<br>• Action/service communication<br>• Real-time control loops (50 Hz)<br>• STM32 optimized (~60% free RAM) |
| **6** | Vision Processing | Jetson localhost | **2 nodes:**<br>• camera_stream<br>• lane_detection | • RGB/Depth streams (30 FPS, 1280×720)<br>• Network isolated (not visible to other hosts)<br>• Lane params relayed to D5 via steering_control |

## Architecture Overview

```mermaid
flowchart TB
    subgraph D6["Domain 6 — Jetson localhost only"]
        CAM["camera_stream"] -->|tpc_rover_d415_rgb| LANE["lane_detection"]
    end

    subgraph D5["Domain 5 — Control Network (all systems via Ethernet)"]
        CTRL["rover_kinematic_control (Jetson)\ndual-context: D6 sub / D5 pub\n→ tpc_rover_ctrl_cmd @ 50 Hz"]
        CC["chassis_controller_node (RPi)"]
        STM32C["chassis_controller (STM32)"]
        STM32S["sensors_node (STM32)"]
        GNSS["GNSS nodes (RPi)"]
        MON["mission_monitoring_node_rpi (RPi)\nAggregates all D5 topics\n→ CSV @ native rates"]
        CMD["mission_command_node (Base)\nactions/services → RPi"]
    end

    subgraph D4["Domain 4 — Telemetry (Base + Jetson, invisible to STM32)"]
        PC["mission_monitoring_node_pc (Base)\ntelemetry display"]
        JLOG["rover_local_monitoring_node (Jetson)\nCSV @ 5 Hz · future DB"]
    end

    LANE -->|tpc_rover_nav_lane| CTRL
    CTRL --> CC --> STM32C
    GNSS --> MON
    STM32C --> MON
    STM32S --> MON
    CTRL --> MON
    MON -->|tpc_telemetry_relay 5 Hz| PC
    MON -->|tpc_telemetry_relay 5 Hz| JLOG
```

## Design Rationale

**Problem 1:** STM32 boards (512 KB SRAM) consume RAM per DDS participant for discovery and message tracking.  
**Solution 1:** Isolate vision traffic to Domain 6 (Jetson localhost), reducing D5 participants from 13 to 11.  
**Result:** ~60% free RAM on STM32 vs OOM without isolation.

**Problem 2:** Base station telemetry node adds an unnecessary D5 participant.  
**Solution 2:** Cross-domain relay — `mission_monitoring_node_rpi` aggregates D5 topics and publishes to D4 at 5 Hz.  
**Result:** D5 count at 11; monitoring traffic completely isolated from control domain.

**Problem 3:** Separate CSV logger (`rover_monitoring_node`) consumed a D5 participant slot.  
**Solution 3:** CSV logging merged into `mission_monitoring_node_rpi`; secondary Jetson logger added on D4.  
**Result:** One fewer D5 participant; dual-tier logging with no STM32 overhead.

**Cross-domain links:**
- Vision to control: `rover_kinematic_control` (D6 sub → D5 pub, dual-context single process)
- Telemetry relay: `mission_monitoring_node_rpi` (D5 sub → D4 pub + CSV logging)

**Benefits:** 11 D5 participants; STM32 memory stable; scalable vision/AI expansion without STM32 firmware changes; Jetson logger ready for database migration.


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
/rover_kinematic_control        # ws_jetson (dual-context D6 sub / D5 pub)
/chassis_controller_node        # ws_rpi
/gnss_mission_monitor_node      # ws_rpi
/gnss_spresense_node            # ws_rpi
/gnss_ublox_node                # ws_rpi
/chassis_imu_node               # ws_rpi
/chassis_sensors_node           # ws_rpi
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
/rover_local_monitoring_node     # ws_jetson (telemetry CSV logger, D4 subscriber)
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

## Telemetry Relay Implementation

### Domain 4 Subscribers

Two nodes subscribe to `/tpc_telemetry_relay` on Domain 4:

**`mission_monitoring_node_pc` (Base station):** real-time telemetry display. Runs entirely in D4 — no D5 participation, never counted in STM32 discovery.

**`rover_local_monitoring_node` (Jetson):** secondary CSV logging at 5 Hz in `ws_jetson/runs/run_NNN_YYYYMMDD_HHMMSS/`. Future: replace CSV writer with SQLite/PostgreSQL. Runs entirely in D4.

### Dual-Context Pattern (RPi)

`mission_monitoring_node_rpi` creates two ROS2 contexts in one process:

```cpp
// Domain 5 context — for subscriptions (all sensor/command topics)
auto d5_init = rclcpp::InitOptions();
d5_init.set_domain_id(5);
auto d5_ctx = std::make_shared<rclcpp::Context>();
d5_ctx->init(argc, argv, d5_init);

// Domain 4 context — for publishing telemetry relay
auto d4_init = rclcpp::InitOptions();
d4_init.set_domain_id(4);
auto d4_ctx = std::make_shared<rclcpp::Context>();
d4_ctx->init(argc, argv, d4_init);

// MultiThreadedExecutor spins both nodes
rclcpp::executors::MultiThreadedExecutor exec;
exec.add_node(node);               // D5 subscriber node
exec.add_node(node->d4_pub_node_); // D4 publisher node
exec.spin();
```

### TelemetryRelay.msg

Published at 5 Hz on `/tpc_telemetry_relay` (Domain 4, ~280 bytes/message):
- Header (timestamp)
- Mission: `mission_active`, `distance_remaining_km`
- RTK GNSS: lat/lon/alt, fix quality, centimeter error, validity flag
- Spresense GNSS: lat/lon/alt, validity flag
- Chassis: encoders, voltage, current, power
- IMU: accel xyz, gyro xyz
- Commands: chassis cmd left/right speed & direction
- Navigation: `steering_command`, `lane_theta`, `lane_b`, `lane_detected`
- Destination: lat/lon

See [`common_ifaces/msgs_ifaces/msg/TelemetryRelay.msg`](../common_ifaces/msgs_ifaces/msg/) for full definition.

## Troubleshooting

**Base station sees no telemetry:**
1. `echo $ROS_DOMAIN_ID` on base — should be 4
2. RPi node running: `ROS_DOMAIN_ID=5 ros2 node list | grep mission_monitoring_node_rpi`
3. Network: `ping 192.168.1.1` from base

**RPi monitoring node missing from D5:**
1. Build: `colcon build --packages-select pkg_rover_monitoring`
2. Check executor running both contexts (no crash in D4 context init)
3. Verify ROS2 Humble+ (multi-domain context requires Humble or later)

**High CPU on RPi:**
Expected ~5–10% for 5 Hz D4 publishing. If higher, check for topic storms or QoS mismatches on D5 subscriptions.

---

**See Also:** [ARCHITECTURE.md](ARCHITECTURE.md) · [TOPICS.md](TOPICS.md)
