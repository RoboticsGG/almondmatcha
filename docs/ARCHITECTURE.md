# System Architecture

Almondmatcha rover system architecture: distributed heterogeneous computing with ROS2 DDS communication.

## Overview

**Design Philosophy:** Modular, distributed architecture with specialized computing nodes

**Key Principles:**
- Separation of concerns (vision, control, sensing)
- Real-time performance on resource-constrained embedded systems
- Centralized sensor fusion for autonomous capabilities
- Scalable network topology

## System Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     ROVER SYSTEM ARCHITECTURE                   │
│                    (Domain 5 - Unified System)                  │
│                All systems connected via Ethernet Switch        │
└─────────────────────────────────────────────────────────────────┘

                    Gigabit Ethernet Switch
                       (192.168.1.0/24)
                              |
        ┌─────────────┬───────┼───────┬─────────────┬─────────────┐
        |             |       |       |             |             |
   Raspberry Pi  Jetson Orin  |  Base Station  STM32 Chassis  STM32 Sensors
   192.168.1.1   192.168.1.5  |   192.168.1.10  192.168.1.2   192.168.1.6
        |             |       |       |             |             |
┌───────▼─────┐ ┌─────▼──────┐ ┌─────▼─────┐ ┌─────▼─────┐ ┌─────▼─────┐
│   ws_rpi    │ │ ws_jetson  │ │  ws_base  │ │  mros2    │ │  mros2    │
│             │ │            │ │           │ │           │ │           │
│ Coordination│ │  Vision    │ │ Telemetry │ │ Motor +   │ │ Sensors + │
│ Sensor      │ │  Lane      │ │ Command   │ │ IMU       │ │ GNSS +    │
│ Fusion      │ │  Detection │ │ Monitor   │ │ Control   │ │ Encoders  │
│ Mission     │ │  Steering  │ │           │ │           │ │ Power     │
└─────────────┘ └────────────┘ └───────────┘ └───────────┘ └───────────┘
     Domain 5       Domain 5      Domain 5      Domain 5      Domain 5
```

## Hardware Architecture

### Computing Nodes

| Node | Hardware | CPU | RAM | Storage | Network |
|------|----------|-----|-----|---------|---------|
| **Jetson** | Orin Nano 8GB | ARM Cortex-A78AE (6-core) + Ampere GPU | 8 GB | 128 GB eMMC | Gigabit Ethernet |
| **RPi** | Raspberry Pi 4B | ARM Cortex-A72 (4-core) | 4-8 GB | 64 GB SD | Gigabit Ethernet |
| **Chassis** | NUCLEO-F767ZI | ARM Cortex-M7 @ 216 MHz | 512 KB SRAM | 2 MB Flash | 100 Mbps Ethernet |
| **Sensors** | NUCLEO-F767ZI | ARM Cortex-M7 @ 216 MHz | 512 KB SRAM | 2 MB Flash | 100 Mbps Ethernet |

### Sensor Suite

**Vision:**
- Intel RealSense D415 RGB-D camera (1280×720 @ 30 FPS)
  - RGB stream for lane detection
  - Depth stream (reserved for obstacle avoidance)

**Navigation:**
- Sony Spresense GNSS (USB to RPi) - 10 Hz position updates
- SimpleRTK2b RTK GNSS (UART to STM32 sensors) - centimeter-level accuracy

**Motion Sensing:**
- LSM6DSV16X 6-axis IMU (I2C to STM32 chassis) - 100 Hz sampling
  - 3-axis accelerometer
  - 3-axis gyroscope
- Quadrature encoders on both drive motors - 10 Hz odometry

**Power Monitoring:**
- INA226 voltage/current sensor (I2C to STM32 sensors)

## Software Architecture

### ROS2 Multi-Domain Strategy

**Domain 5 (Control Network):** Network-wide, 10 participants
- All control systems: ws_rpi (7 nodes), ws_base (1 command node), ws_jetson (1 node), STM32 (2 nodes)
- Low-frequency control messages optimized for STM32 memory constraints
- Native action/service support across all systems

**Domain 4 (Telemetry):** Base + Jetson, 2 participants
- `mission_monitoring_node_pc` (Base): displays aggregated telemetry relay, no D5 participation
- `node_rover_local_monitoring` (Jetson): logs TelemetryRelay CSV at 5 Hz, future DB backend

**Domain 6 (Vision Processing):** Jetson localhost only, 2 participants
- camera_stream, lane_detection nodes
- High-bandwidth RGB/Depth streams (30 FPS, 1280×720) isolated from network
- Invisible to STM32 boards

**Cross-domain bridges:** `rover_kinematic_control` (D6→D5 for vision + kinematic control), `mission_monitoring_node_rpi` (D5→D4 for telemetry relay + CSV logging).

**Rationale:** Domain isolation reduces STM32 discovery overhead (10 vs 13+ participants), enables scalable vision expansion, and completely isolates monitoring traffic from the control network.

### Node Distribution

**Raspberry Pi 4 (192.168.1.1 — Domain 5):**
```
├── node_chassis_controller     - Motor command coordination (D5 sub/pub)
├── node_chassis_imu            - IMU data relay (D5 sub)
├── node_chassis_sensors        - Encoder/power relay (D5 sub)
├── node_gnss_spresense         - Standard GPS position processing (D5 pub)
├── node_gnss_ublox             - RTK GNSS centimeter-level processing (D5 pub)
├── node_gnss_mission_monitor   - Waypoint navigation state machine (D5)
└── mission_monitoring_node_rpi - Aggregates 10 D5 topics:
                                    Sub: D5 (all sensor/command topics)
                                    Pub: D4 /tpc_telemetry_relay (5 Hz)
                                    CSV: 6 per-topic files, native rates (4–50 Hz)
```

**Jetson Orin Nano (192.168.1.5 — Multi-Domain):**
```
Domain 6 (Vision Processing — localhost):
├── camera_stream       - D415 RGB/depth streaming @ 30 FPS
└── lane_detection      - Lane feature extraction @ 30 FPS

Domain 5 (Control Network):
└── rover_kinematic_control - Bicycle-model PID: steering + speed @ 50 Hz
    ├── Sub: tpc_rover_nav_lane (Domain 6)
    └── Pub: tpc_rover_ctrl_cmd [steer_angle, speed_cmd, detected] (Domain 5)

Domain 4 (Telemetry):
└── node_rover_local_monitoring  - Telemetry CSV logger
    ├── Sub: /tpc_telemetry_relay (Domain 4, 5 Hz)
    └── CSV: telemetry_unified + categorical files in ws_jetson/runs/
    (Future: DB backend replacing CSV)
```

**STM32 Chassis (192.168.1.2):**
```
Domain 5 (mROS2):
├── Motor Control Task - 50 ms @ High Priority
│   └── Subscribes: tpc_chassis_cmd
└── IMU Reader Task - 10 ms @ Normal Priority
    └── Publishes: tpc_chassis_imu @ 10 Hz
```

**STM32 Sensors (192.168.1.6):**
```
Domain 5 (mROS2):
├── Encoder Task - 100 ms
├── Power Monitor Task - 200 ms
├── GNSS Reader Task - 100 ms
└── Publishes: tpc_chassis_sensors @ 4 Hz (aggregated)
```

## Data Flow Architecture

### Vision-Based Lane Following

```
Camera (30 FPS) → Lane Detection (30 FPS) → Steering Control (50 Hz)
                                              ↓
                                        Chassis Controller (50 Hz)
                                              ↓
                                        STM32 Motor Control (20 Hz)
```

**Latency Budget:**
- Camera capture: 33 ms
- Lane detection: 30-40 ms
- Steering control: 20 ms (50 Hz cycle)
- Chassis command: 20 ms (50 Hz cycle)
- Motor actuation: 50 ms (20 Hz cycle)
- **Total end-to-end: 100-150 ms**

### Message Flow Patterns

**Publish-Subscribe:**
- Vision stream: camera → lane_detection
- Sensor data: STM32s → RPi logging nodes
- Control commands: steering_control → chassis_controller

**Request-Response (Services):** Speed limit updates — base station → chassis controller.

**Action (Goal-Based):** Destination waypoints — base station → mission monitor, with remaining-distance feedback.

**Request-Response (Services):**
- Speed limit updates: base station → chassis controller
- Navigation goals: base station → mission monitor

**Action (Goal-Based):**
- Destination waypoints: base station → mission monitor
- Feedback: remaining distance, ETA

## Communication Architecture

### Network Topology

```
                 Gigabit Ethernet Switch (192.168.1.0/24)
                              |
        ┌─────────────┬───────┼───────┬─────────────┬─────────────┐
        |             |       |       |             |             |
   RPi (.1)      Jetson (.5)  |  Base (.10)    Chassis (.2)  Sensors (.6)
```

| Device | IP Address | Role |
|--------|-----------|------|
| Raspberry Pi | 192.168.1.1 | Coordination / sensor fusion |
| Jetson Orin | 192.168.1.5 | Vision processing / kinematic control |
| Base Station | 192.168.1.10 | Command / monitoring |
| STM32 Chassis | 192.168.1.2 | Motor / IMU control |
| STM32 Sensors | 192.168.1.6 | GNSS / encoders / power |

**DDS:** Fast-RTPS on Linux; embeddedRTPS (mROS2) on STM32. Multicast discovery on 239.255.0.1, UDP 7400–7500. Critical commands use Reliable QoS; high-frequency sensor streams use Best-Effort.

## Storage Architecture

### Data Logging

**Primary — RPi** (`mission_monitoring_node_rpi`, ws_rpi/runs/run_NNN_YYYYMMDD_HHMMSS/):
- `rtk_gnss.csv` — RTK position, ~10 Hz
- `spresense_gnss.csv` — Spresense GPS, ~10 Hz
- `chassis_imu.csv` — Accel/gyro, ~10 Hz
- `chassis_sensors.csv` — Encoders, voltage, current, ~4 Hz
- `chassis_cmd.csv` — Motor commands, ~50 Hz
- `mission_state.csv` — Event-driven mission status

**Secondary — Jetson** (`node_rover_local_monitoring`, ws_jetson/runs/run_NNN_YYYYMMDD_HHMMSS/):
- `telemetry_unified.csv` — All fields, 5 Hz
- `rtk_gnss.csv`, `spresense_gnss.csv`, `chassis_data.csv`, `mission_state.csv` — 5 Hz
- Future: replaces CSV with SQLite/PostgreSQL backend

See [CSV_LOGGING.md](CSV_LOGGING.md) for full schema and analysis guidance.

### Configuration Storage

**YAML Configuration (Jetson):**
- `vision_nav_gui.yaml` / `vision_nav_headless.yaml`
- `steering_control_params.yaml`

**Hardcoded (STM32):**
- Network: IP, netmask, gateway in `mros2-platform.h`
- Sensors: Pins, I2C addresses in workspace apps

## Scalability

- Vision/AI nodes: add to Domain 6 — zero STM32 impact.
- Control nodes: add to Domain 5; monitor `ros2 node list | wc -l` against `MAX_NUM_PARTICIPANTS`.
- Additional computing boards: assign a static IP in 192.168.1.0/24, set `ROS_DOMAIN_ID=5`.

## Performance Characteristics

| Subsystem | Metric | Value |
|-----------|--------|-------|
| Vision | Processing latency | 30–40 ms |
| Vision | Frame rate | 30 FPS |
| Control | Steering update rate | 50 Hz |
| Control | Motor command latency | 20 ms |
| Sensors | IMU sample rate | 10 Hz |
| Sensors | GNSS update rate | 10 Hz |
| Network | End-to-end latency | <50 ms (Domain 5) |

---

**See Also:** [TOPICS.md](TOPICS.md) · [DOMAINS.md](DOMAINS.md)