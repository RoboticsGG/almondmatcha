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

The rover uses **three DDS domains**: D6 (Jetson vision, localhost only), D5 (control network, all systems), D4 (telemetry relay, Base + Jetson only, never visible to STM32).

```mermaid
graph LR
    SW[["Gigabit Ethernet Switch\n192.168.1.0/24"]]

    subgraph RPi ["Raspberry Pi · 192.168.1.1"]
        R1["D5 · 8 nodes\nGNSS, chassis, mission, monitoring"]
        R2["D4 · mission_monitoring_node_rpi\ntelemetry relay + CSV"]
    end

    subgraph JET ["Jetson Orin · 192.168.1.5"]
        J1["D6 · camera_stream_node + lane_detection_node\nlocalhost only · 30 FPS"]
        J2["D5 · rover_kinematic_control\ndual-context: D6 sub / D5 pub"]
        J3["D4 · rover_local_monitoring_node\nCSV + future DB"]
    end

    subgraph BASE ["Base Station · 192.168.1.10"]
        B1["D5 · mission_command_node\nactions + services"]
        B2["D4 · mission_monitoring_node_pc\ntelemetry display"]
    end

    subgraph CHS ["STM32 Chassis · 192.168.1.2"]
        C1["D5 · chassis_controller\nmotor + IMU"]
    end

    subgraph SNS ["STM32 Sensors · 192.168.1.6"]
        S1["D5 · sensors_node\nGNSS + encoders + power"]
    end

    RPi --- SW
    JET --- SW
    BASE --- SW
    CHS --- SW
    SNS --- SW
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

**Domain 5 (Control Network):** Network-wide, 12 nodes
- ws_rpi (8) + ws_base (1) + ws_jetson (1) + STM32 (2) = 12 nodes
- Low-frequency control messages optimized for STM32 memory constraints
- Native action/service support across all systems

**Domain 4 (Telemetry):** Base + Jetson, 2 participants
- `mission_monitoring_node_pc` (Base): displays aggregated telemetry relay, no D5 participation
- `rover_local_monitoring_node` (Jetson): logs TelemetryRelay CSV at 5 Hz, future DB backend

**Domain 6 (Vision Processing):** Jetson localhost only, 2 participants
- camera_stream_node, lane_detection_node nodes
- High-bandwidth RGB/Depth streams (30 FPS, 1280×720) isolated from network
- Invisible to STM32 boards

**Dual-context nodes:** `rover_kinematic_control` subscribes D6 + publishes D5 in one process (no bridge); `mission_monitoring_node_rpi` subscribes D5 + publishes D4 in one process (no bridge).

**Rationale:** Domain isolation reduces STM32 discovery overhead (11 vs 14+ participants), enables scalable vision/AI expansion without STM32 firmware changes, and completely isolates monitoring/logging traffic from the control network.

### Node Distribution

**Raspberry Pi 4 (192.168.1.1 — Domain 5):**
```
├── chassis_controller_node     - Motor command coordination (D5 sub/pub)
├── chassis_imu_node            - IMU data relay (D5 sub)
├── chassis_sensors_node        - Encoder/power relay (D5 sub)
├── gnss_spresense_node         - Standard GPS position processing (D5 pub)
├── gnss_ublox_node             - RTK GNSS centimeter-level processing (D5 pub)
├── gnss_mission_monitor_node   - Waypoint navigation state machine (D5)
├── mission_monitoring_node_rpi - Telemetry bridge:
│                                   Sub: D5 (all sensor/command topics)
│                                   Pub: D4 /tpc_telemetry_relay (5 Hz)
│                                   CSV: 6 per-topic files, native rates (4–50 Hz)
└── rover_monitoring_node       - CSV logger (D5 sub, all topics)
```

**Jetson Orin Nano (192.168.1.5 — Multi-Domain):**
```
Domain 6 (Vision Processing — localhost):
├── camera_stream_node       - D415 RGB/depth streaming @ 30 FPS
└── lane_detection_node      - Lane feature extraction @ 30 FPS

Domain 5 (Control Network):
└── rover_kinematic_control - Bicycle-model PID: steering + speed @ 50 Hz
    ├── Sub: tpc_rover_nav_lane (Domain 6)
    └── Pub: tpc_rover_ctrl_cmd [steer_angle, speed_cmd, detected] (Domain 5)

Domain 4 (Telemetry):
└── rover_local_monitoring_node  - Telemetry CSV logger
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

```mermaid
flowchart LR
    CAM["camera_stream_node\nD6 · 30 FPS"] -->|tpc_rover_d415_rgb| LANE["lane_detection_node\nD6 · 30 FPS"]
    LANE -->|tpc_rover_nav_lane| CTRL["rover_kinematic_control\nD6 sub / D5 pub · 50 Hz"]
    CTRL -->|tpc_rover_ctrl_cmd| CC["chassis_controller_node\nD5 · 50 Hz"]
    CC -->|tpc_chassis_cmd| MTR["STM32 chassis_controller\nD5 · 20 Hz"]
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
- Vision stream: camera → lane_detection_node
- Sensor data: STM32s → RPi logging nodes
- Control commands: rover_kinematic_control → chassis_controller

**Request-Response (Services):**
- Speed limit updates: base station → chassis controller
- Navigation goals: base station → mission monitor

**Action (Goal-Based):**
- Destination waypoints: base station → mission monitor
- Feedback: remaining distance

## Communication Architecture

### Network Topology

| Device | IP | SSH | Domains | Role |
|--------|-----|-----|---------|------|
| Raspberry Pi | 192.168.1.1 | `curry@192.168.1.1` | D5 (8 nodes) + D4 (relay) | Coordination, sensing, mission |
| Jetson Orin | 192.168.1.5 | `yupi@192.168.1.5` | D6 (vision) + D5 (control) + D4 (logging) | Vision processing, kinematic control |
| Base Station | 192.168.1.10 | `yupi@192.168.1.10` | D5 (command) + D4 (display) | Mission command, telemetry monitoring |
| STM32 Chassis | 192.168.1.2 | — (mROS2) | D5 only | Motor control, IMU |
| STM32 Sensors | 192.168.1.6 | — (mROS2) | D5 only | GNSS, encoders, power |

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

**Secondary — Jetson** (`rover_local_monitoring_node`, ws_jetson/runs/run_NNN_YYYYMMDD_HHMMSS/):
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
- Control nodes: add to Domain 5; monitor `ros2 node list | wc -l` against SPDP_MAX in `platform/rtps/config.h`.
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