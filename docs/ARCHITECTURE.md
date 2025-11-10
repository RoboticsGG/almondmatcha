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
└─────────────────────────────────────────────────────────────────┘

┌──────────────┐      ┌──────────────┐      ┌──────────────┐
│   Jetson     │      │ Raspberry Pi │      │  STM32 Boards│
│  Orin Nano   │      │      4B      │      │  (2 boards)  │
│              │      │              │      │              │
│ Vision       │◄────►│ Coordination │◄────►│ Real-time    │
│ Processing   │      │ Sensor Fusion│      │ Motor/Sensor │
│              │      │ Mission Ctrl │      │ Control      │
└──────────────┘      └──────┬───────┘      └──────────────┘
                             │
                             │ Direct DDS
                             │ (Domain 5)
                             │
                      ┌──────▼───────┐
                      │ Base Station │
                      │   (ws_base)  │
                      │  Domain 5    │
                      └──────────────┘
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

### ROS2 Domain Strategy

Two-domain architecture for sensor fusion:

**Domain 5 (Rover Internal):**
- All system processing (rover, base, vision, STM32)
- Direct low-latency communication
- Native action/service support
- Simplified architecture without relays
- Nodes: All systems on same domain

**Rationale:** Unified domain eliminates bridge overhead, enables native ROS2 features (actions/services), and simplifies system architecture.

### Node Distribution

**Raspberry Pi 4 (192.168.1.1 - Domain 5):**
```
├── node_chassis_controller - Motor command coordination
├── node_chassis_imu - IMU data logging
├── node_chassis_sensors - Encoder/power data logging
├── node_gnss_spresense - GPS position processing
├── node_gnss_mission_monitor - Waypoint navigation
└── [FUTURE] node_ekf_fusion - Multi-sensor fusion
```

**Jetson Orin Nano (192.168.1.5 - Domain 5):**
```
├── camera_stream_node - D415 RGB/depth streaming
├── lane_detection_node - Lane feature extraction
└── steering_control_node - PID steering control
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

### Sensor Fusion Data Flow (Future)

```
┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│ IMU (10 Hz) │  │GNSS (10 Hz) │  │Odom (10 Hz) │  │Lane (30 Hz) │
└──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘
       │                │                │                │
       └────────────────┴────────────────┴────────────────┘
                               ▼
                    ┌──────────────────────┐
                    │  EKF Sensor Fusion   │
                    │  node_ekf_fusion     │
                    │  (Raspberry Pi)      │
                    └──────────┬───────────┘
                               │
                    Fused State (10 Hz)
                    [x, y, θ, vx, vy, ωz]
                               │
       ┌───────────────────────┼───────────────────────┐
       │                       │                       │
       ▼                       ▼                       ▼
 Mission Control        Adaptive Steering      Fault Detection
```

### Message Flow Patterns

**Publish-Subscribe (Most Common):**
- Vision stream: camera → lane_detection
- Sensor data: STM32s → RPi logging nodes
- Control commands: steering_control → chassis_controller

**Request-Response (Services):**
- Speed limit updates: base station → chassis controller
- Navigation goals: base station → mission monitor

**Action (Goal-Based):**
- Destination waypoints: base station → mission monitor
- Feedback: remaining distance, ETA

## Communication Architecture

### Network Topology

```
         Ethernet Switch (192.168.1.0/24)
                    |
    ┌───────────────┼───────────────┬──────────────┐
    │               │               │              │
Jetson (.5)     RPi (.1)      Chassis (.2)   Sensors (.6)
```

**Physical Layer:**
- Gigabit Ethernet switch
- Cat5e/Cat6 cables
- Static IP addressing

**DDS Middleware:**
- Fast-RTPS (default ROS2 Humble implementation)
- Multicast discovery on LAN
- Reliable QoS for critical commands
- Best-effort QoS for high-frequency sensor streams

### QoS Profiles

| Topic Type | Reliability | History | Depth | Use Case |
|------------|-------------|---------|-------|----------|
| Camera streams | Best Effort | Keep Last | 1 | High-frequency vision data |
| Sensor data | Reliable | Keep Last | 10 | Critical state information |
| Commands | Reliable | Keep Last | 10 | Motor control commands |
| Logs | Best Effort | Keep Last | 100 | Debug/telemetry |

## Storage Architecture

### Data Logging

**Location:** `~/almondmatcha/runs/logs/` (centralized on RPi)

**CSV Logging Nodes:**
- `node_chassis_imu`: `chassis_imu_YYYYMMDD_HHMMSS.csv`
- `node_chassis_sensors`: `chassis_sensors_YYYYMMDD_HHMMSS.csv`
- `node_gnss_spresense`: `gnss_spresense_YYYYMMDD_HHMMSS.csv`
- `lane_detection_node`: `lane_pub_log.csv`
- `steering_control_node`: `rover_ctl_log_ver_3.csv`

**Rate:** 1 Hz (to minimize I/O overhead)

### Configuration Storage

**YAML Configuration (Jetson):**
- `vision_nav_gui.yaml` / `vision_nav_headless.yaml`
- `steering_control_params.yaml`

**Hardcoded (STM32):**
- Network: IP, netmask, gateway in `mros2-platform.h`
- Sensors: Pins, I2C addresses in workspace apps

## Scalability Considerations

### Adding New Sensors

1. If high-frequency (>10 Hz): Add task to STM32 firmware
2. If low-frequency (<10 Hz): Add ROS2 node to RPi
3. Update EKF fusion node to subscribe to new sensor topic

### Adding New Computing Nodes

1. Assign static IP in 192.168.1.0/24 range
2. Configure ROS_DOMAIN_ID=5
3. Build and deploy ROS2 packages
4. Update launch files

### Multi-Rover Systems

- Use different Domain IDs per rover (5, 8, 11, ...)
- Base station can monitor multiple domains
- Coordinate via higher-level planner

## Performance Characteristics

| Subsystem | Metric | Value |
|-----------|--------|-------|
| **Vision** | Processing latency | 30-40 ms |
| **Vision** | Frame rate | 30 FPS |
| **Control** | Steering update rate | 50 Hz |
| **Control** | Motor command latency | 20 ms |
| **Sensors** | IMU sample rate | 10 Hz |
| **Sensors** | GNSS update rate | 10 Hz |
| **Network** | End-to-end latency | <50 ms (Domain 5) |
| **Storage** | Log write rate | 1 Hz per node |

## Failure Modes & Recovery

**STM32 Communication Loss:**
- Rover stops receiving motor commands
- Motors timeout after 500 ms (safety feature)
- Manual recovery: reset STM32 board

**Network Partition:**
- Jetson vision isolated: rover operates in blind mode
- RPi isolated: full system failure (requires manual intervention)
- Base station isolated: rover continues autonomous operation

**Sensor Failures:**
- IMU failure: Fall back to GNSS + odometry only
- GNSS failure: Dead reckoning with IMU + odometry
- Camera failure: GPS waypoint navigation only

---

**See Also:**
- [TOPICS.md](TOPICS.md) - Complete topic reference
- [DOMAINS.md](DOMAINS.md) - Domain architecture details
