# Almondmatcha - Autonomous Mobile Rover System

Distributed ROS2-based control system for autonomous mobile rover with vision navigation, chassis control, and advanced sensor fusion (EKF). The system consists of heterogeneous computing nodes (Raspberry Pi, Jetson, STM32 microcontrollers) connected via Ethernet for real-time control and telemetry.

## System Architecture

### Overview

The rover system employs a distributed architecture with role-specific computing nodes and two-domain isolation:

- **ws_rpi (Raspberry Pi 4)**: Main on-board computer for rover coordination, sensor fusion (EKF), and mission control
- **ws_jetson (Jetson Orin Nano)**: Vision processing computer for lane detection and navigation
- **mros2-mbed-chassis-dynamics (STM32 Nucleo-F767ZI)**: Real-time motor control and IMU sensor reading
- **mros2-mbed-sensors-gnss (STM32 Nucleo-F767ZI)**: Encoder reading, power monitoring, and GNSS positioning
- **ws_base**: Ground station for telemetry monitoring and telecommand control

All nodes communicate via Ethernet using ROS2 DDS middleware with **two-domain isolation**:
- **Domain 2**: Base station ↔ Rover bridge (telemetry/telecommand only)
- **Domain 5**: All rover-internal nodes (sensor fusion, processing, STM32 communication)

### Network Topology

```
                    Ethernet Switch (192.168.1.0/24)
                              |
        +---------------------+---------------------+---------------------+
        |                     |                     |                     |
   Raspberry Pi          Jetson Orin          STM32 Chassis          STM32 Sensors
   192.168.1.1           192.168.1.5          192.168.1.2            192.168.1.6
   (Main Control)     (Vision Processing)     (Motor + IMU)      (Encoder + GNSS)
   ROS2 Domain 5        ROS2 Domain 5         ROS2 Domain 5         ROS2 Domain 5
        |                     |                     |                     |
        +---------------------+---------------------+---------------------+
                              |
                   [node_base_bridge]
                   Domain 2 Bridge to ws_base
                              |
                      Base Station PC
                      (Telemetry/Telecommand)
                      ROS2 Domain 2
                      ws_base
```

### Computing Nodes

| Node | Hardware | IP Address | Role | ROS2 Domain |
|------|----------|------------|------|-------------|
| **ws_rpi** | Raspberry Pi 4 | 192.168.1.1 | Main rover computer, EKF sensor fusion | 5 (+ bridge to 2) |
| **ws_jetson** | Jetson Orin Nano | 192.168.1.5 | Vision processing, lane detection | 5 |
| **chassis-dynamics** | STM32 Nucleo-F767ZI | 192.168.1.2 | Motor control, IMU sensor | 5 |
| **sensors-gnss** | STM32 Nucleo-F767ZI | 192.168.1.6 | Encoders, power, GNSS | 5 |
| **ws_base** | Ground Station PC | Variable | Telemetry/telecommand | 2 |

## Workspace Structure

```
almondmatcha/
├── common_ifaces/                           Shared interface definitions (SINGLE SOURCE)
│   ├── msgs_ifaces/
│   │   ├── msg/
│   │   │   ├── ChassisCtrl.msg              Chassis control commands
│   │   │   ├── ChassisIMU.msg               IMU sensor data
│   │   │   ├── ChassisSensors.msg           Encoder/battery data
│   │   │   └── SpresenseGNSS.msg            GNSS position data
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── action_ifaces/
│   │   ├── action/DesData.action            Navigation goal action
│   │   └── package.xml
│   │
│   ├── services_ifaces/
│   │   ├── srv/SpdLimit.srv                 Speed limit service
│   │   └── package.xml
│   │
│   └── README.md                            Interface documentation
│
├── ws_rpi/                                  Raspberry Pi 4 workspace
│   ├── src/
│   │   ├── pkg_chassis_control/
│   │   │   ├── src/
│   │   │   │   └── node_chassis_controller.cpp      Main control coordinator (Domain 2)
│   │   │   └── package.xml
│   │   │
│   │   ├── pkg_chassis_sensors/
│   │   │   ├── src/
│   │   │   │   ├── node_chassis_imu.cpp             IMU data processor (Domain 5)
│   │   │   │   └── node_chassis_sensors.cpp         Encoder & power aggregator (Domain 5)
│   │   │   └── package.xml
│   │   │
│   │   ├── pkg_gnss_navigation/
│   │   │   ├── src/
│   │   │   │   ├── node_gnss_spresense.cpp          GPS position reader (Domain 2)
│   │   │   │   └── node_gnss_mission_monitor.cpp    Mission planner & tracker (Domain 2)
│   │   │   └── package.xml
│   │   │
│   │   ├── rover_launch_system/
│   │   │   ├── launch/
│   │   │   │   └── rover_startup.launch.py          System-wide launcher
│   │   │   ├── setup.py
│   │   │   └── package.xml
│   │   │
│   │   ├── action_ifaces -> ../../common_ifaces/action_ifaces  (symlink)
│   │   ├── msgs_ifaces -> ../../common_ifaces/msgs_ifaces      (symlink)
│   │   └── services_ifaces -> ../../common_ifaces/services_ifaces  (symlink)
│   │
│   ├── build.sh                             Automated colcon build script
│   ├── launch_rover_tmux.sh                 Tmux monitoring launcher
│   ├── README.md                            Architecture and nodes documentation
│   └── BUILD.md                             Detailed build instructions
│
├── ws_jetson/                               Jetson Orin Nano workspace
│   └── vision_navigation/
│       ├── vision_navigation_pkg/
│       │   ├── camera_stream_node.py        D415 RGB/depth streaming (30 FPS)
│       │   ├── lane_detection_node.py       Lane marker detection and analysis
│       │   ├── steering_control_node.py     PID-based steering control (50 Hz)
│       │   ├── lane_detector.py             Lane detection pipeline (LAB, Sobel)
│       │   ├── control_filters.py           Control utilities (EMA filtering)
│       │   ├── helpers.py                   Reusable helper functions (50+)
│       │   ├── config.py                    Centralized config (6 classes)
│       │   └── __init__.py
│       ├── launch/
│       │   └── vision_navigation.launch.py  System launcher with Domain 2 setup
│       ├── setup.py                         Package configuration
│       ├── package.xml
│       ├── README.md                        Complete vision system documentation
│       └── resource/                        Package resources
│
├── ws_base/                                 Base station workspace (Ground PC)
│   └── src/
│       ├── mission_control/
│       │   ├── src/
│       │   │   ├── node_commands.cpp        Command dispatcher and mission planner
│       │   │   └── node_monitoring.cpp      Telemetry monitoring and status display
│       │   ├── config/
│       │   │   └── params.yaml              Mission parameters and settings
│       │   ├── launch/
│       │   │   └── node_comlaunch.py        Mission control launcher
│       │   ├── package.xml
│       │   └── README.md
│       │
│       ├── action_ifaces/
│       │   ├── action/
│       │   │   └── DesData.action           Shared destination action interface
│       │   └── package.xml
│       │
│       ├── action_ifaces -> ../../common_ifaces/action_ifaces  (symlink)
│       ├── msgs_ifaces -> ../../common_ifaces/msgs_ifaces      (symlink)
│       └── services_ifaces -> ../../common_ifaces/services_ifaces  (symlink)
│
├── mros2-mbed-chassis-dynamics/             STM32 chassis motor controller
│   ├── workspace/chassis_controller/
│   │   ├── app.cpp                          ROS2 node init + motor task (269 lines)
│   │   ├── motor_control.h                  Motor PWM/H-bridge control (243 lines)
│   │   ├── motor_control.cpp
│   │   ├── led_status.h                     LED indicator control (83 lines)
│   │   ├── led_status.cpp
│   │   └── README.md                        Task descriptions and usage
│   ├── platform/
│   │   ├── mros2-platform.h                 mROS2 platform abstraction
│   │   ├── mros2-platform.cpp
│   │   └── rtps/
│   │       └── config.h                     RTPS/DDS configuration
│   ├── mros2_add_msgs/
│   │   ├── mros2_header_generator/          Message header generation tools
│   │   │   ├── templates_generator.py
│   │   │   ├── header_generator.py
│   │   │   ├── msg_data_generator.py
│   │   │   └── msg_def_generator.py
│   │   └── mros2_msgs/                      Generated message headers
│   ├── libs/
│   │   └── X-Nucleo-IKS4A1_mbedOS/         Sensor driver libraries
│   ├── build.bash                           Docker-based build script
│   ├── mbed_app.json                        Mbed OS configuration
│   ├── LICENSE
│   └── README.md
│
├── mros2-mbed-sensors-gnss/                 STM32 sensors and GNSS node
│   ├── workspace/sensors_node/
│   │   ├── app.cpp                          3-task sensor integration node
│   │   ├── encoder_control.h                Encoder reader task
│   │   ├── encoder_control.cpp
│   │   ├── power_monitor.h                  Battery/power monitor task
│   │   ├── power_monitor.cpp
│   │   ├── gnss_reader.h                    GNSS position reader task
│   │   ├── gnss_reader.cpp
│   │   └── README.md                        Task descriptions and sensors
│   ├── platform/
│   │   ├── mros2-platform.h
│   │   ├── mros2-platform.cpp
│   │   └── rtps/
│   │       └── config.h
│   ├── mros2_add_msgs/
│   │   ├── mros2_header_generator/
│   │   └── mros2_msgs/                      Generated message headers
│   ├── build.bash
│   ├── mbed_app.json                        Mbed OS configuration
│   ├── LICENSE
│   └── README.md
│
├── copilot-session-note/                    Development session logs and notes
│   ├── SESSION_CONSOLIDATED_2025-11-04.md   Comprehensive development history
│   │                                         (all sessions Nov 1-4, critical info)
│   └── TOMORROW_ACTION_ITEMS.md             Next steps and future work
│
├── command/                                 Quick reference command files
│   ├── command.txt                          All manual launch commands (updated)
│   ├── cmd_rpi_launch.txt                   RPi rover startup sequence
│   ├── cmd_jetson_launch.txt                Jetson vision system launch
│   └── cmd_base_launch.txt                  Base station launch sequence
│
├── runs/                                    Data logging and ROS logs
│   ├── logs/                                CSV data files (ignored by git)
│   │   ├── chassis_imu_*.csv                IMU acceleration and rotation
│   │   ├── chassis_sensors_*.csv            Encoder and power data
│   │   └── gnss_spresense_*.csv             GPS position records
│   └── ros_logs/                            ROS2 node execution logs
│
└── README.md                                This file (system overview)
```

## System Components

### ws_rpi (Raspberry Pi 4) - Main Rover Computer

**Purpose:** Central rover coordination, extended Kalman filter (EKF) sensor fusion, mission planning, and base station bridging

**Running Nodes (Domain 5 - Rover Internal):**
- `node_chassis_controller` - Coordinates motor commands and cruise control (Domain 5)
- `node_chassis_imu` - Processes IMU sensor data (Domain 5)
- `node_chassis_sensors` - Aggregates encoder and power data (Domain 5)
- `node_gnss_spresense` - GNSS position processing (Domain 5)
- `node_gnss_mission_monitor` - Mission waypoint tracking and EKF integration (Domain 5)
- **[Future]** `node_ekf_fusion` - Extended Kalman Filter for multi-sensor fusion (Domain 5)

**Bridge Node (Domain 2 - Base Station Bridge):**
- `node_base_bridge` - Relays telemetry to ws_base and receives commands (Domain 2 ↔ Domain 5)

**Architecture:** All rover-internal processing on Domain 5 enables centralized sensor fusion with direct access to all sensor streams (IMU, GNSS, encoders, vision).

**Launch Command:**
```bash
cd ~/almondmatcha/ws_rpi
ros2 launch rover_launch_system rover_startup.launch.py
```

**Build:**
```bash
cd ~/almondmatcha/ws_rpi
bash build.sh
```

**Documentation:** `ws_rpi/README.md`, `ws_rpi/BUILD.md`

---

### ws_jetson (Jetson Orin Nano) - Vision Processing Computer

**Purpose:** Real-time vision processing for sensor fusion integration

**Running Nodes (Domain 5 - Rover Internal):**
- `camera_stream` - Intel RealSense D415 RGB/depth streaming (30 FPS @ 1280x720, GUI/headless modes)
- `lane_detection` - Lane marker detection and parameter calculation (theta, b)
- `steering_control` - PID-based steering control with EMA filtering (50 Hz)

**Architecture:** All vision nodes on Domain 5 enables direct integration with EKF sensor fusion node on ws_rpi.

**Launch Commands:**

GUI Mode (development with visualization):
```bash
cd ~/almondmatcha/ws_jetson
ros2 launch vision_navigation vision_nav_gui.launch.py
```

Headless Mode (production):
```bash
cd ~/almondmatcha/ws_jetson
ros2 launch vision_navigation vision_nav_headless.launch.py
```

**Key Features:**
- YAML-based configuration (best practice) with separate tuning files
  - `vision_nav_gui.yaml` / `vision_nav_headless.yaml` - System configuration (camera + lane detection)
  - `steering_control_params.yaml` - Steering control parameters (separate for easy tuning)
- GUI/headless mode switching via launch files
- 100% type hint coverage for all Python code
- 50+ reusable helper functions
- Proper initialization sequencing: Camera (0s) → Lane Detection (2s) → Control (3s)
- Two build scripts for development workflow:
  - `build_clean.sh` - Full clean rebuild (removes build/install/log artifacts)
  - `build_inc.sh` - Incremental build (preserves previous artifacts)

**Build:**
```bash
cd ~/almondmatcha/ws_jetson

# Clean build (fresh start, fixes environment issues)
./build_clean.sh

# Incremental build (faster for minor changes)
./build_inc.sh
```

**Documentation:** `ws_jetson/vision_navigation/README.md`

---

### mros2-mbed-chassis-dynamics (STM32 Nucleo-F767ZI) - Chassis Controller

**Purpose:** Real-time motor control and IMU sensor reading

**Hardware:**
- MCU: STM32F767ZI (Cortex-M7, 216 MHz)
- Flash: 385.7 KB
- RAM: 102 KB
- IP: 192.168.1.2
- Domain: 5

**Tasks:**
- **Task 1 (High Priority, 50 ms):** Motor Control
  - Subscribes to rover control commands
  - Calculates steering servo PWM and H-bridge signals
  - Responsive real-time control loop
  
- **Task 2 (Normal Priority, 10 ms):** IMU Reader
  - Polls LSM6DSV16X sensor at 100 Hz
  - Publishes aggregated IMU data at 10 Hz
  - Non-blocking I2C operations

**Subscribed Topics:**
- `tpc_chassis_cmd` (msgs_ifaces/ChassisCtrl) - Motor and steering commands

**Published Topics:**
- `tpc_chassis_imu` (msgs_ifaces/ChassisIMU) - IMU sensor data

**Build:**
```bash
cd ~/almondmatcha/mros2-mbed-chassis-dynamics
./build.bash all NUCLEO_F767ZI chassis_controller
```

**Flash:** Copy `build/mros2-mbed.bin` to NUCLEO board via USB mass storage

**Documentation:** `mros2-mbed-chassis-dynamics/workspace/chassis_controller/README.md`

---

### mros2-mbed-sensors-gnss (STM32 Nucleo-F767ZI) - Sensors Node

**Purpose:** Encoder reading, power monitoring, GNSS positioning

**Hardware:**
- MCU: STM32F767ZI (Cortex-M7, 216 MHz)
- Flash: 384.5 KB
- IP: 192.168.1.6
- Domain: 5

**Tasks:**
- **Task 1 (Normal Priority, 20 ms):** Encoder Reader
  - Polls quadrature encoders for wheel speed feedback
  
- **Task 2 (Normal Priority, 100 ms):** Power Monitor
  - Monitors INA226 power sensors
  - Measures battery voltage and current
  
- **Task 3 (Normal Priority, 100 ms):** GNSS Reader
  - Reads SimpleRTK2b GNSS module
  - Provides positioning data

**Published Topics:**
- `tpc_chassis_sensors` (msgs_ifaces/ChassisSensors) - Encoder and power data

**Build:**
```bash
cd ~/almondmatcha/mros2-mbed-sensors-gnss
./build.bash all NUCLEO_F767ZI sensors_node
```

**Flash:** Copy `build/mros2-mbed.bin` to NUCLEO board via USB mass storage

**Documentation:** `mros2-mbed-sensors-gnss/workspace/sensors_node/README.md`

---

### ws_base - Ground Station

**Purpose:** Telemetry monitoring and telecommand control from base station

**Features:**
- Mission planning and waypoint management
- Real-time telemetry visualization
- Remote command and control interface
- Data logging and analysis

**Build:**
```bash
cd ~/almondmatcha/ws_base
colcon build
source install/setup.bash
```

## Quick Start Guide

### Network Setup

Configure static IP addresses for all nodes on 192.168.1.0/24 network:

```bash
# On Raspberry Pi (192.168.1.1)
sudo ip addr add 192.168.1.1/24 dev eth0
sudo ip link set eth0 up

# On Jetson (192.168.1.5)
sudo ip addr add 192.168.1.5/24 dev eth0
sudo ip link set eth0 up

# Verify connectivity
ping 192.168.1.2  # Chassis controller
ping 192.168.1.6  # Sensors node
```

---

### Building the System

**1. Build STM32 Firmware (mROS2-mbed):**

```bash
# Chassis controller
cd ~/almondmatcha/mros2-mbed-chassis-dynamics
./build.bash all NUCLEO_F767ZI chassis_controller
# Output: build/mros2-mbed.bin

# Sensors node
cd ~/almondmatcha/mros2-mbed-sensors-gnss
./build.bash all NUCLEO_F767ZI sensors_node
# Output: build/mros2-mbed.bin
```

Flash .bin files to STM32 boards via USB mass storage or STLink.

**2. Build Raspberry Pi Workspace:**

```bash
cd ~/almondmatcha/ws_rpi
bash build.sh
source install/setup.bash
```

**3. Build Jetson Workspace:**

```bash
cd ~/almondmatcha/ws_jetson
colcon build --packages-select vision_navigation
source install/setup.bash
```

**4. Build Base Station:**

```bash
cd ~/almondmatcha/ws_base
colcon build
source install/setup.bash
```

---

### Running the Rover System

**1. Start STM32 Nodes:**
- Power on both STM32 Nucleo boards
- Verify Ethernet connectivity (LED indicators)
- Monitor serial output at 115200 baud

**2. Start Raspberry Pi Main Controller:**

```bash
cd ~/almondmatcha/ws_rpi
ros2 launch rover_launch_system rover_startup.launch.py
```

This launches all 6 rover control nodes with proper domain configuration.

**3. Start Jetson Vision Navigation:**

```bash
cd ~/almondmatcha/ws_jetson
ros2 launch vision_navigation vision_navigation.launch.py
```

Optional: Enable visualization for debugging:
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  camera_preview:=true lane_visualization:=true
```

**4. Monitor from Base Station:**

```bash
cd ~/almondmatcha/ws_base
# Launch telemetry monitoring
ros2 run mission_control telemetry_monitor
```

---

### Testing and Verification

**Check Active Nodes:**
```bash
ros2 node list
```

**Monitor Topics:**
```bash
# List all topics
ros2 topic list

# Monitor camera stream
ros2 topic hz tpc_rover_d415_rgb

# Monitor lane detection
ros2 topic echo tpc_rover_nav_lane

# Monitor steering commands
ros2 topic echo tpc_rover_fmctl

# Monitor IMU data
ros2 topic echo tp_imu_data_d5

# Monitor sensor data
ros2 topic echo tpc_chassis_sensors
```

**Manual Motor Control Test:**
```bash
# Set Domain ID for chassis controller
export ROS_DOMAIN_ID=5

# Send motor command (speed: 100, steering: 0.5, forward)
ros2 topic pub tpc_chassis_cmd msgs_ifaces/msg/ChassisCtrl \
  '{fdr_msg: 2, ro_ctrl_msg: 0.5, spd_msg: 100, bdr_msg: 1}'
```

**Vision System Test:**
```bash
# Check camera stream availability
ros2 topic hz tpc_rover_d415_rgb

# Monitor lane detection output
ros2 topic echo tpc_rover_nav_lane

# Verify steering control is publishing
ros2 topic hz tpc_rover_fmctl
```

## ROS2 Domain Architecture

The system uses two-domain isolation to enable centralized sensor fusion:

| Domain ID | Purpose | Nodes | Use Case |
|-----------|---------|-------|----------|
| **2** | Base station bridge only | ws_base ↔ node_base_bridge | Telemetry/telecommand relay between ground station and rover |
| **5** | All rover-internal processing | All STM32s, all ROS2 rover nodes | Sensor fusion (EKF), centralized control, low-latency data access |

**Architecture Benefits:**
- **Domain 5 (Rover):** Direct inter-node communication for EKF sensor fusion (IMU → GNSS → encoders → vision)
- **Domain 2 (Bridge):** Separate domain isolates base station from rover internals, improves scalability
- **node_base_bridge:** Single bridge node translates between domains, handles telemetry aggregation

**Future Sensor Fusion (EKF):**
- Central `node_ekf_fusion` on Domain 5 will subscribe to all sensor streams
- Produces fused state estimate: `[position, velocity, orientation, biases]`
- All rover control nodes will consume fused estimates for coordinated decision-making

**Note:** Refactored from 3 domains (2, 5, 6) to 2 domains (2, 5) on November 6, 2025. See `DOMAIN_CONSOLIDATION_SUMMARY.md` for architectural evolution details.

---

## Topic Pub/Sub Architecture

### Complete Message Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                        AUTONOMOUS MOBILE ROVER SYSTEM                               │
│              Two-Domain Architecture for Sensor Fusion (EKF)                         │
└─────────────────────────────────────────────────────────────────────────────────────┘

                           ╔═══════════════════════════════════╗
                           ║     GROUND STATION (ws_base)      ║
                           ║     Domain 2 Only                 ║
                           ╚═══════════════════════════════════╝
                                      ▲         ▼
                              Telemetry / Commands
                                      │         │
                              [node_base_bridge]
                              (D2 ↔ D5 Bridge)
                                      │         │
                    ┌─────────────────┴─────────┴─────────────────┐
                    │                                             │
                    │            DOMAIN 5 (Rover)                │
                    │        All Rover-Internal Nodes            │
                    │                                             │
                    │  ┌─────────────────────────────────────┐   │
                    │  │   SENSOR DATA STREAMS               │   │
                    │  ├─────────────────────────────────────┤   │
                    │  │ • STM32 Chassis: IMU @ 10 Hz        │   │
                    │  │ • STM32 Sensors: Encoders @ 10 Hz   │   │
                    │  │ • Jetson: Vision/Lane @ 30 FPS      │   │
                    │  │ • RPi: GNSS Position @ 10 Hz        │   │
                    │  └─────────────────────────────────────┘   │
                    │                     │                       │
                    │                     ▼                       │
                    │  ┌─────────────────────────────────────┐   │
                    │  │   EKF SENSOR FUSION (Future)        │   │
                    │  │   node_ekf_fusion (Domain 5)        │   │
                    │  │                                     │   │
                    │  │  Fused State: [x, y, θ, vx, vy]   │   │
                    │  └─────────────────────────────────────┘   │
                    │                     │                       │
                    │                     ▼                       │
                    │  ┌─────────────────────────────────────┐   │
                    │  │   ROVER CONTROL NODES               │   │
                    │  │ • Chassis Controller                │   │
                    │  │ • Mission Monitor                   │   │
                    │  │ • Vision Navigation                 │   │
                    │  └─────────────────────────────────────┘   │
                    │                                             │
                    └─────────────────────────────────────────────┘
```

### Topic Subscription/Publication Table

| Node | Domain | Publishes | Subscribes | Message Type | Rate/QoS |
|------|--------|-----------|-----------|--------------|----------|
| **camera_stream** (ws_jetson) | 5 | `tpc_rover_d415_rgb` | - | sensor_msgs/Image | 30 FPS (BEST_EFFORT) |
| | | `tpc_rover_d415_depth` | - | sensor_msgs/Image | 30 FPS (BEST_EFFORT) |
| **lane_detection** (ws_jetson) | 5 | `tpc_rover_nav_lane` | `tpc_rover_d415_rgb` | Float32MultiArray | 25-30 FPS |
| **steering_control** (ws_jetson) | 5 | `tpc_rover_fmctl` | `tpc_rover_nav_lane` | Float32MultiArray | 50 Hz |
| **node_chassis_controller** (ws_rpi) | 5 | `tpc_chassis_cmd` | `tpc_rover_fmctl` | ChassisCtrl | 50 Hz |
| | | | `tpc_gnss_mission_active` | | |
| **node_gnss_spresense** (ws_rpi) | 5 | `tpc_gnss_spresense` | - | SpresenseGNSS | 10 Hz |
| **node_gnss_mission_monitor** (ws_rpi) | 5 | `tpc_gnss_mission_active` | `tpc_rover_dest_coordinate` | Bool | 10 Hz |
| | | `tpc_gnss_mission_remain_dist` | `tpc_gnss_spresense` | Float64 | |
| | | `tpc_rover_dest_coordinate` | Service: desigation | Float64MultiArray | |
| **[Future] node_ekf_fusion** (ws_rpi) | 5 | `tpc_ekf_state_estimate` | `tpc_chassis_imu`, `tpc_gnss_spresense`, `tpc_chassis_sensors`, `tpc_rover_nav_lane` | State (x,y,θ,vx,vy) | 10 Hz |
| **chassis_controller** (mros2-mbed-chassis-dynamics) | 5 | `tpc_chassis_imu` | `tpc_chassis_cmd` | ChassisIMU | 10 Hz |
| **sensors_node** (mros2-mbed-sensors-gnss) | 5 | `tpc_chassis_sensors` | - | ChassisSensors | 10 Hz |
| **node_chassis_imu** (ws_rpi) | 5 | (logs to CSV) | `tpc_chassis_imu` | - | CSV write @ 1 Hz |
| **node_chassis_sensors** (ws_rpi) | 5 | (logs to CSV) | `tpc_chassis_sensors` | - | CSV write @ 1 Hz |
| **node_base_bridge** (ws_rpi) | 2↔5 | `tpc_telemetry` (D2) | `tpc_gnss_spresense`, `tpc_chassis_sensors` (D5) | Custom | 10 Hz |
| | | | `tpc_command` (D2) | | |
| **mission_monitoring_node** (ws_base) | 2 | - | `tpc_telemetry` | Custom | Telemetry only |
| | | | `tpc_command` (Service) | | |

### Data Flow Sequences

#### Vision-Based Lane Following (Domain 5 - Rover Internal)

```
1. camera_stream (ws_jetson/Domain 5)
   └─▶ Publishes RGB frames: tpc_rover_d415_rgb (30 FPS)
       
2. lane_detection (ws_jetson/Domain 5)
   ├─ Subscribes: tpc_rover_d415_rgb
   └─▶ Publishes: tpc_rover_nav_lane [theta, b, detected] (25-30 FPS)
       
3. steering_control (ws_jetson/Domain 5)
   ├─ Subscribes: tpc_rover_nav_lane
   ├─ Applies PID controller (Kp, Ki, Kd)
   └─▶ Publishes: tpc_rover_fmctl [steering_angle, detected] (50 Hz)
       
4. node_chassis_controller (ws_rpi/Domain 5)
   ├─ Subscribes: tpc_rover_fmctl
   ├─ Provides cruise control logic
   └─▶ Publishes: tpc_chassis_cmd (50 Hz)
       
5. chassis_controller (STM32/Domain 5)
   ├─ Subscribes: tpc_chassis_cmd
   └─▶ Executes motor control + publishes IMU: tpc_chassis_imu (10 Hz)
```

#### GNSS-Based Navigation (Domain 5 - Rover Internal)

```
1. node_gnss_spresense (ws_rpi/Domain 5)
   ├─ Reads: Sony Spresense GNSS via serial
   └─▶ Publishes: tpc_gnss_spresense (lat, long, accuracy) (10 Hz)
       
2. node_gnss_mission_monitor (ws_rpi/Domain 5)
   ├─ Subscribes: tpc_gnss_spresense
   ├─ Listens for: tpc_rover_dest_coordinate (target waypoint)
   ├─ Calculates: Distance to waypoint, mission progress
   └─▶ Publishes:
       • tpc_gnss_mission_active (Bool): Mission status
       • tpc_gnss_mission_remain_dist (Float64): Distance remaining (km)
       
3. node_chassis_controller (ws_rpi/Domain 5)
   ├─ Subscribes: tpc_gnss_mission_active
   └─▶ Enables/disables cruise control based on mission state
```

#### Sensor Data Acquisition (Domain 5 - Rover Internal)

```
1. chassis_controller (STM32-Dynamics/Domain 5)
   ├─ Polls: LSM6DSV16X IMU sensor @ 100 Hz
   ├─ Publishes every 10 samples (100 ms interval)
   └─▶ Topic: tpc_chassis_imu (accel_x,y,z + gyro_x,y,z) (10 Hz)
   
2. sensors_node (STM32-Sensors/Domain 5)
   ├─ Polls: Motor encoders + INA226 power monitor
   └─▶ Topic: tpc_chassis_sensors (encoder + battery data) (10 Hz)
   
3. node_chassis_imu (ws_rpi/Domain 5)
   ├─ Subscribes: tpc_chassis_imu (from STM32)
   └─▶ Logs to CSV file (no republish)
   
4. node_chassis_sensors (ws_rpi/Domain 5)
   ├─ Subscribes: tpc_chassis_sensors (from STM32)
   └─▶ Logs to CSV file (no republish)
   
5. [FUTURE] node_ekf_fusion (ws_rpi/Domain 5) - Centralized Sensor Fusion
   ├─ Subscribes to all sensors:
   │   • tpc_chassis_imu (raw accelerometer, gyroscope from STM32)
   │   • tpc_gnss_spresense (GPS position, fix quality)
   │   • tpc_chassis_sensors (raw wheel odometry from STM32)
   │   • tpc_rover_nav_lane (vision-based lane offset)
   ├─ Implements: Extended Kalman Filter (EKF)
   └─▶ Publishes: tpc_ekf_state_estimate [x, y, θ, vx, vy, biases] (10 Hz)
```

#### Base Station Bridge (Domain 2 ↔ Domain 5 Bridge)

```
1. node_base_bridge (ws_rpi - Multi-domain node)
   ├─ Domain 5 (Rover) Subscriptions:
   │   • tpc_chassis_imu (raw sensor data)
   │   • tpc_gnss_spresense (GNSS position)
   │   • tpc_chassis_sensors (raw odometry data)
   │   • tpc_ekf_state_estimate (fusion result, when available)
   ├─ Domain 2 (Base) Publications:
   │   • tpc_telemetry (aggregated sensor telemetry)
   ├─ Domain 2 (Base) Subscriptions:
   │   • tpc_command (rover control commands)
   └─▶ Domain 5 (Rover) Publications:
       • Motor commands to Domain 5 control nodes
```

---

## Key Technologies

- **ROS2 Humble**: Distributed middleware with DDS communication
- **mROS2**: Embedded ROS2 implementation for STM32 microcontrollers
- **Mbed OS 6.x**: Real-time operating system for STM32 boards
- **Intel RealSense D415**: RGB-D camera for vision navigation
- **OpenCV**: Computer vision processing
- **Python 3.10+**: High-level control and vision processing (100% type hints)
- **C++17**: Real-time embedded control on STM32

---

## Configuration Management

### Vision Navigation (ws_jetson) - YAML Configuration

Configuration follows ROS2 best practices with YAML files instead of config.py.

**Configuration Files Structure:**
- `vision_nav_gui.yaml` - System configuration for GUI mode
  - `camera_stream`: open_cam=true (display RGB/depth windows)
  - `lane_detection`: show_window=true (display lane detection visualization)
  
- `vision_nav_headless.yaml` - System configuration for headless mode
  - `camera_stream`: open_cam=false (no GUI windows)
  - `lane_detection`: show_window=false (no visualization)
  
- `steering_control_params.yaml` - Steering control tuning parameters (independent file)
  - Contains all PID gains (k_p, k_i, k_d)
  - Error weighting parameters (k_e1, k_e2)
  - EMA filtering coefficient (ema_alpha)
  - Steering limits and safety parameters

**How They Work Together:**
- System configuration files (vision_nav_gui/headless.yaml) load first
- Steering control parameters file loads second, overriding/adding to system config
- No parameter duplication: each parameter lives in exactly one file
- Steering control kept separate for easy tuning during development without changing system config

**Example: Tuning Steering Control**
```bash
cd ~/almondmatcha/ws_jetson/vision_navigation/config

# Edit steering_control_params.yaml
nano steering_control_params.yaml
# Change: k_p: 4.5  (adjust as needed)

# Launch with updated parameters (no rebuild needed)
cd ~/almondmatcha/ws_jetson
ros2 launch vision_navigation vision_nav_gui.launch.py
```

**Example: Creating Custom Configuration**
```bash
cd ~/almondmatcha/ws_jetson/vision_navigation/config

# Copy GUI configuration as template
cp vision_nav_gui.yaml my_custom_config.yaml

# Edit for your specific needs
nano my_custom_config.yaml

# Launch with custom config (modify launch file to use my_custom_config.yaml)
cd ~/almondmatcha/ws_jetson
ros2 launch vision_navigation vision_nav_gui.launch.py
```

**Launching with Parameter Overrides:**
```bash
# Override steering gains at runtime
ros2 launch vision_navigation vision_nav_gui.launch.py \
  k_p:=5.0 k_i:=0.15 k_d:=0.2
```

### STM32 Embedded Configuration

Hardware parameters configured in respective workspace directories:
- `mros2-mbed-chassis-dynamics/workspace/chassis_controller/`
- `mros2-mbed-sensors-gnss/workspace/sensors_node/`

Network configuration (IP addresses, domain IDs) set in platform layer.

---

## Data Logging

### Vision Navigation Logs

**Lane Detection:** `lane_pub_log.csv`
```
timestamp,theta,b,detected
2025-11-04T10:30:45.123456,5.23,45.67,1.0
```

**Steering Control:** `logs/rover_ctl_log_ver_3.csv`
```
time_sec,theta_ema,b_ema,u,e_sum
1730708000.123,4.85,44.25,19.4,5.12
```

### Sign Conventions

- **Steering Angle:** Positive = RIGHT, Negative = LEFT (degrees)
- **Heading Error (theta):** Positive = lane RIGHT (turn right), Negative = lane LEFT (turn left)
- **Lateral Offset (b):** Positive = camera RIGHT of center, Negative = camera LEFT of center (pixels)

---

## Troubleshooting

### STM32 Nodes

**Problem:** Board not visible on network
- Check Ethernet cable and switch
- Verify IP configuration (192.168.1.2 or 192.168.1.6)
- Monitor serial output for initialization messages
- Check LED1: OFF during init, ON when ready

**Problem:** "logging was initialized more than once" warning
- This is expected for multi-domain architecture
- Not a bug, safe to ignore

### Vision Navigation

**Problem:** D415 camera not detected
- Check USB connection
- Verify device: `rs-enumerate-devices`
- Install librealsense: `sudo apt install librealsense2`
- Check serial matches: 806312060441

**Problem:** Lane detection not working
- Enable visualization: `lane_visualization:=true`
- Check lighting conditions
- Verify lane markers are visible and contrasting
- Adjust color thresholds in config.py

**Problem:** High CPU usage
- Reduce frame rate: `fps:=15`
- Reduce resolution: `width:=640 height:=480`
- Disable visualization: `lane_visualization:=false`

### Network Communication

**Problem:** Topics not visible across nodes
- Verify ROS_DOMAIN_ID matches target domain
- Check network connectivity: `ping <node_ip>`
- Use `ros2 topic list` with correct domain ID
- Verify firewall allows DDS traffic (UDP ports)

---

## Development Workflow

### Git Workflow

```bash
# Check status
git status

# Stage changes
git add <files>

# Commit with descriptive message
git commit -m "category: brief description

- Detailed change 1
- Detailed change 2"

# Push to remote
git push
```

### Testing Checklist

**STM32 Nodes:**
- [ ] Build completes without errors
- [ ] LED1 OFF during startup (first ~1 second)
- [ ] LED1 ON solid after initialization
- [ ] Serial console shows all tasks starting
- [ ] Motor responds to ROS2 commands
- [ ] IMU data publishes at expected rate
- [ ] Network connectivity verified

**Vision Navigation:**
- [ ] Camera stream at 30 FPS
- [ ] Lane detection running without errors
- [ ] Steering commands publishing
- [ ] End-to-end latency < 150 ms

**Overall System:**
- [ ] All nodes visible in `ros2 node list`
- [ ] All expected topics active
- [ ] No error messages in logs
- [ ] Network latency acceptable

---

## Documentation

Comprehensive documentation available in workspace-specific README files:

- **ws_rpi/README.md** - Raspberry Pi workspace architecture and nodes
- **ws_rpi/BUILD.md** - Detailed build instructions and troubleshooting
- **ws_jetson/vision_navigation/README.md** - Vision navigation system complete guide
- **mros2-mbed-chassis-dynamics/workspace/chassis_controller/README.md** - Chassis controller architecture
- **mros2-mbed-sensors-gnss/workspace/sensors_node/README.md** - Sensors node documentation
- **copilot-session-note/SESSION_CONSOLIDATED_2025-11-04.md** - Development session log with critical information

---

## Recent Updates

### November 6, 2025 (Continued) - Sensor Fusion Architecture Refactoring
- **Critical Architecture Fix:** Refactored from mixed-domain to centralized sensor fusion architecture
  - **Domain 5 (Rover Internal):** ALL rover nodes - chassis, GNSS, vision, sensor fusion
  - **Domain 2 (Bridge Only):** ONLY base station bridge (ws_base communication)
  - Removed node_chassis_controller from Domain 2, moved to Domain 5
  - Moved GNSS nodes (node_gnss_spresense, node_gnss_mission_monitor) from Domain 2 → Domain 5
  - Moved ws_jetson vision nodes (camera_stream, lane_detection, steering_control) from Domain 2 → Domain 5
- **New Bridge Node:** Created node_base_bridge for Domain 2 ↔ Domain 5 communication
  - Single bridge node simplifies telemetry/telecommand relay to ws_base
  - All rover-internal processing isolated in Domain 5 for low-latency data access
- **EKF Sensor Fusion Preparation:** Architecture now supports centralized Extended Kalman Filter
  - All sensor streams available on same domain (IMU, GNSS, encoders, vision)
  - Future node_ekf_fusion node will fuse all measurements into single state estimate
  - Enable advanced autonomous capabilities: adaptive control, fault detection, state prediction
- **Updated Documentation:**
  - Revised network topology diagram showing sensor fusion integration
  - Updated ROS2 domain architecture section with rationale
  - Added data flow diagram showing EKF integration points
  - Updated topic subscription/publication table with sensor fusion nodes
  - Added base station bridge documentation

### November 6, 2025 - Domain Architecture Consolidation (Part 1)
- **Major Refactoring:** Consolidated rover system from 3 domains (2, 5, 6) to 2 domains (2, 5)
  - Removed unnecessary Domain 6 separation for STM32 sensors board
  - Both STM32 boards now communicate on Domain 5 (main rover domain)
  - Eliminated node_domain_bridge complexity (completely removed)
- **Root Cause Analysis:** mROS2 limitation is per-board participant count (10 max), not per-domain
  - Previous multi-domain architecture was based on incorrect assumption
  - Consolidation reduces complexity without sacrificing functionality or performance
- **Initial Changes:**
  - Updated STM32 sensors board (mros2-mbed-sensors-gnss) from Domain 6 → Domain 5
  - Simplified initial node_chassis_controller to operate only in Domain 2
  - Removed node_domain_bridge.cpp entirely from codebase
  - Updated launch files (rover_startup.launch.py and launch_rover_tmux.sh)
- **Documentation:**
  - Created `DOMAIN_CONSOLIDATION_SUMMARY.md` with complete refactoring details

### November 4, 2025 - Vision Navigation & Modular Architecture
- **ws_jetson Vision Navigation**
  - Separate Launch Files: Created `vision_nav_gui.launch.py` and `vision_nav_headless.launch.py`
  - YAML Configuration (Best Practice): Migrated from config.py to YAML files
  - Build Scripts: Created `build_clean.sh` and `build_inc.sh` for development workflow
  - GUI/Headless Mode Support with conditional window display
- **mros2-mbed-chassis-dynamics**
  - Refactored from monolithic to modular 4-file architecture
  - Motor control and LED status isolated into separate modules
- **Documentation**
  - Consolidated session notes into single comprehensive file
  - Professional README structure across all workspaces

---

## Performance Specifications

| Metric | Value |
|--------|-------|
| Camera stream | 30 FPS @ 1280x720 |
| Lane detection | 25-30 FPS (real-time) |
| Steering control loop | 50 Hz |
| End-to-end vision latency | 100-150 ms |
| IMU sampling | 100 Hz (publish at 10 Hz) |
| Motor control loop | 50 ms (20 Hz) |
| STM32 flash usage | 384-386 KB |
| STM32 RAM usage | ~102 KB |

---

## Future Enhancements

- Unit tests for vision detection pipeline
- Real-time parameter tuning via ROS2 services
- Machine learning-based lane detection
- Multi-lane tracking and path planning
- Sensor fusion (IMU, encoders, GNSS, lidar)
- Command timeout safety mechanism
- Encoder feedback for closed-loop motor control
- Configuration files for hardware parameters
- Web dashboard for telemetry monitoring
- Autonomous waypoint navigation

---

**Last Updated:** November 6, 2025  
**Latest Change:** Sensor Fusion Architecture (Domain 5 centralized, Domain 2 bridge only)  
**Repository:** github.com/RoboticsGG/almondmatcha  
**Branch:** main  
**License:** Apache 2.0

---

## Contributors

Development Team - Vision Navigation System - Embedded Systems Team

For issues, questions, or contributions, refer to workspace-specific documentation or session logs in `copilot-session-note/`.

## Future Work

- Non-blocking LED error indication via Ticker
- Motor controller C++ class for encapsulation
- Unit tests for PWM calculations
- Configuration file for hardware parameters
- Encoder feedback for closed-loop motor control
- Command timeout safety mechanism
- GNSS-based autonomous navigation

---

**Last Updated:** November 6, 2025  
**Latest Change:** Domain Architecture Consolidation (Domains 2, 5)  
**Repository:** github.com/RoboticsGG/almondmatcha  
**Branch:** main  
**License:** Apache 2.0

