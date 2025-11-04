# Almondmatcha - Autonomous Mobile Rover System

Distributed ROS2-based control system for autonomous mobile rover with vision navigation, chassis control, and multi-domain sensor integration. The system consists of heterogeneous computing nodes (Raspberry Pi, Jetson, STM32 microcontrollers) connected via Ethernet for real-time control and telemetry.

## System Architecture

### Overview

The rover system employs a distributed architecture with role-specific computing nodes:

- **ws_rpi (Raspberry Pi 4)**: Main on-board computer for rover coordination, mission control, and sensor fusion
- **ws_jetson (Jetson Orin Nano)**: Vision processing computer for lane detection and navigation
- **mros2-mbed-chassis-dynamics (STM32 Nucleo-F767ZI)**: Real-time motor control and IMU sensor reading
- **mros2-mbed-sensors-gnss (STM32 Nucleo-F767ZI)**: Encoder reading, power monitoring, and GNSS positioning
- **ws_base**: Ground station for telemetry monitoring and telecommand control

All nodes communicate via Ethernet using ROS2 DDS middleware with domain-based isolation for performance and modularity.

### Network Topology

```
                    Ethernet Switch (192.168.1.0/24)
                              |
        +---------------------+---------------------+---------------------+
        |                     |                     |                     |
   Raspberry Pi          Jetson Orin         STM32 Chassis          STM32 Sensors
   192.168.1.1           192.168.1.5         192.168.1.2            192.168.1.6
   (Main Control)     (Vision Navigation)  (Motor + IMU)       (Encoder + GNSS)
   ROS2 Domains 2,5,6    ROS2 Domain 5      ROS2 Domain 5       ROS2 Domain 6
        |                     |                     |                     |
        +---------------------+---------------------+---------------------+
                              |
                      Base Station PC
                      (Telemetry/Telecommand)
                      ROS2 Domain 2
                      ws_base
```

### Computing Nodes

| Node | Hardware | IP Address | Role | ROS2 Domains |
|------|----------|------------|------|--------------|
| **ws_rpi** | Raspberry Pi 4 | 192.168.1.1 | Main rover computer, coordination | 2, 5, 6 |
| **ws_jetson** | Jetson Orin Nano | 192.168.1.5 | Vision processing, lane detection | 5 |
| **chassis-dynamics** | STM32 Nucleo-F767ZI | 192.168.1.2 | Motor control, IMU sensor | 5 |
| **sensors-gnss** | STM32 Nucleo-F767ZI | 192.168.1.6 | Encoders, power, GNSS | 6 |
| **ws_base** | Ground Station PC | Variable | Telemetry/telecommand | Default |

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
│   │   │   │   ├── node_chassis_controller.cpp      Main control coordinator (Domains 2,5)
│   │   │   │   └── node_domain_bridge.cpp           Domain 2↔5 relay (50Hz)
│   │   │   └── package.xml
│   │   │
│   │   ├── pkg_chassis_sensors/
│   │   │   ├── src/
│   │   │   │   ├── node_chassis_imu.cpp             IMU data processor (Domain 5)
│   │   │   │   └── node_chassis_sensors.cpp         Encoder & power aggregator (Domain 6)
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

**Purpose:** Central coordination, sensor fusion, mission planning, domain bridging

**Running Nodes:**
- `node_chassis_controller` - Coordinates motor commands and cruise control (Domains 2, 5)
- `node_chassis_imu` - Processes IMU sensor data (Domain 5)
- `node_chassis_sensors` - Aggregates encoder and power data (Domain 6)
- `node_gnss_spresense` - GNSS position processing (Domain 5)
- `node_gnss_mission_monitor` - Mission waypoint tracking
- `node_domain_bridge` - Bridges communication between Domain 2 and Domain 5

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

**Purpose:** Real-time vision navigation, lane detection, steering parameter computation

**Running Nodes:**
- `camera_stream` - Intel RealSense D415 RGB/depth streaming (30 FPS @ 1280x720)
- `lane_detection` - Lane marker detection and parameter calculation (theta, b)
- `steering_control` - PID-based steering control with EMA filtering

**Launch Command:**
```bash
cd ~/almondmatcha/ws_jetson
ros2 launch vision_navigation vision_navigation.launch.py
```

**Key Features:**
- Centralized configuration system (config.py) with 6 config classes
- Auto-sync launch file (reads defaults from config.py)
- 100% type hint coverage for all Python code
- 50+ reusable helper functions
- Proper initialization sequencing: Camera (0s) → Lane Detection (2s) → Control (3s)

**Build:**
```bash
cd ~/almondmatcha/ws_jetson
colcon build --packages-select vision_navigation
source install/setup.bash
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
- `/tpc_chassis_cmd` (msgs_ifaces/ChassisCtrl) - Motor and steering commands

**Published Topics:**
- `/tpc_chassis_imu` (msgs_ifaces/ChassisIMU) - IMU sensor data

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
- Domain: 6

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
- `/tpc_chassis_sensors` (msgs_ifaces/ChassisSensors) - Encoder and power data

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
ros2 topic hz /tpc_rover_d415_rgb

# Monitor lane detection
ros2 topic echo /tpc_rover_nav_lane

# Monitor steering commands
ros2 topic echo /tpc_rover_fmctl

# Monitor IMU data
ros2 topic echo /tp_imu_data_d5

# Monitor sensor data
ros2 topic echo /tpc_chassis_sensors
```

**Manual Motor Control Test:**
```bash
# Set Domain ID for chassis controller
export ROS_DOMAIN_ID=5

# Send motor command (speed: 100, steering: 0.5, forward)
ros2 topic pub /tpc_chassis_cmd msgs_ifaces/msg/ChassisCtrl \
  '{fdr_msg: 2, ro_ctrl_msg: 0.5, spd_msg: 100, bdr_msg: 1}'
```

**Vision System Test:**
```bash
# Check camera stream availability
ros2 topic hz /tpc_rover_d415_rgb

# Monitor lane detection output
ros2 topic echo /tpc_rover_nav_lane

# Verify steering control is publishing
ros2 topic hz /tpc_rover_fmctl
```

## ROS2 Domain Architecture

The system uses ROS2 domain isolation for performance optimization and modularity:

| Domain ID | Purpose | Nodes |
|-----------|---------|-------|
| **Default** | Base station communication | ws_base telemetry/telecommand |
| **2** | Chassis control and vision | node_chassis_controller (publish only), ws_jetson vision nodes |
| **5** | Main rover domain | node_chassis_controller (subscribe), chassis-dynamics STM32, node_gnss_spresense |
| **6** | Sensors domain | sensors-gnss STM32, node_chassis_sensors |

**Domain Bridge:** `node_domain_bridge` translates messages between Domain 2 and Domain 5 for chassis control coordination.

**Note:** A harmless "logging was initialized more than once" warning is expected due to multiple ROS2 contexts for multi-domain support.

---

## Topic Pub/Sub Architecture

### Complete Message Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                        AUTONOMOUS MOBILE ROVER SYSTEM                               │
│                          ROS2 Multi-Domain Architecture                             │
└─────────────────────────────────────────────────────────────────────────────────────┘

                           ╔═══════════════════════════════════╗
                           ║     GROUND STATION (ws_base)      ║
                           ║  Default Domain (Telemetry)       ║
                           ╚═══════════════════════════════════╝
                                      ▲         ▼
                    Subscribes to telemetry / Publishes commands
                                      │         │
                    ┌─────────────────┴─────────┴─────────────────┐
                    │                                             │
        ┌───────────────────────┐                    ┌───────────────────────┐
        │   DOMAIN 2            │                    │   DOMAIN 5            │
        │ (Chassis & Vision)    │                    │ (Main Rover)          │
        │                       │                    │                       │
        │ • RPi (Chassis Ctrl)  │──Domain Bridge───▶ | • STM32 Chassis      │
        │ • Jetson (Vision)     │ (50Hz relay)       │ • RPi (Bridge)        │
        │ • GNSS (Base)         │◀───────────────────│ • GNSS Processor     │
        └───────────────────────┘                    └───────────────────────┘
                    │                                             │
                    └─────────────────┬─────────────────────────┬─┘
                                      │                         │
                        ┌────────────────────────────────────────────┐
                        │         DOMAIN 6 (Sensors)                 │
                        │         STM32 Sensors Node                 │
                        │                                            │
                        │ • IMU data (LSM6DSV16X)                    │
                        │ • Sensor telemetry                  │
                        └────────────────────────────────────────────┘
```

### Topic Subscription/Publication Table

| Node | Domain | Publishes | Subscribes | Message Type | Rate/QoS |
|------|--------|-----------|-----------|--------------|----------|
| **camera_stream** (ws_jetson) | 2 | `/tpc_rover_d415_rgb` | - | sensor_msgs/Image | 30 FPS (BEST_EFFORT) |
| | | `/tpc_rover_d415_depth` | - | sensor_msgs/Image | 30 FPS (BEST_EFFORT) |
| **lane_detection** (ws_jetson) | 2 | `/tpc_rover_nav_lane` | `/tpc_rover_d415_rgb` | Float32MultiArray | 25-30 FPS |
| **steering_control** (ws_jetson) | 2 | `/tpc_rover_fmctl` | `/tpc_rover_nav_lane` | Float32MultiArray | 50 Hz |
| **node_chassis_controller** (ws_rpi) | 2,5 | `/tpc_chassis_ctrl_d2` (D2) | `/tpc_rover_fmctl` | ChassisCtrl | 50 Hz |
| | | `/tpc_chassis_ctrl_d5` (D5) | `/tpc_gnss_mission_active` | | |
| **node_domain_bridge** (ws_rpi) | 2,5 | `/tpc_chassis_ctrl` (D2) | `/tpc_chassis_ctrl_d5` (D5) | ChassisCtrl | 50 Hz relay |
| **node_gnss_spresense** (ws_rpi) | 2 | `/tpc_gnss_spresense` | - | SpresenseGNSS | 10 Hz |
| **node_gnss_mission_monitor** (ws_rpi) | 2 | `/tpc_gnss_mission_active` | `/tpc_rover_dest_coordinate` | Bool | 10 Hz |
| | | `/tpc_gnss_mission_remain_dist` | `/tpc_gnss_spresense` | Float64 | |
| | | `/tpc_rover_dest_coordinate` | Service: desigation | Float64MultiArray | |
| **chassis_controller** (mros2-mbed-chassis-dynamics) | 5 | `/tpc_chassis_imu` | `/tpc_chassis_cmd` | ChassisIMU | 10 Hz |
| **sensors_node** (mros2-mbed-sensors-gnss) | 6 | `/tpc_chassis_sensors` | - | ChassisSensors | 10 Hz |
| **node_chassis_imu** (ws_rpi) | 5 | `/tpc_chassis_imu_processed` | `/tpc_chassis_imu` | ChassisIMU | 10 Hz |
| **node_chassis_sensors** (ws_rpi) | 6 | `/tpc_chassis_sensors_processed` | `/tpc_chassis_sensors` | ChassisSensors | 10 Hz |
| **mission_monitoring_node** (ws_base) | Default | - | `/tpc_gnss_mission_active` | Bool | Telemetry only |
| | | | `/tpc_gnss_mission_remain_dist` | Float64 | |
| | | | `/tpc_gnss_spresense` | SpresenseGNSS | |
| | | | `/tpc_rover_dest_coordinate` | Float64MultiArray | |
| | | | `/tpc_chassis_ctrl_d2` | ChassisCtrl | |

### Data Flow Sequences

#### Vision-Based Lane Following

```
1. camera_stream (ws_jetson/Domain 2)
   └─▶ Publishes RGB frames: /tpc_rover_d415_rgb (30 FPS)
       
2. lane_detection (ws_jetson/Domain 2)
   ├─ Subscribes: /tpc_rover_d415_rgb
   └─▶ Publishes: /tpc_rover_nav_lane [theta, b, detected] (25-30 FPS)
       
3. steering_control (ws_jetson/Domain 2)
   ├─ Subscribes: /tpc_rover_nav_lane
   ├─ Applies PID controller (Kp, Ki, Kd)
   └─▶ Publishes: /tpc_rover_fmctl [steering_angle, detected] (50 Hz)
       
4. node_chassis_controller (ws_rpi/Domain 2,5)
   ├─ Subscribes (D2): /tpc_rover_fmctl
   ├─ Provides cruise control logic
   └─▶ Publishes (D5): /tpc_chassis_ctrl_d5 (50 Hz)
       
5. node_domain_bridge (ws_rpi/Domain 2,5)
   ├─ Subscribes (D5): /tpc_chassis_ctrl_d5
   └─▶ Relays (D2): /tpc_chassis_ctrl (50 Hz relay)
       
6. chassis_controller (STM32/Domain 5)
   ├─ Subscribes (D5): /tpc_chassis_cmd
   └─▶ Executes motor control + publishes IMU: /tpc_chassis_imu (10 Hz)
```

#### GNSS-Based Navigation

```
1. node_gnss_spresense (ws_rpi/Domain 2)
   ├─ Reads: Sony Spresense GNSS via serial
   └─▶ Publishes: /tpc_gnss_spresense (lat, long, accuracy) (10 Hz)
       
2. node_gnss_mission_monitor (ws_rpi/Domain 2)
   ├─ Subscribes: /tpc_gnss_spresense
   ├─ Listens for: /tpc_rover_dest_coordinate (target waypoint)
   ├─ Calculates: Distance to waypoint, mission progress
   └─▶ Publishes:
       • /tpc_gnss_mission_active (Bool): Mission status
       • /tpc_gnss_mission_remain_dist (Float64): Distance remaining (km)
       
3. node_chassis_controller (ws_rpi/Domain 2,5)
   ├─ Subscribes (D2): /tpc_gnss_mission_active
   └─▶ Enables/disables cruise control based on mission state
```

#### Sensor Data Acquisition

```
1. chassis_controller (STM32-Dynamics/Domain 5)
   ├─ Polls: LSM6DSV16X IMU sensor @ 100 Hz
   ├─ Publishes every 10 samples (100 ms interval)
   └─▶ Topic: /tpc_chassis_imu (accel_x,y,z + gyro_x,y,z) (10 Hz)
   
2. sensors_node (STM32-Sensors/Domain 6)
   ├─ Polls: Motor encoders + INA226 power monitor
   └─▶ Topic: /tpc_chassis_sensors (encoder + battery data) (10 Hz)
   
3. node_chassis_imu (ws_rpi/Domain 5)
   ├─ Subscribes: /tpc_chassis_imu
   └─▶ Processes and republishes: /tpc_chassis_imu_processed
   
4. node_chassis_sensors (ws_rpi/Domain 6)
   ├─ Subscribes: /tpc_chassis_sensors
   └─▶ Processes and republishes: /tpc_chassis_sensors_processed
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

### Vision Navigation (ws_jetson)

Centralized configuration in `config.py` with 6 configuration classes:
- `CameraConfig`: Resolution, FPS, camera modes
- `LaneDetectionConfig`: Color thresholds, gradient parameters
- `ControlConfig`: PID gains, error weights, saturation limits
- `LoggingConfig`: File paths, CSV headers
- `TopicConfig`: ROS2 topic names
- `SystemConfig`: Node names, timing, QoS settings

Launch file automatically reads defaults from config.py (auto-sync).

**Quick Tuning Workflow:**
```bash
# 1. Test parameters (no rebuild)
ros2 launch vision_navigation vision_navigation.launch.py \
  k_p:=4.5 k_i:=0.1 k_d:=0.15

# 2. Save best values to config.py
# Edit: ControlConfig.K_P = 4.5

# 3. Rebuild
colcon build --packages-select vision_navigation

# 4. Next launch uses new defaults automatically
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

### Code Quality Standards

- Python: PEP 8 naming conventions, 100% type hints, comprehensive docstrings
- C++: snake_case naming, modular architecture, clear separation of concerns
- Documentation: Professional README files in each workspace/module
- Version Control: All work committed to main branch with clear commit messages

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

## Recent Updates (November 4, 2025)

### ws_jetson Vision Navigation
- Complete refactoring with 100% type hints
- Centralized configuration system (config.py)
- Reusable helper library (helpers.py) with 50+ functions
- Launch file auto-sync with config defaults
- Professional documentation consolidation
- Removed obsolete test directory

### mros2-mbed-chassis-dynamics
- Refactored from monolithic to modular 4-file architecture
- Motor control logic isolated (motor_control.h/cpp)
- LED status control isolated (led_status.h/cpp)
- Improved API design and thread safety
- Comprehensive module documentation

### Documentation
- Consolidated session notes into single comprehensive file
- Removed redundant markdown files
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

**Last Updated:** November 4, 2025  
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

**Last Updated:** November 4, 2025
**Repository:** github.com/RoboticsGG/almondmatcha
**Branch:** main

