# Almondmatcha - Autonomous Mobile Rover

Distributed ROS2-based autonomous rover system with vision navigation, chassis dynamics control, and multi-sensor fusion architecture.

## System Overview

**Purpose:** Autonomous mobile robot with lane-following, GPS waypoint navigation, and real-time telemetry

**Architecture:** Heterogeneous distributed computing (Raspberry Pi, Jetson Orin Nano, STM32 microcontrollers)

**Communication:** ROS2 DDS over Ethernet with two-domain isolation for sensor fusion

## Hardware Components

| Component | Hardware | IP Address | Function |
|-----------|----------|------------|----------|
| **Main Computer** | Raspberry Pi 4B | 192.168.1.1 | Sensor fusion, mission control, coordination |
| **Vision Computer** | Jetson Orin Nano 8GB | 192.168.1.5 | Lane detection, visual navigation |
| **Chassis Controller** | NUCLEO-F767ZI + IKS4A1 | 192.168.1.2 | Motor control, IMU sensing |
| **Sensors Controller** | NUCLEO-F767ZI + SimpleRTK2b | 192.168.1.6 | GNSS, encoders, power monitoring |
| **Base Station** | Linux PC | Variable | Ground station telemetry/command |
| **GNSS Module** | Sony Spresense | USB (RPi) | High-precision GNSS receiver |

### Sensors & Peripherals

- **IMU:** LSM6DSV16X 6-axis (X-NUCLEO-IKS4A1 shield on chassis controller)
- **RTK GNSS:** SimpleRTK2b board (attached to sensors controller)
- **Camera:** Intel RealSense D415 RGB-D (on Jetson)
- **Encoders:** Quadrature encoders on both drive motors
- **Power Monitor:** INA226 voltage/current sensor

## Network Architecture

```
                    Ethernet Switch (192.168.1.0/24)
                              |
        ┌─────────────────────┼─────────────────────┐
        |                     |                     |
   Raspberry Pi          Jetson Orin          STM32 Boards
   192.168.1.1           192.168.1.5          .2 (Chassis)
   (ROS2 Domain 5)       (ROS2 Domain 5)      .6 (Sensors)
   + Bridge (Domain 2)                        (ROS2 Domain 5)
        |
        └─── Base Station PC (ROS2 Domain 2)
```

## ROS2 Domain Architecture

Two-domain design for centralized sensor fusion:

| Domain | Purpose | Nodes |
|--------|---------|-------|
| **Domain 5** | Rover-internal processing | All rover nodes (STM32s, RPi, Jetson) |
| **Domain 2** | Base station bridge | ws_base ↔ bridge node only |

**Benefits:**
- All sensor data accessible on same domain (enables EKF fusion)
- Low-latency direct communication between rover nodes
- Isolated telemetry/command interface to base station

See [docs/DOMAINS.md](docs/DOMAINS.md) for detailed architecture.

## Workspace Structure

```
almondmatcha/
├── README.md                          # This file
├── docs/                              # System-level documentation
│   ├── ARCHITECTURE.md                # System architecture & design
│   ├── TOPICS.md                      # Complete topic reference
│   └── DOMAINS.md                     # Domain architecture details
│
├── common_ifaces/                     # Shared ROS2 interfaces (messages/actions/services)
│   ├── msgs_ifaces/                   # ChassisCtrl, ChassisIMU, ChassisSensors, SpresenseGNSS
│   ├── action_ifaces/                 # DesData (navigation goals)
│   └── services_ifaces/               # SpdLimit (speed control)
│
├── ws_rpi/                            # Raspberry Pi 4 workspace
│   ├── README.md                      # Build & run instructions
│   ├── build.sh                       # Automated build script
│   ├── launch_rover_tmux.sh          # Tmux-based system launcher
│   └── src/
│       ├── pkg_chassis_control/       # Motor coordination, cruise control
│       ├── pkg_chassis_sensors/       # Sensor data logging (IMU, encoders)
│       ├── pkg_gnss_navigation/       # GPS waypoint navigation
│       └── rover_launch_system/       # ROS2 launch files
│
├── ws_jetson/                         # Jetson Orin Nano workspace
│   ├── README.md                      # Build & run instructions
│   ├── build_clean.sh / build_inc.sh  # Build scripts
│   ├── launch_gui.sh / launch_headless.sh  # Launch scripts
│   └── vision_navigation/
│       ├── vision_navigation_pkg/     # Lane detection nodes
│       └── config/                    # YAML configuration
│
├── ws_base/                           # Base station workspace
│   ├── README.md                      # Build & run instructions
│   ├── launch_base_screen.sh          # Launch script (GNU screen)
│   └── src/mission_control/           # Telemetry monitoring, command
│
├── mros2-mbed-chassis-dynamics/       # STM32 motor controller firmware
│   ├── README.md                      # Build & flash instructions
│   ├── build.bash                     # Docker-based build
│   └── workspace/chassis_controller/  # Motor control + IMU tasks
│
├── mros2-mbed-sensors-gnss/           # STM32 sensors firmware
│   ├── README.md                      # Build & flash instructions
│   ├── build.bash                     # Docker-based build
│   └── workspace/sensors_node/        # Encoder + power + GNSS tasks
│
└── runs/logs/                         # Data logging output (CSV files)
```

## Quick Start

### 1. Build All Workspaces

```bash
# STM32 Firmware
cd ~/almondmatcha/mros2-mbed-chassis-dynamics
sudo ./build.bash all NUCLEO_F767ZI chassis_controller
# Flash: Copy build/mros2-mbed.bin to NUCLEO board

cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash all NUCLEO_F767ZI sensors_node
# Flash: Copy build/mros2-mbed.bin to NUCLEO board

# Raspberry Pi
cd ~/almondmatcha/ws_rpi
./build.sh
source install/setup.bash

# Jetson
cd ~/almondmatcha/ws_jetson
./build_clean.sh
source install/setup.bash

# Base Station
cd ~/almondmatcha/ws_base
colcon build
source install/setup.bash
```

### 2. Network Configuration

Configure static IPs on all devices:

```bash
# Raspberry Pi (192.168.1.1)
sudo ip addr add 192.168.1.1/24 dev eth0
sudo ip link set eth0 up

# Jetson (192.168.1.5)
sudo ip addr add 192.168.1.5/24 dev eth0
sudo ip link set eth0 up

# STM32 boards have hardcoded IPs (.2 and .6)
# Verify connectivity
ping 192.168.1.2  # Chassis
ping 192.168.1.6  # Sensors
```

### 3. Launch System

```bash
# Raspberry Pi (Domain 5 + bridge to Domain 2)
cd ~/almondmatcha/ws_rpi
./launch_rover_tmux.sh

# Jetson (Domain 5)
cd ~/almondmatcha/ws_jetson
./launch_headless.sh  # or ./launch_gui.sh for development

# Base Station (Domain 2)
cd ~/almondmatcha/ws_base
./launch_base_screen.sh
```

### 4. Verify Operation

```bash
# Check active topics (on Raspberry Pi)
export ROS_DOMAIN_ID=5
ros2 topic list

# Monitor camera stream
ros2 topic hz tpc_rover_d415_rgb

# Monitor lane detection
ros2 topic echo tpc_rover_nav_lane

# Check IMU data
ros2 topic echo tpc_chassis_imu
```

## Key Topics

**Vision Navigation (30 FPS):**
- `tpc_rover_d415_rgb` - Camera RGB stream
- `tpc_rover_nav_lane` - Lane parameters [theta, b, detected]
- `tpc_rover_fmctl` - Steering commands [angle, detected]

**Chassis Control (50 Hz):**
- `tpc_chassis_cmd` - Motor commands to STM32

**Sensor Data (10 Hz):**
- `tpc_chassis_imu` - IMU accelerometer/gyroscope
- `tpc_chassis_sensors` - Encoders, voltage, current
- `tpc_gnss_spresense` - GPS position data

**Mission Control:**
- `tpc_gnss_mission_active` - Navigation mission status
- `tpc_gnss_mission_remain_dist` - Distance to waypoint

See [docs/TOPICS.md](docs/TOPICS.md) for complete reference.

## Documentation

### System-Level (Top-Level /docs)
- [ARCHITECTURE.md](docs/ARCHITECTURE.md) - System design, data flow, hardware integration
- [TOPICS.md](docs/TOPICS.md) - Complete topic reference with message types
- [DOMAINS.md](docs/DOMAINS.md) - Domain architecture and rationale

### Workspace-Level (Build & Run)
- [ws_rpi/README.md](ws_rpi/README.md) - Raspberry Pi build/run instructions
- [ws_jetson/README.md](ws_jetson/README.md) - Jetson build/run instructions  
- [ws_base/README.md](ws_base/README.md) - Base station build/run instructions
- [mros2-mbed-chassis-dynamics/README.md](mros2-mbed-chassis-dynamics/README.md) - STM32 chassis firmware
- [mros2-mbed-sensors-gnss/README.md](mros2-mbed-sensors-gnss/README.md) - STM32 sensors firmware

## Performance Specifications

| Metric | Value |
|--------|-------|
| Vision processing | 30 FPS @ 1280×720 |
| Lane detection | 25-30 FPS |
| Steering control loop | 50 Hz |
| IMU sampling | 10 Hz (published) |
| GNSS update rate | 10 Hz |
| End-to-end vision latency | 100-150 ms |

## Development

### Repository
- **GitHub:** RoboticsGG/almondmatcha
- **Branch:** main
- **License:** Apache 2.0

### Contributing
- Follow ROS2 naming conventions (nodes: `node_*`, topics: `tpc_*`)
- Use descriptive commit messages
- Update relevant documentation for architectural changes
- Test across all platforms before merging

## Troubleshooting

**STM32 not visible on network:**
- Verify Ethernet cable connection
- Check LED1 status (should be ON after init)
- Monitor serial output at 115200 baud

**ROS2 topics not visible:**
- Verify `ROS_DOMAIN_ID` matches (5 for rover, 2 for base)
- Check network connectivity: `ping <ip>`
- Restart ROS2 daemon: `ros2 daemon stop && ros2 daemon start`

**Vision node errors:**
- Check camera connection: `rs-enumerate-devices`
- Verify USB bandwidth (use USB 3.0 port)
- Reduce resolution/FPS if CPU overloaded

**Build failures:**
- Clean workspace: `rm -rf build install log`
- Source ROS2 environment: `source /opt/ros/humble/setup.bash`
- Rebuild dependencies first (interface packages)

## Future Enhancements

- Extended Kalman Filter (EKF) for multi-sensor fusion
- Autonomous obstacle avoidance with depth camera
- Path planning with GNSS waypoint navigation
- Machine learning-based lane detection
- Web-based telemetry dashboard
- Multi-rover coordination

---

**Last Updated:** November 7, 2025  
**Version:** 2.0 (Two-domain architecture with sensor fusion preparation)
