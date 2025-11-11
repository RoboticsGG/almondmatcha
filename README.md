# Almondmatcha - Autonomous Mobile Rover

Distributed ROS2-based autonomous rover system with vision navigation, chassis dynamics control, and multi-sensor fusion architecture.

## System Overview

**Purpose:** Autonomous mobile robot with lane-following, GPS waypoint navigation, and real-time telemetry

**Architecture:** Heterogeneous distributed computing (Raspberry Pi, Jetson Orin Nano, STM32 microcontrollers)

**Communication:** ROS2 DDS over Ethernet with multi-domain architecture (Domain 5: control, Domain 6: vision)

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

## Network Architecture

```
                    Gigabit Ethernet Switch (192.168.1.0/24)
                              |
        ┌─────────────┬───────┼───────┬─────────────┬─────────────┐
        |             |       |       |             |             |
   Raspberry Pi  Jetson Orin  |  Base Station  STM32 Chassis  STM32 Sensors
   192.168.1.1   192.168.1.5  |   192.168.1.10  192.168.1.2   192.168.1.6
   Domain 5      D5 + D6      |   Domain 5      Domain 5      Domain 5
```

**Configuration:**
- Domain 5 (Control): All systems participate (10 nodes total)
- Domain 6 (Vision): Jetson localhost only (camera, lane detection)
- Gigabit Ethernet switch (multicast-enabled)
- Static IP addressing (192.168.1.0/24)
- Wired connections only for reliability

## ROS2 Domain Architecture

**Multi-Domain Architecture:**
- **Domain 5 (Control):** All rover control systems (10 participants)
  - ws_rpi: 5 nodes
  - ws_base: 2 nodes  
  - ws_jetson: 1 control node
  - STM32: 2 nodes
  
- **Domain 6 (Vision):** Jetson vision processing only (localhost isolation)
  - camera_stream
  - lane_detection

**Benefits:** Reduced STM32 memory usage, scalable vision/AI expansion, network isolation for high-bandwidth streams.

See [docs/DOMAINS.md](docs/DOMAINS.md) for details.

## Workspace Structure

```
almondmatcha/
├── README.md                          # This file
├── docs/                              # System-level documentation
│   ├── ARCHITECTURE.md                # System architecture & design
│   ├── TOPICS.md                      # Complete topic reference
│   ├── DOMAINS.md                     # Multi-domain architecture details
│   └── LAUNCH_INSTRUCTIONS.md         # Complete system launch guide
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

Connect all systems to Gigabit Ethernet switch with static IPs (192.168.1.0/24):

```bash
# Raspberry Pi (192.168.1.1)
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.1/24 ipv4.method manual
sudo nmcli con up "Wired connection 1"

# Jetson (192.168.1.5)
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.5/24 ipv4.method manual
sudo nmcli con up "Wired connection 1"

# Base Station (192.168.1.10)
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.10/24 ipv4.method manual
sudo nmcli con up "Wired connection 1"

# STM32 boards (.2, .6): IPs hardcoded in firmware - no configuration needed

# Verify connectivity
ping 192.168.1.1 && ping 192.168.1.5 && ping 192.168.1.2 && ping 192.168.1.6
```

### 3. Launch System

**Optimal sequence (~25s startup):**

```bash
# 1. Power on STM32 boards → wait for "Discovery complete" (~10s)

# 2. Launch Raspberry Pi (wait 3-5s)
cd ~/almondmatcha/ws_rpi && export ROS_DOMAIN_ID=5 && ./launch_rover_tmux.sh

# 3. Launch Jetson (wait 3-5s)
ssh yupi@192.168.1.5
cd ~/almondmatcha/ws_jetson && export ROS_DOMAIN_ID=5 && ./launch_headless.sh

# 4. Launch Base Station
cd ~/almondmatcha/ws_base && export ROS_DOMAIN_ID=5 && ./launch_base_tmux.sh
```

See [docs/LAUNCH_SEQUENCE_GUIDE.md](docs/LAUNCH_SEQUENCE_GUIDE.md) for detailed timing.

### 4. Verify Operation

```bash
export ROS_DOMAIN_ID=5

# Check all nodes visible
ros2 node list  # Should show 12 nodes

# Monitor key topics
ros2 topic hz /tpc_rover_d415_rgb     # ~30 Hz (vision)
ros2 topic hz /tpc_chassis_imu        # ~10 Hz (STM32)
ros2 topic echo /tpc_rover_nav_lane   # Lane parameters
```

## Key Topics

| Topic | Rate | Description |
|-------|------|-------------|
| `tpc_rover_d415_rgb` | 30 Hz | Camera RGB stream (Jetson) |
| `tpc_rover_nav_lane` | 30 Hz | Lane parameters [theta, b, detected] (Jetson) |
| `tpc_rover_fmctl` | 50 Hz | Steering commands (Jetson) |
| `tpc_chassis_cmd` | 50 Hz | Motor commands to STM32 (RPi) |
| `tpc_chassis_imu` | 10 Hz | IMU accel/gyro data (STM32) |
| `tpc_chassis_sensors` | 4 Hz | Encoders, voltage, current (STM32) |
| `tpc_gnss_spresense` | 10 Hz | GPS position (RPi) |

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

**STM32 not visible:**
```bash
# Verify switch connectivity
ethtool eth0  # Link detected: yes
ping 192.168.1.2 && ping 192.168.1.6
arp -a  # Should show all systems
# Check serial console (115200 baud) for network errors
```

**ROS2 topics not visible:**
```bash
echo $ROS_DOMAIN_ID  # Must be 5
ros2 multicast send/receive  # Test switch multicast support
sudo ufw allow from 192.168.1.0/24  # Allow DDS traffic
ros2 daemon stop && ros2 daemon start
```

**Vision node errors:**
```bash
rs-enumerate-devices  # Verify D415 detected
# Ensure USB 3.0 port, check /dev/video* permissions
```

See workspace README files for detailed troubleshooting.

## Future Enhancements

- Extended Kalman Filter (EKF) for multi-sensor fusion
- Autonomous obstacle avoidance with depth camera
- Path planning with GNSS waypoint navigation
- Machine learning-based lane detection
- Web-based telemetry dashboard
- Multi-rover coordination

---

**Last Updated:** November 10, 2025  
**Version:** 3.0 (Unified Domain 5, Ethernet switch topology, optimized STM32 memory pools)
