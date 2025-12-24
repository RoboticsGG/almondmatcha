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

| Domain | Purpose | Network Scope | Participants | Key Characteristics |
|--------|---------|---------------|--------------|---------------------|
| **5** | Control Network | Network-wide (all systems) | **10 nodes total:**<br>• ws_rpi: 5 nodes (GNSS spresense, mission monitor, chassis controller, IMU logger, sensors logger)<br>• ws_base: 2 nodes (mission command, monitoring)<br>• ws_jetson: 1 node (steering control)<br>• STM32: 2 nodes (chassis, sensors) | • Real-time control loop<br>• Low-frequency messages<br>• Visible to all systems<br>• Optimized for STM32 memory |
| **6** | Vision Processing | Localhost only (Jetson) | **2 nodes:**<br>• camera_stream<br>• lane_detection | • High-bandwidth streams (30 FPS)<br>• RGB/Depth 1280×720<br>• Isolated from network<br>• Invisible to STM32 boards |

**Cross-Domain Bridge:** The `steering_control` node subscribes to Domain 6 (`/tpc_rover_nav_lane`) and publishes to Domain 5 (`/tpc_rover_fmctl`), enabling seamless vision-to-control data flow.

**Benefits:** Reduced STM32 memory usage (60% free RAM), network bandwidth optimization, scalable vision expansion without affecting control loop, native multi-domain communication without bridge nodes.

See [docs/DOMAINS.md](docs/DOMAINS.md) for complete architecture details.

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

# 2. Launch Raspberry Pi (Domain 5 - wait 3-5s)
cd ~/almondmatcha/ws_rpi
export ROS_DOMAIN_ID=5
./launch_rover_tmux.sh

# 3. Launch Jetson (Domains 6 + 5 - wait 3-5s)
ssh yupi@192.168.1.5
cd ~/almondmatcha/ws_jetson
# Note: Script handles both Domain 6 (vision) and Domain 5 (control) automatically
./launch_headless.sh

# 4. Launch Base Station (Domain 5)
cd ~/almondmatcha/ws_base
export ROS_DOMAIN_ID=5
./launch_base_tmux.sh
```

**Domain Architecture:**
- **Domain 6:** Jetson vision processing only (camera, lane detection) - localhost isolated
- **Domain 5:** All control systems (RPi, Jetson control node, STM32s, Base Station)

See [docs/LAUNCH_INSTRUCTIONS.md](docs/LAUNCH_INSTRUCTIONS.md) for detailed timing and [docs/DOMAINS.md](docs/DOMAINS.md) for architecture details.

### 4. Verify Operation

```bash
# Set domain for control network
export ROS_DOMAIN_ID=5

# Check all Domain 5 nodes visible (10 nodes expected)
ros2 node list
# Expected: RPi (5 nodes), Jetson (1 control node), STM32 (2 nodes), Base (2 nodes)

# Monitor key topics (all on Domain 5)
ros2 topic hz /tpc_rover_fmctl        # ~50 Hz (steering commands from Jetson)
ros2 topic hz /tpc_chassis_cmd        # ~50 Hz (motor commands to STM32)
ros2 topic hz /tpc_chassis_imu        # ~10 Hz (IMU from STM32)
ros2 topic hz /tpc_rover_nav_lane     # ~30 Hz (lane parameters from Jetson)

# Note: Camera topics (/tpc_rover_d415_rgb) run on Domain 6 (Jetson localhost only)
# and won't be visible from other systems
```

## Ending a tmux Session

To gracefully stop all running nodes and close the tmux session:

- **Detach from tmux:**
  Press `Ctrl+b` then `d` (session keeps running in background)

- **Kill the tmux session (stop all nodes):**
  ```bash
  tmux kill-session -t rover         # For ws_rpi
  tmux kill-session -t jetson_vision # For ws_jetson
  tmux kill-session -t base_station  # For ws_base
  ```

- **Reattach to a running session:**
  ```bash
  tmux attach-session -t <session_name>
  ```

## Key Topics

**Domain 5 (Control Network) - Visible Across All Systems:**

| Topic | Rate | Publisher | Description |
|-------|------|-----------|-------------|
| `tpc_rover_fmctl` | 50 Hz | Jetson (steering_control) | Steering commands to RPi |
| `tpc_chassis_cmd` | 50 Hz | RPi (chassis_controller) | Motor commands to STM32 |
| `tpc_chassis_imu` | 10 Hz | STM32 (chassis_controller) | IMU accel/gyro data |
| `tpc_chassis_sensors` | 4 Hz | STM32 (sensors_node) | Encoders, voltage, current |
| `tpc_gnss_spresense` | 10 Hz | RPi (node_gnss_spresense) | GPS position |
| `tpc_rover_nav_lane` | 30 Hz | Jetson (lane_detection) | Lane parameters [theta, b, detected] |

**Domain 6 (Vision Processing) - Jetson Localhost Only:**

| Topic | Rate | Publisher | Description |
|-------|------|-----------|-------------|
| `tpc_rover_d415_rgb` | 30 Hz | Jetson (camera_stream) | RGB image stream (1280×720) |
| `tpc_rover_d415_depth` | 30 Hz | Jetson (camera_stream) | Depth image stream |

**Note:** Domain 6 topics are NOT visible from RPi, Base Station, or STM32 boards. Only the processed lane parameters (`tpc_rover_nav_lane`) are published to Domain 5.

See [docs/TOPICS.md](docs/TOPICS.md) for complete reference.

## Documentation

**System-Level:** [ARCHITECTURE.md](docs/ARCHITECTURE.md) | [TOPICS.md](docs/TOPICS.md) | [DOMAINS.md](docs/DOMAINS.md) | [LAUNCH_INSTRUCTIONS.md](docs/LAUNCH_INSTRUCTIONS.md)

**Workspace-Level:** [ws_rpi](ws_rpi/README.md) | [ws_jetson](ws_jetson/README.md) | [ws_base](ws_base/README.md) | [STM32 Chassis](mros2-mbed-chassis-dynamics/README.md) | [STM32 Sensors](mros2-mbed-sensors-gnss/README.md)

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

**Repository:** RoboticsGG/almondmatcha (main branch) | **License:** Apache 2.0

**Contributing:** Follow ROS2 naming conventions (`node_*`, `tpc_*`), write descriptive commits, update documentation for architectural changes, test on all platforms before merging.

## Troubleshooting


**STM32 not visible:**
```bash
# Verify switch connectivity
ethtool eth0  # Link detected: yes
ping 192.168.1.2 && ping 192.168.1.6
arp -a  # Should show all systems
# Check serial console (115200 baud) for network errors
```

## Monitoring STM32 Boards with minicom

You can monitor the serial output of both STM32 boards (chassis and sensors) using `minicom` on any Linux PC. This is useful for debugging firmware, checking network status, and viewing real-time logs.

### 1. Identify Serial Ports

Plug each NUCLEO-F767ZI board into your PC via USB. List available serial devices:

```bash
ls /dev/ttyACM*
```

Typical output (with two boards):

```
/dev/ttyACM0  /dev/ttyACM1
```

Unplug/replug each board to determine which port corresponds to chassis or sensors firmware.

### 2. Start minicom

Open a terminal for each board and run:

```bash
sudo minicom -D /dev/ttyACM0 -b 115200
```

Replace `/dev/ttyACM0` with the correct port for each board. The default baud rate is **115200**.

### 3. Typical Output

You should see boot messages, network discovery, and real-time logs from the firmware. Example:

```
[mros2] Discovery complete. IP: 192.168.1.2
[chassis] Motor enabled. IMU OK.
```

### 4. Exit minicom

Press `Ctrl+A` then `X` to exit minicom.

**Tip:** You can run minicom in multiple terminals to monitor both boards simultaneously.

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

**Last Updated:** December 24, 2025  
**Version:** 3.1 (Multi-domain D5/D6 architecture, improved documentation)
