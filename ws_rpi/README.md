# ws_rpi - Raspberry Pi Rover Workspace

ROS2 workspace for rover coordination, sensor fusion, and mission control running on Raspberry Pi 4B.

## Quick Start

```bash
cd ~/almondmatcha/ws_rpi
./build.sh
source install/setup.bash
./launch_rover_tmux.sh
```
## Tmux Controls
to end tmux session of ws_rpi
```bash
tmux kill-session -t rover
```

## Hardware

- **Platform:** Raspberry Pi 4B (4-8 GB RAM)
- **Network:** Static IP 192.168.1.1 (Gigabit Ethernet via switch)
- **Peripherals:** Sony Spresense GNSS module (USB port)
- **Domain:** ROS2 Domain 5 (unified architecture - all systems)
- **Connectivity:** Wired Ethernet only (no WiFi for reliability)

## Packages

| Package | Purpose |
|---------|---------|
| `pkg_chassis_control` | Motor coordination, cruise control |
| `pkg_chassis_sensors` | Sensor data logging (IMU, encoders, power) |
| `pkg_gnss_navigation` | GPS waypoint navigation, mission monitoring |
| `rover_launch_system` | System-wide launch configuration |

## Building

### Quick Start Build (Recommended)

```bash
cd ~/almondmatcha/ws_rpi
./build.sh           # Normal incremental build
./build.sh clean     # Clean rebuild (removes build/install/log folders)
source install/setup.bash
```

**What the build script does:**
- Builds interface packages first (action_ifaces, msgs_ifaces, services_ifaces)
- Sources environment automatically
- Builds application packages (pkg_chassis_control, pkg_chassis_sensors, pkg_gnss_navigation, rover_launch_system)
- Handles proper dependency ordering
- Creates install/ directory with all executables

### Manual Build (Step-by-Step)

If you need more control or want to build specific packages:

```bash
cd ~/almondmatcha/ws_rpi

# Step 1: Build interface packages
colcon build --packages-select action_ifaces msgs_ifaces services_ifaces
source install/setup.bash

# Step 2: Build application packages
colcon build --packages-select pkg_chassis_control pkg_chassis_sensors \
    pkg_gnss_navigation rover_launch_system
source install/setup.bash
```

### Build Individual Packages

After interfaces are built and sourced, you can build packages individually:

```bash
# Must source environment first
source install/setup.bash

# Chassis control (motor coordination + cruise control)
colcon build --packages-select pkg_chassis_control

# Chassis sensors (IMU + encoder/power data loggers)
colcon build --packages-select pkg_chassis_sensors

# GNSS navigation (Spresense, Ublox, mission monitor)
colcon build --packages-select pkg_gnss_navigation

# Launch system (ROS2 launch files)
colcon build --packages-select rover_launch_system
```

### Package Build Dependencies

The build order matters due to dependencies:

```
action_ifaces, msgs_ifaces, services_ifaces (must build first)
    ↓
pkg_chassis_control, pkg_chassis_sensors, pkg_gnss_navigation, rover_launch_system
```

**Important:** Always source `install/setup.bash` after building interface packages before building application packages.

## Running

The ws_rpi system can be launched in three ways: **tmux session** (recommended for full system), **monitoring-only mode**, or **manual node-by-node**.

### Option 1: Full System with Tmux (Recommended)

Launch all rover nodes in an organized tmux session with 8 panes:

```bash
cd ~/almondmatcha/ws_rpi
./launch_rover_tmux.sh
```

**What Gets Launched:**
- **Pane 0:** GNSS Spresense (Sony Spresense GNSS module)
- **Pane 1:** GNSS Ublox RTK (High-precision GNSS)
- **Pane 2:** GNSS Mission Monitor (Mission status & waypoint tracking)
- **Pane 3:** Chassis Controller (Motor coordination & cruise control)
- **Pane 4:** Chassis IMU (Accelerometer/gyroscope data logger)
- **Pane 5:** Chassis Sensors (Encoders, voltage, current logger)
- **Pane 6:** Rover Monitoring (CSV data logger for all sensors)
- **Pane 7:** Domain Relay (Bridges data from Domain 5 → Domain 4 for base station)

**All nodes run on Domain 5** (unified rover architecture).

**Tmux Session Controls:**
- **Navigate panes:** `Ctrl+b` then arrow keys
- **Zoom pane (fullscreen):** `Ctrl+b z` (toggle)
- **Scroll mode:** `Ctrl+b [` (press `q` to exit)
- **Detach session:** `Ctrl+b d` (session keeps running in background)
- **Reattach session:** `tmux attach -t rover`
- **Kill session:** `Ctrl+b &` or `tmux kill-session -t rover`
- **Close pane:** `Ctrl+d` or type `exit`

**To check if session is running:**
```bash
tmux ls
```

### Option 2: Monitoring Mode Only

Launch only monitoring components (GNSS Ublox, Rover Monitoring, Domain Relay):

```bash
cd ~/almondmatcha/ws_rpi
./launch_monitoring.sh
```

**What Gets Launched:**
- **Pane 0:** GNSS Ublox RTK (Domain 5)
- **Pane 1:** Rover Monitoring (Domain 5) - CSV logger
- **Pane 2:** Domain Relay (5→4) - Relays status to base station

**Use case:** When you only need monitoring/logging without active control, or when chassis/sensors are already running separately.

**Tmux session name:** `rover_monitoring`

**Controls:** Same as full system, use `tmux attach -t rover_monitoring` to reattach.

### Option 3: ROS2 Launch File

Launch via ROS2 launch system (all nodes at once):

```bash
cd ~/almondmatcha/ws_rpi
source install/setup.bash
export ROS_DOMAIN_ID=5
ros2 launch rover_launch_system rover_startup.launch.py
```

**Note:** This launches all nodes in a single terminal. Less visibility than tmux but suitable for automated startup.

### Option 4: Manual Launch (Individual Nodes)

```bash
# Terminal 1: Chassis Controller (Domain 5)
export ROS_DOMAIN_ID=5
cd ~/almondmatcha/ws_rpi
source install/setup.bash
ros2 run pkg_chassis_control node_chassis_controller

# Terminal 2: GNSS Spresense (Domain 5)
export ROS_DOMAIN_ID=5
ros2 run pkg_gnss_navigation node_gnss_spresense

# Terminal 3: GNSS Mission Monitor (Domain 5)
export ROS_DOMAIN_ID=5
ros2 run pkg_gnss_navigation node_gnss_mission_monitor

# Terminal 4: Chassis IMU Logger (Domain 5)
export ROS_DOMAIN_ID=5
ros2 run pkg_chassis_sensors node_chassis_imu

# Terminal 5: Chassis Sensors Logger (Domain 5)
export ROS_DOMAIN_ID=5
ros2 run pkg_chassis_sensors node_chassis_sensors
```

## Configuration

### Network Setup

Set static IP on Raspberry Pi:

```bash
# Temporary (until reboot)
sudo ip addr add 192.168.1.1/24 dev eth0
sudo ip link set eth0 up

# Permanent (edit /etc/netplan/*.yaml)
network:
  ethernets:
    eth0:
      addresses: [192.168.1.1/24]
      gateway4: 192.168.1.254
  version: 2
```

Verify connectivity:

```bash
ping 192.168.1.2    # STM32 chassis
ping 192.168.1.5    # Jetson
ping 192.168.1.6    # STM32 sensors
```

### Serial Port (Sony Spresense)

Ensure user has dialout permissions:

```bash
sudo usermod -aG dialout $USER
# Log out and log back in

# Verify device
ls -l /dev/ttyUSB*  # Should show /dev/ttyUSB0
```

## Testing

### Verify Nodes Running

```bash
export ROS_DOMAIN_ID=5
ros2 node list

# Expected:
# /node_chassis_controller
# /node_chassis_imu
# /node_chassis_sensors
# /node_gnss_spresense
# /node_gnss_mission_monitor
```

### Monitor Topics

```bash
# Chassis commands (from vision system)
ros2 topic echo tpc_chassis_cmd

# IMU data (from STM32)
ros2 topic echo tpc_chassis_imu

# Sensors data (from STM32)
ros2 topic echo tpc_chassis_sensors

# GNSS position
ros2 topic echo tpc_gnss_spresense

# Mission status
ros2 topic echo tpc_gnss_mission_active
```

### Check Data Logging

Logs are written to:

```bash
ls -lh ~/almondmatcha/runs/logs/
# Expected files:
# chassis_imu_YYYYMMDD_HHMMSS.csv
# chassis_sensors_YYYYMMDD_HHMMSS.csv
# gnss_spresense_YYYYMMDD_HHMMSS.csv
```

## Troubleshooting

### Build Errors

**Symptom:** CMake can't find `action_ifaces`, `msgs_ifaces`, or `services_ifaces`

**Solution:**
```bash
# Make sure you've sourced the environment after building interfaces
source install/setup.bash
```

**Symptom:** Build fails with old package artifacts or leftover files

**Solution:**
```bash
cd ~/almondmatcha/ws_rpi
./build.sh clean     # Removes build/, install/, log/ and rebuilds everything
source install/setup.bash
```

### Node Won't Start

**Symptom:** Node fails with "cannot open serial port"

**Solution:**
```bash
# Check permissions
sudo usermod -aG dialout $USER
# Log out and log back in

# Check device exists
ls -l /dev/ttyUSB0
```

**Symptom:** "no such package/node"

**Solution:**
```bash
# Source environment
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Topics Not Visible

**Symptom:** `ros2 topic list` shows no topics

**Solution:**
```bash
# Verify domain ID
echo $ROS_DOMAIN_ID  # Should be 5

# Check network connectivity via switch
ping 192.168.1.2  # STM32 chassis
ping 192.168.1.5  # Jetson
ping 192.168.1.6  # STM32 sensors

# Check Ethernet link status
ethtool eth0  # Should show "Link detected: yes"

# Verify switch connectivity
arp -a  # Should show all connected systems

# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

### STM32 Data Not Received

**Symptom:** No data on `tpc_chassis_imu` or `tpc_chassis_sensors`

**Solution:**
```bash
# Verify STM32 boards powered and connected to switch
ping 192.168.1.2  # Chassis
ping 192.168.1.6  # Sensors

# Check switch port LEDs (should show link activity)

# Check topics exist (means STM32 is publishing)
ros2 topic list | grep chassis

# Monitor raw topic
ros2 topic echo tpc_chassis_imu
```

## Data Logging

All sensor data logged to CSV files in `~/almondmatcha/runs/logs/`:

| Node | Log File | Rate | Content |
|------|----------|------|---------|
| `node_chassis_imu` | `chassis_imu_*.csv` | 1 Hz | Accelerometer, gyroscope |
| `node_chassis_sensors` | `chassis_sensors_*.csv` | 1 Hz | Encoders, voltage, current |
| `node_gnss_spresense` | `gnss_spresense_*.csv` | 1 Hz | Latitude, longitude, altitude |

**CSV Format Examples:**

`chassis_imu_*.csv`:
```
timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
2025-11-07T10:30:00.123,100,-50,1000,5,-3,2
```

`chassis_sensors_*.csv`:
```
timestamp,encoder_left,encoder_right,voltage,current
2025-11-07T10:30:00.123,12345,12340,12.5,2.3
```

## Directory Structure

```
ws_rpi/
├── README.md                    # This file
├── build.sh                     # Automated build script
├── launch_rover_tmux.sh         # Tmux launcher
├── BUILD.md                     # Detailed build documentation
└── src/
    ├── pkg_chassis_control/     # Motor coordination
    │   ├── src/
    │   │   └── node_chassis_controller.cpp
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── pkg_chassis_sensors/     # Sensor logging
    │   ├── src/
    │   │   ├── node_chassis_imu.cpp
    │   │   └── node_chassis_sensors.cpp
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── pkg_gnss_navigation/     # GPS navigation
    │   ├── src/
    │   │   ├── node_gnss_spresense.cpp
    │   │   └── node_gnss_mission_monitor.cpp
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── rover_launch_system/     # Launch files
    │   ├── launch/
    │   │   └── rover_startup.launch.py
    │   ├── setup.py
    │   └── package.xml
    │
    ├── action_ifaces/           # Symlink to common_ifaces
    ├── msgs_ifaces/             # Symlink to common_ifaces
    └── services_ifaces/         # Symlink to common_ifaces
```

## Documentation

- [/docs/ARCHITECTURE.md](/docs/ARCHITECTURE.md) - System architecture
- [/docs/TOPICS.md](/docs/TOPICS.md) - Topic reference
- [/docs/DOMAINS.md](/docs/DOMAINS.md) - Domain configuration
- [/docs/LAUNCH_SEQUENCE_GUIDE.md](/docs/LAUNCH_SEQUENCE_GUIDE.md) - System startup guide

---

**Platform:** Raspberry Pi 4B  
**ROS2:** Humble  
**Domain:** 5 (rover-internal)
