# ws_rpi - Raspberry Pi Rover Workspace

ROS2 workspace for rover coordination, sensor fusion, and mission control running on Raspberry Pi 4B.

## Quick Start

```bash
cd ~/almondmatcha/ws_rpi
./build.sh
source install/setup.bash
./launch_rover_tmux.sh
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

### Automated Build (Recommended)

```bash
cd ~/almondmatcha/ws_rpi
./build.sh           # Normal build
./build.sh clean     # Clean rebuild
source install/setup.bash
```

Build script handles:
- Interface packages first (msgs, actions, services)
- Application packages second
- Proper dependency ordering
- Environment sourcing

### Manual Build

```bash
cd ~/almondmatcha/ws_rpi

# Step 1: Build interfaces
colcon build --packages-select action_ifaces msgs_ifaces services_ifaces
source install/setup.bash

# Step 2: Build application packages
colcon build --packages-select pkg_chassis_control pkg_chassis_sensors \
    pkg_gnss_navigation rover_launch_system
source install/setup.bash
```

## Running

### Tmux Launch (Recommended)

Launches all nodes in organized tmux session:

```bash
cd ~/almondmatcha/ws_rpi
./launch_rover_tmux.sh
```

**Tmux Layout (3×2 grid):**
```
┌────────────────────┬────────────────────┬────────────────────┐
│ Chassis Controller │ GNSS Spresense     │ GNSS Mission Mon   │
│ (Domain 5)         │ (Domain 5)         │ (Domain 5)         │
├────────────────────┼────────────────────┼────────────────────┤
│ Chassis IMU Logger │ Chassis Sensors    │ Monitor (Reserved) │
│ (Domain 5)         │ (Domain 5)         │                    │
└────────────────────┴────────────────────┴────────────────────┘
```

**Tmux Commands:**
- `Ctrl+b` then `arrow keys` - Navigate between panes
- `Ctrl+b` then `d` - Detach session
- `tmux attach -t rover` - Reattach session
- `Ctrl+b` then `&` - Kill session

### ROS2 Launch File

```bash
cd ~/almondmatcha/ws_rpi
source install/setup.bash
ros2 launch rover_launch_system rover_startup.launch.py
```

### Manual Launch (Individual Nodes)

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

**Symptom:** Interface packages not found

**Solution:**
```bash
cd ~/almondmatcha/ws_rpi
rm -rf build install log
./build.sh
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

- [BUILD.md](BUILD.md) - Detailed build instructions
- [/docs/ARCHITECTURE.md](/docs/ARCHITECTURE.md) - System architecture
- [/docs/TOPICS.md](/docs/TOPICS.md) - Topic reference
- [/docs/DOMAINS.md](/docs/DOMAINS.md) - Domain configuration

---

**Platform:** Raspberry Pi 4B  
**ROS2:** Humble  
**Domain:** 5 (rover-internal)
