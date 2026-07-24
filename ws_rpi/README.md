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
| `chassis_control` | Motor coordination, closed-loop speed control (encoder feedback), cruise control |
| `chassis_sensors` | Sensor data logging (IMU, encoders, power) |
| `gnss_navigation` | GPS waypoint navigation, mission monitoring || `rover_monitoring` | Telemetry relay publisher (D4), CSV data logger || `rover_bringup` | System-wide launch configuration |

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
- Builds application packages (chassis_control, chassis_sensors, gnss_navigation, rover_monitoring, rover_bringup)
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
colcon build --packages-select chassis_control chassis_sensors \
    gnss_navigation rover_monitoring rover_bringup
source install/setup.bash
```

### Build Individual Packages

After interfaces are built and sourced, you can build packages individually:

```bash
# Must source environment first
source install/setup.bash

# Chassis control (motor coordination + cruise control)
colcon build --packages-select chassis_control

# Chassis sensors (IMU + encoder/power data loggers)
colcon build --packages-select chassis_sensors

# GNSS navigation (Spresense, Ublox, mission monitor)
colcon build --packages-select gnss_navigation

# Rover monitoring (telemetry relay + CSV logger)
colcon build --packages-select rover_monitoring

# Launch system (ROS2 launch files)
colcon build --packages-select rover_bringup
```

### Package Build Dependencies

The build order matters due to dependencies:

```
action_ifaces, msgs_ifaces, services_ifaces (must build first)
    ↓
chassis_control, chassis_sensors, gnss_navigation, rover_monitoring, rover_bringup
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
ros2 launch rover_bringup rover_startup.launch.py
```

**Note:** This launches all nodes in a single terminal. Less visibility than tmux but suitable for automated startup.

### Option 4: Manual Launch (Individual Nodes)

```bash
# Terminal 1: Chassis Controller (Domain 5)
export ROS_DOMAIN_ID=5
cd ~/almondmatcha/ws_rpi
source install/setup.bash
ros2 run chassis_control chassis_controller_node

# Terminal 2: GNSS Spresense (Domain 5)
export ROS_DOMAIN_ID=5
ros2 run gnss_navigation gnss_spresense_node

# Terminal 3: GNSS Mission Monitor (Domain 5)
export ROS_DOMAIN_ID=5
ros2 run gnss_navigation gnss_mission_monitor_node

# Terminal 4: Chassis IMU Logger (Domain 5)
export ROS_DOMAIN_ID=5
ros2 run chassis_sensors chassis_imu_node

# Terminal 5: Chassis Sensors Logger (Domain 5)
export ROS_DOMAIN_ID=5
ros2 run chassis_sensors chassis_sensors_node
```

## Configuration

### Chassis Speed Control

`chassis_controller_node` closes the speed loop against `/tpc_chassis_sensors` wheel
encoder feedback (~4 Hz) instead of applying PWM duty open-loop, so the chassis holds
its target speed under varying terrain load. Steering keeps updating at the full 50 Hz
of `/tpc_rover_ctrl_cmd`; only the speed correction is paced by the slower encoder feed.
Falls back to open-loop passthrough (today's legacy behavior) if the encoder feed goes
stale.

Tuned via `src/chassis_control/config/chassis_speed_control_params.yaml`
(`use_closed_loop_speed`, `speed_kp`/`speed_ki`/`speed_kd`, `speed_integral_limit`,
`max_ticks_per_sec`, `sensor_timeout_sec`) — loaded automatically by
`ros2 launch rover_bringup rover_startup.launch.py` and by `launch_rover_tmux.sh`.
Set `use_closed_loop_speed` to `false` (no rebuild needed via `ros2 param set`) to
force legacy open-loop behavior.

`max_ticks_per_sec` needs field calibration: drive open-loop at 100% duty on flat
ground, then compute ticks/sec from consecutive `Encoder_Left`/`Encoder_Right` rows in
the logged `chassis_sensors.csv` (see [docs/CSV_LOGGING.md](../docs/CSV_LOGGING.md)).

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
# /chassis_controller_node
# /chassis_imu_node
# /chassis_sensors_node
# /gnss_spresense_node
# /gnss_ublox_node
# /gnss_mission_monitor_node
# /mission_monitoring_node_rpi
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

Logs are written to run directories:

```bash
ls ~/almondmatcha/ws_rpi/runs/
# Expected directories:
# run_001_YYYYMMDD_HHMMSS/
# run_002_YYYYMMDD_HHMMSS/
# ...

ls ~/almondmatcha/ws_rpi/runs/run_001_*/
# Expected files: rtk_gnss.csv, spresense_gnss.csv, chassis_imu.csv,
#                 chassis_sensors.csv, chassis_cmd.csv, mission_state.csv
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

All sensor data is logged to time-stamped run directories by `mission_monitoring_node_rpi`
(rover_monitoring package) in `~/almondmatcha/ws_rpi/runs/run_NNN_YYYYMMDD_HHMMSS/`:

| File | Rate | Content |
|------|------|---------|
| `rtk_gnss.csv` | ~10 Hz | RTK GNSS position (u-blox) |
| `spresense_gnss.csv` | ~10 Hz | GPS position (Spresense) |
| `chassis_imu.csv` | ~10 Hz | Accelerometer, gyroscope |
| `chassis_sensors.csv` | ~4 Hz | Encoders, voltage, current |
| `chassis_cmd.csv` | ~50 Hz | Motor commands |
| `mission_state.csv` | event | Mission status, steering, lane detection |

Logging starts automatically when `mission_monitoring_node_rpi` launches.
See [docs/CSV_LOGGING.md](../docs/CSV_LOGGING.md) for complete schema.

## Directory Structure

```
ws_rpi/
├── README.md                    # This file
├── build.sh                     # Automated build script
├── launch_rover_tmux.sh         # Tmux launcher
├── BUILD.md                     # Detailed build documentation
└── src/
    ├── chassis_control/     # Motor coordination + closed-loop speed control
    │   ├── src/
    │   │   └── chassis_controller_node.cpp
    │   ├── config/
    │   │   └── chassis_speed_control_params.yaml
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── chassis_sensors/     # Sensor logging
    │   ├── src/
    │   │   ├── chassis_imu_node.cpp
    │   │   └── chassis_sensors_node.cpp
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── gnss_navigation/     # GPS navigation
    │   ├── src/
    │   │   ├── gnss_spresense_node.cpp
    │   │   ├── gnss_ublox_node.cpp
    │   │   └── gnss_mission_monitor_node.cpp
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── rover_monitoring/    # Telemetry relay + CSV logger
    │   ├── src/
    │   │   ├── mission_monitoring_node_rpi.cpp
    │   │   └── rover_monitoring_node.cpp
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── rover_bringup/     # Launch files
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
**Domain:** 5 (7 control nodes) + 4 (mission_monitoring_node_rpi relay context)
