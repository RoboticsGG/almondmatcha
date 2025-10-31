# ROS2 Rover Workspace (RPI)

This workspace contains the ROS2 packages for the autonomous rover system running on Raspberry Pi 4. The system handles GNSS-based navigation, chassis sensor data logging, and low-level motor control.

## System Architecture

### Multi-Domain Design
The system uses multiple ROS2 domains to isolate communication between different subsystems:

- **Domain 2**: GNSS navigation and high-level control
- **Domain 5**: IMU data logging and domain bridging (mROS2 communication)
- **Domain 6**: Chassis sensors logging (mROS2 communication)

### Hardware Setup
- **Raspberry Pi 4**: Main compute unit running this workspace
- **Sony Spresense**: GNSS module connected via USB serial (/dev/ttyUSB0)
- **mROS2 Microcontrollers**: 
  - Domain 5: IMU data publisher
  - Domain 6: Chassis sensors (encoders, current, voltage) publisher

---

## Package Overview

### 📡 pkg_gnss_navigation
**Purpose:** GNSS-based waypoint navigation system

**Nodes:**
- `node_gnss_spresense` - Reads GNSS data from Sony Spresense via serial port, publishes position data and logs to CSV
- `node_gnss_mission_monitor` - Monitors mission progress, calculates distance to waypoint, manages mission state

**Key Topics:**
- `/tpc_gnss_spresense` (publish) - Current GPS position
- `/tpc_gnss_mission_active` (publish) - Mission active status
- `/tpc_gnss_mission_remain_dist` (publish) - Remaining distance to waypoint
- `/tpc_rover_dest_coordinate` (publish) - Destination coordinates

**Domain:** 2

---

### 🔌 pkg_chassis_sensors
**Purpose:** Chassis sensor data collection and logging

**Nodes:**
- `node_chassis_imu` - Subscribes to IMU data from mROS2, logs accelerometer and gyroscope data to CSV
- `node_chassis_sensors` - Subscribes to chassis sensors from mROS2, logs motor encoders, system current, and voltage to CSV

**Key Topics:**
- `/tp_imu_data_d5` (subscribe) - IMU data (accel, gyro)
- `/tp_sensdata_d5` (subscribe) - Chassis sensors (encoders, current, voltage)

**Logging:**
- All sensor data is logged to `/home/curry/almondmatcha/runs/logs/`
- CSV files: `chassis_imu_YYYYMMDD_HHMMSS.csv`, `chassis_sensors_YYYYMMDD_HHMMSS.csv`
- Logging rate: 1 Hz

**Domains:** 5 (IMU), 6 (Chassis Sensors)

---

### 🎮 pkg_chassis_control
**Purpose:** Low-level chassis control and domain bridging

**Nodes:**
- `node_chassis_controller` - Low-level motor control, interfaces with mROS2 motor controller, implements cruise control logic
- `node_domain_bridge` - Bridges messages between Domain 2 and Domain 5 for cross-domain communication

**Key Topics:**
- `/tpc_gnss_mission_active` (subscribe) - Mission status for cruise control
- Motor control topics (domain-specific)

**Domains:** 2 (Controller), 5 (Bridge)

---

### 🚀 rover_launch_system
**Purpose:** System-wide launch configuration

**Launch Files:**
- `rover_startup.launch.py` - Launches all rover nodes across multiple domains

**Usage:**
```bash
cd ~/Almond/ros2-rover-ws/
source install/setup.bash
ros2 launch rover_launch_system rover_startup.launch.py
```

---

## Interface Packages

### msgs_ifaces
Custom message definitions shared across all packages:
- `SpresenseGNSS.msg` - GNSS data structure
- `ChassisIMU.msg` - IMU data structure
- `ChassisSensors.msg` - Chassis sensor data structure

### action_ifaces
Action definitions for mission control

### services_ifaces
Service definitions for rover control

---

## Building the Workspace

### Build All Packages
```bash
cd ~/almondmatcha/ws_rpi
colcon build
source install/setup.bash
```

### Build Specific Packages
```bash
# Build navigation package
colcon build --packages-select pkg_gnss_navigation

# Build sensors package
colcon build --packages-select pkg_chassis_sensors

# Build control package
colcon build --packages-select pkg_chassis_control

# Build launch system
colcon build --packages-select rover_launch_system
```

---

## Running Individual Nodes

### GNSS Navigation (Domain 2)
```bash
# Terminal 1: GNSS Data Reader
cd ~/Almond/ros2-rover-ws/
source install/setup.bash
ros2 run pkg_gnss_navigation node_gnss_spresense

# Terminal 2: Mission Monitor
cd ~/Almond/ros2-rover-ws/
source install/setup.bash
ros2 run pkg_gnss_navigation node_gnss_mission_monitor
```

### Chassis Control (Domain 2)
```bash
cd ~/Almond/ros2-rover-ws/
source install/setup.bash
ros2 run pkg_chassis_control node_chassis_controller
```

### Sensor Logging (Domain 5)
```bash
export ROS_DOMAIN_ID=5
cd ~/Almond/ros2-rover-ws/
source install/setup.bash
ros2 run pkg_chassis_sensors node_chassis_imu
```

### Sensor Logging (Domain 6)
```bash
export ROS_DOMAIN_ID=6
cd ~/Almond/ros2-rover-ws/
source install/setup.bash
ros2 run pkg_chassis_sensors node_chassis_sensors
```

### Domain Bridge (Domain 5)
```bash
export ROS_DOMAIN_ID=5
cd ~/Almond/ros2-rover-ws/
source install/setup.bash
ros2 run pkg_chassis_control node_domain_bridge
```

---

## Design Principles

### 1. Separation by Function
Packages are organized by functional subsystem rather than by hardware or deployment location:
- **Navigation** - All GNSS-related functionality
- **Sensors** - All sensor data collection
- **Control** - All control logic

### 2. Sensor-Specific Naming
Package names indicate the sensor/input source (e.g., `pkg_gnss_navigation` vs future `pkg_vision_navigation` on Jetson)

### 3. Consistent Naming Convention
- **Packages:** `pkg_<subsystem>_<function>`
- **Nodes:** `node_<descriptive_name>` (all executables prefixed with `node_`)
- **Topics:** `tpc_<descriptive_name>` (all topics prefixed with `tpc_`)
- **Variables:** `pub_` for publishers, `sub_` for subscribers

### 4. Domain Independence
Packages can span multiple domains as needed - domain assignment is a deployment concern, not a code organization concern

### 5. Scalability
Structure supports future expansion:
- Additional navigation systems (vision, lidar) in separate packages
- New sensor types can be added to existing packages
- Multi-machine deployment ready (RPI, Jetson, Base station)

---

## Future Expansion

### Vision-Based Navigation (Jetson)
A separate workspace (`ws_jetson`) will contain `pkg_vision_navigation` for camera-based obstacle avoidance and path planning. This maintains clear separation between GNSS-based (low-level) and vision-based (high-level) navigation.

### Communication Pattern
```
High-Level (Jetson) → Mid-Level (RPI GNSS) → Low-Level (RPI Control)
  Vision Nav              Waypoint Nav           Motor Control
```

---

## Dependencies

### System Requirements
- ROS2 Humble
- Ubuntu 22.04 (or compatible)
- C++17 compiler
- Python 3.10+

### ROS2 Dependencies
- `rclcpp`
- `std_msgs`
- `geometry_msgs`
- `rcpputils`

### External Libraries
- `jsoncpp` - JSON parsing for GNSS data
- `termios` - Serial communication

---

## Troubleshooting

### Build Errors
If you encounter build errors after renaming packages:
```bash
# Clean old build artifacts
rm -rf build/ install/ log/

# Rebuild all packages
colcon build
source install/setup.bash
```

### Serial Port Issues
If GNSS node can't open serial port:
```bash
# Check USB device
ls -l /dev/ttyUSB*

# Add user to dialout group
sudo usermod -aG dialout $USER
# Log out and log back in
```

### Domain Communication Issues
Ensure ROS_DOMAIN_ID is set correctly:
```bash
# Check current domain
echo $ROS_DOMAIN_ID

# List nodes in specific domain
export ROS_DOMAIN_ID=2
ros2 node list
```

---

## Contributing

When adding new nodes:
1. Follow the naming convention (`node_` prefix)
2. Use consistent topic naming (`tpc_` prefix)
3. Update this README with new nodes/topics
4. Add proper documentation in code
5. Use modern C++ practices (std::ostringstream, rcpputils::fs)

---

## License

Apache-2.0

## Maintainer

Curry (viewjirapat@gmail.com)
