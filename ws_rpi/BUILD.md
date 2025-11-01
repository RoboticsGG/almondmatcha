# ROS2 Workspace Build Scripts

## Quick Build

To build the entire workspace:

```bash
./build.sh
```

## Clean Build

To completely clean and rebuild everything:

```bash
./build.sh clean
```

## Manual Build Process

If you need to build manually:

### Step 1: Build Interface Packages
```bash
colcon build --packages-select action_ifaces msgs_ifaces services_ifaces
```

### Step 2: Source the Environment
```bash
source install/setup.bash
```

### Step 3: Build Application Packages
```bash
colcon build --packages-select pkg_chassis_control pkg_chassis_sensors pkg_gnss_navigation rover_launch_system
```

## Build Individual Packages

After interfaces are built and sourced:

```bash
# Chassis control (motor control + domain bridge)
colcon build --packages-select pkg_chassis_control

# Sensors (IMU + encoder/power data)
colcon build --packages-select pkg_chassis_sensors

# GNSS navigation
colcon build --packages-select pkg_gnss_navigation

# Launch files
colcon build --packages-select rover_launch_system
```

## Troubleshooting

**Problem:** CMake can't find `action_ifaces` or `msgs_ifaces`
- **Solution:** Make sure you've sourced the environment: `source install/setup.bash`

**Problem:** Old package artifacts (e.g., pkg_poseproc) causing issues
- **Solution:** Run `./build.sh clean` for a fresh build

## Package Dependencies

```
action_ifaces, msgs_ifaces, services_ifaces (interface packages)
    ↓
pkg_chassis_control, pkg_chassis_sensors, pkg_gnss_navigation, rover_launch_system
```
