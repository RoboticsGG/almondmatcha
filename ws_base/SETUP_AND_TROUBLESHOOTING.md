# ws_base Setup and Troubleshooting Guide

## Prerequisites

### System Requirements
- Ubuntu 20.04+ or Ubuntu 22.04+
- ROS 2 Humble or Iron installed
- Python 3.8+
- C++17 compiler (g++ 9+)

### ROS2 Installation Check
```bash
# Verify ROS2 installation
echo $ROS_DISTRO
ros2 --version

# Expected output: humble or iron
```

## Setup Instructions

### 1. Verify Workspace Structure
```bash
cd /home/yupi/almondmatcha/ws_base
ls -la

# Expected structure:
# src/
#   ├── mission_control/
#   ├── msgs_ifaces/
#   ├── action_ifaces/
#   └── services_ifaces/
# build/ (created after build)
# install/ (created after build)
```

### 2. Source ROS2 Environment
```bash
# Add to ~/.bashrc for permanent setup
source /opt/ros/iron/setup.bash  # or humble, depending on version

# Verify
echo $ROS_DISTRO
```

### 3. Build the Workspace

**Full Build**:
```bash
cd /home/yupi/almondmatcha/ws_base
colcon build
```

**Build Specific Packages** (in order):
```bash
# Build interfaces first
colcon build --packages-select msgs_ifaces action_ifaces services_ifaces

# Then build mission_control
colcon build --packages-select mission_control

# Verify build
ls -la install/
```

**Build Output**:
```
Summary: 4 packages finished [1.23s]
```

### 4. Source Workspace
```bash
cd /home/yupi/almondmatcha/ws_base
source install/setup.bash

# Verify
ros2 pkg prefix mission_control
# Output: /home/yupi/almondmatcha/ws_base/install/mission_control
```

## Running the Nodes

### Quick Start - All Default
```bash
cd /home/yupi/almondmatcha/ws_base
source install/setup.bash
ros2 launch mission_control mission_control.launch.py
```

### Quick Start - Custom Parameters
```bash
ros2 launch mission_control mission_control.launch.py \
  rover_spd:=75 \
  des_lat:=35.6892 \
  des_long:=139.6917
```

### Run Individual Nodes

**Terminal 1 - Command Node**:
```bash
cd /home/yupi/almondmatcha/ws_base
source install/setup.bash
ros2 run mission_control mission_command_node --ros-args \
  -p rover_spd:=75 \
  -p des_lat:=35.6892 \
  -p des_long:=139.6917
```

**Terminal 2 - Monitoring Node**:
```bash
cd /home/yupi/almondmatcha/ws_base
source install/setup.bash
ros2 run mission_control mission_monitoring_node
```

### Run with Debug Output
```bash
ROS_LOG_LEVEL=debug ros2 run mission_control mission_command_node
```

## Environment Variables

### Critical Variables

**ROS_DOMAIN_ID**: 
```bash
# Must match rover domain (typically 6 for base station)
export ROS_DOMAIN_ID=6
ros2 launch mission_control mission_control.launch.py

# Check current domain
echo $ROS_DOMAIN_ID
```

**ROS_LOG_LEVEL**:
```bash
# Set logging level (DEBUG, INFO, WARN, ERROR)
export ROS_LOG_LEVEL=info
ros2 run mission_control mission_command_node
```

**RMW_IMPLEMENTATION**:
```bash
# Set ROS2 middleware (Cyclone DDS recommended)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run mission_control mission_command_node
```

### Permanent Setup
Add to `~/.bashrc`:
```bash
# ROS 2 Setup
source /opt/ros/iron/setup.bash
export ROS_DOMAIN_ID=6
export ROS_LOG_LEVEL=info
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Workspace Setup
source /home/yupi/almondmatcha/ws_base/install/setup.bash
```

Then reload:
```bash
source ~/.bashrc
```

## Troubleshooting

### Build Issues

#### Error: "CMake 3.5 or later is required"
**Solution**:
```bash
cmake --version
sudo apt install cmake
```

#### Error: "Could not find a package configuration file"
**Solution**:
```bash
# Rebuild interfaces first
colcon build --packages-select msgs_ifaces action_ifaces services_ifaces

# Clean and rebuild
rm -rf build install
colcon build
```

#### Error: "rclcpp" not found
**Solution**:
```bash
# Ensure ROS2 is properly installed
source /opt/ros/iron/setup.bash

# Verify ament_cmake
apt list --installed | grep ament

# If missing, install dev tools
sudo apt install ros-iron-ament-cmake
```

### Runtime Issues

#### Error: "Destination action server not available"
**Diagnosis**:
```bash
# Check if action exists
ros2 action list | grep des_data

# If empty, rover domain not running
# Expected: /des_data
```

**Solution**:
1. Start rover nodes in their respective domains
2. Verify rover domain ID matches: `echo $ROS_DOMAIN_ID`
3. Check rover node is publishing the action:
   ```bash
   ros2 action list
   ```

#### Error: "Speed limit service not available"
**Diagnosis**:
```bash
# Check if service exists
ros2 service list | grep spd_limit

# If empty, rover service not available
# Expected: /spd_limit
```

**Solution**:
```bash
# Verify rover service is available
ros2 service list
ros2 service type /spd_limit

# Check service works
ros2 service call /spd_limit services_ifaces/srv/SpdLimit "{rover_spd: 50}"
```

#### Error: "No nodes found"
**Diagnosis**:
```bash
# Check if nodes are running
ros2 node list

# If empty, environment not set up
```

**Solution**:
```bash
# Verify environment
source /home/yupi/almondmatcha/ws_base/install/setup.bash
echo $ROS_PACKAGE_PATH

# Launch nodes again
ros2 launch mission_control mission_control.launch.py
```

#### Error: "DDS timeout" or "Discovery failed"
**Diagnosis**:
```bash
# Check domain ID
echo $ROS_DOMAIN_ID

# Check network connectivity
ping 224.0.0.251  # Multicast address
```

**Solution**:
```bash
# Set domain explicitly
export ROS_DOMAIN_ID=6

# Switch to unicast if multicast fails
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><DiscoveryMode>no</DiscoveryMode></General></Domain></CycloneDDS>'

# Or use loopback for testing
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>'
```

#### Error: "No telemetry data" in monitoring node
**Diagnosis**:
```bash
# Check if topics exist
ros2 topic list | grep tpc_

# Check topic data flow
ros2 topic echo /tpc_gnss_mission_remain_dist --max-msg-count=1

# If no output, topics not publishing
```

**Solution**:
1. Verify rover nodes are running
2. Check GNSS/sensors initialized on rover
3. Verify domain ID matches
4. Monitor individual topics:
   ```bash
   ros2 topic list
   ros2 topic echo /tpc_gnss_mission_active
   ```

### Performance Issues

#### High CPU Usage
**Diagnosis**:
```bash
# Monitor node process
top -p $(pgrep -f mission_command_node)

# Check for spinning
ps aux | grep mission_control
```

**Solution**:
- Verify no infinite loops in callbacks
- Check timer frequency in monitoring node
- Limit ROS2 logging level: `ROS_LOG_LEVEL=warn`

#### Slow Feedback/Lag
**Diagnosis**:
```bash
# Check network latency
ping rover_hostname
latency-test

# Monitor topic rate
ros2 topic hz /tpc_gnss_mission_remain_dist
```

**Solution**:
- Verify network connection stability
- Check rover feedback rate
- Reduce monitoring timer frequency if needed
- Use dedicated network for robot communication

### Debugging Techniques

#### Enable Full Debug Logging
```bash
# Terminal with command node
ROS_LOG_LEVEL=debug ros2 run mission_control mission_command_node 2>&1 | tee command_debug.log

# Terminal with monitoring node
ROS_LOG_LEVEL=debug ros2 run mission_control mission_monitoring_node 2>&1 | tee monitor_debug.log
```

#### Monitor All Topics
```bash
# In separate terminal
ros2 topic list -v  # Show topic types
ros2 topic hz /tpc_gnss_mission_remain_dist  # Show publish rate
ros2 topic bw /tpc_gnss_mission_remain_dist  # Show bandwidth
```

#### Introspect Node Details
```bash
# After nodes are running
ros2 node info /mission_command_node
ros2 node info /mission_monitoring_node

# Show what each node publishes/subscribes
ros2 node info /mission_command_node --verbose
```

#### Test Service/Action Manually
```bash
# Test speed limit service
ros2 service call /spd_limit services_ifaces/srv/SpdLimit "{rover_spd: 50}"

# Test destination action
ros2 action send_goal /des_data action_ifaces/action/DesData \
  "{des_lat: 35.6892, des_long: 139.6917}" \
  --feedback

# List all services/actions
ros2 service list
ros2 action list
```

## Advanced Configuration

### Custom Mission Parameters File

Create `ws_base/src/mission_control/config/my_mission.yaml`:
```yaml
mission_command_node:
  ros__parameters:
    rover_spd: 80
    des_lat: 35.689200
    des_long: 139.691700

mission_monitoring_node:
  ros__parameters: {}
```

Run with:
```bash
ros2 launch mission_control mission_control.launch.py \
  params_file:=src/mission_control/config/my_mission.yaml
```

### Customize Launch File

Create `custom_launch.py`:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rover_spd', default_value='75'),
        DeclareLaunchArgument('des_lat', default_value='35.6892'),
        DeclareLaunchArgument('des_long', default_value='139.6917'),
        
        Node(
            package='mission_control',
            executable='mission_command_node',
            name='mission_cmd',
            parameters=[{
                'rover_spd': LaunchConfiguration('rover_spd'),
                'des_lat': LaunchConfiguration('des_lat'),
                'des_long': LaunchConfiguration('des_long')
            }],
            output='screen',
            on_exit='respawn'  # Auto-restart if exits
        ),
        
        Node(
            package='mission_control',
            executable='mission_monitoring_node',
            name='mission_mon',
            output='screen'
        )
    ])
```

Run with:
```bash
ros2 launch custom_launch.py rover_spd:=90
```

## Network Configuration

### For Multi-Machine Setup

**On Base Station** (this machine):
```bash
export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=0  # Allow network communication
export DDS_INTERFACE_NAME=eth0  # Or your interface name
```

**On Rover/Remote Machines**:
```bash
export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=0
export DDS_INTERFACE_NAME=eth0  # Must match network
```

### DDS Configuration for Complex Networks

Create `cyclonedds.xml`:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>true</AllowMulticast>
      <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
    </General>
    <Discovery>
      <ParticipantIndex>automatic</ParticipantIndex>
      <MaxAutoParticipantIndex>9</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Use with:
```bash
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
ros2 launch mission_control mission_control.launch.py
```

## Verification Checklist

Before running production missions:

- [ ] Workspace builds successfully
- [ ] Environment variables set (`ROS_DOMAIN_ID=6`)
- [ ] Both nodes start without errors
- [ ] Rover nodes are running
- [ ] Services are available: `ros2 service list | grep spd_limit`
- [ ] Actions are available: `ros2 action list | grep des_data`
- [ ] Topics are publishing: `ros2 topic list | grep tpc_`
- [ ] Command node accepts goal
- [ ] Monitoring node displays status
- [ ] No error logs in output
- [ ] Network latency is acceptable (<100ms)

## Recovery Procedures

### If Nodes Crash
```bash
# Check crash log
journalctl -xe

# Restart
ros2 launch mission_control mission_control.launch.py
```

### If Mission Hangs
```bash
# Stop command node (Ctrl+C)
# Monitoring will continue running

# Restart command node with same parameters
ros2 run mission_control mission_command_node --ros-args \
  -p rover_spd:=50 \
  -p des_lat:=35.6892 \
  -p des_long:=139.6917
```

### If Complete Restart Needed
```bash
# Kill all ROS processes
pkill -f "ros2 run"
pkill -f "ros2 launch"

# Clean and rebuild
cd /home/yupi/almondmatcha/ws_base
rm -rf build install log
colcon build

# Re-source and restart
source install/setup.bash
ros2 launch mission_control mission_control.launch.py
```

## Performance Profiling

### Measure Latency
```bash
# Create latency test script
ros2 topic echo /des_data/feedback --csv > feedback.csv

# Analyze timing between messages
# Calculate average, max, min delays
```

### Monitor Memory Usage
```bash
# Watch memory growth
watch -n 1 'ps aux | grep mission_control | grep -v grep'

# Record baseline before long mission
free -h

# Check after mission
free -h
```

## Next Steps

1. **Validate Setup**: Run through checklist above
2. **Test Communication**: Verify services/actions work
3. **Dry Run**: Run with non-critical parameters
4. **Full Mission**: Execute actual rover commands
5. **Monitor**: Watch telemetry in separate window
6. **Record Logs**: Save output for debugging if needed

## Support Resources

- **ROS2 Documentation**: https://docs.ros.org/en/latest/
- **Troubleshooting Guide**: https://docs.ros.org/en/latest/The-ROS2-Project/Contributing/Code-style-language-support.html
- **Community Forums**: https://discourse.ros.org/
- **Package Issues**: Check GitHub workspace repository
