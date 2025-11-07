# Setup & Troubleshooting

## Prerequisites

- Ubuntu 20.04/22.04
- ROS 2 Humble/Iron
- GNU screen or tmux
- Git

## Initial Setup

```bash
# 1. Clone repository
cd ~
git clone https://github.com/RoboticsGG/almondmatcha.git

# 2. Build workspace
cd ~/almondmatcha/ws_base
colcon build

# 3. Source workspace
source install/setup.bash

# 4. Set domain
export ROS_DOMAIN_ID=2
```

## Build Issues

### "Package not found"
```bash
# Check dependencies
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build --symlink-install
```

### "CMake Error"
```bash
# Clean build
rm -rf build install log
colcon build
```

### Interface packages fail
```bash
# Build order
colcon build --packages-select action_ifaces
colcon build --packages-select msgs_ifaces
colcon build --packages-select services_ifaces
colcon build --packages-select mission_control
```

## Runtime Issues

### Nodes won't start

**Check ROS 2:**
```bash
echo $ROS_DISTRO    # Should show: humble or iron
which ros2          # Should show path
```

**Fix:**
```bash
source /opt/ros/humble/setup.bash
source ~/almondmatcha/ws_base/install/setup.bash
```

### "No executable found"

```bash
# Check build output
ls ~/almondmatcha/ws_base/install/mission_control/lib/mission_control/

# Should show:
# - mission_command_node
# - mission_monitoring_node
```

### No topics visible

**Check domain:**
```bash
echo $ROS_DOMAIN_ID    # Should be 6
```

**Check network:**
```bash
ros2 topic list        # Should show topics
ros2 node list         # Should show nodes
```

**Check firewall:**
```bash
# Allow DDS multicast
sudo ufw allow from 224.0.0.0/4
sudo ufw allow proto udp from any to any port 7400:7500
```

### Parameters not loading

**Check config file:**
```bash
cat ~/almondmatcha/ws_base/src/mission_control/config/params.yaml
```

**Check install:**
```bash
ls ~/almondmatcha/ws_base/install/mission_control/share/mission_control/config/
```

**Rebuild if missing:**
```bash
colcon build --packages-select mission_control
```

## Screen/Tmux Issues

### "screen: command not found"
```bash
sudo apt update
sudo apt install screen
```

### "tmux: command not found"
```bash
sudo apt update
sudo apt install tmux
```

### Session already exists
```bash
# Screen
screen -S base_station -X quit

# Tmux
tmux kill-session -t base_station
```

### Can't reattach
```bash
# List sessions
screen -ls         # Screen
tmux ls            # Tmux

# Force reattach
screen -d -r base_station    # Screen
tmux a -t base_station       # Tmux
```

## Performance Issues

### High CPU usage
```bash
# Check node stats
ros2 topic hz tpc_chassis_cmd    # Should be ~10 Hz
ros2 node info /mission_monitoring_node
```

### High network usage
```bash
# Check DDS discovery
ros2 doctor
ros2 daemon stop    # Reset daemon
```

## Configuration

### Permanent ROS 2 sourcing

Add to `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
source ~/almondmatcha/ws_base/install/setup.bash
export ROS_DOMAIN_ID=2
```

### Change mission parameters

Edit `src/mission_control/config/params.yaml`:
```yaml
node_commands:
  ros__parameters:
    rover_spd: 15      # Speed (0-100%)
    des_lat: 7.007286  # Target latitude
    des_long: 100.50203 # Target longitude
```

Rebuild:
```bash
colcon build --packages-select mission_control
```

## Verification

```bash
# Check workspace
cd ~/almondmatcha/ws_base
source install/setup.bash

# List packages
colcon list

# Check executables
ros2 pkg executables mission_control

# Test node
export ROS_DOMAIN_ID=2
ros2 run mission_control mission_command_node

# Expected output:
# [INFO] Mission parameters loaded...
# [INFO] Sending speed limit...
# [INFO] Sending navigation goal...
```

## Log Files

```bash
# View logs
cd ~/almondmatcha/ws_base/log/latest_build/mission_control
cat stdout.log
cat stderr.log

# Runtime logs
cd ~/almondmatcha/ws_base/log/
ls -lht | head
```

## Getting Help

1. Check logs: `~/almondmatcha/ws_base/log/`
2. Verify build: `colcon build --event-handlers console_direct+`
3. Test manually: Run nodes without scripts
4. Check network: `ros2 doctor`
5. Review config: Verify params.yaml exists and is valid
