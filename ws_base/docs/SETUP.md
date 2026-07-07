# Setup & Troubleshooting

---

## Field Laptop Onboarding

Use this section when replacing the lab PC with a different laptop for an outdoor
field run. The rover network uses `192.168.1.0/24` and the lab PC is permanently
assigned `192.168.1.4`.

You can use a **base-IP pool** for convenience, with exactly one active base
machine at a time:

- `192.168.1.4`
- `192.168.1.10`
- `192.168.1.11`

No XML edits are needed as long as the active base machine uses one of these
three addresses.

### Hardware prerequisites

- Laptop with a wired Ethernet port (USB-C adapter works)
- Ubuntu 22.04 LTS (ROS2 Humble requirement — 20.04 also works but untested)
- tmux installed (`sudo apt install tmux`)

---

### Step 1 - Assign one approved static base IP

```bash
# Find the Ethernet interface name first
ip link show          # look for the wired interface, e.g. enp3s0, eth0, enx...

# Assign static IP (replace <iface> with your interface name)
# Use ONE approved base IP: 192.168.1.4 / 192.168.1.10 / 192.168.1.11
sudo nmcli con mod "Wired connection 1" \
    ipv4.method manual \
    ipv4.addresses 192.168.1.10/24 \
    ipv4.gateway "" \
    ipv4.dns ""
sudo nmcli con up "Wired connection 1"

# Verify
ip addr show <iface>   # should show one of: .4 / .10 / .11
ping -c 2 192.168.1.1  # RPi should reply
```

> **Why this works:** `fastdds_base.xml`, `fastdds_rover.xml`, and
> `fastdds_jetson.xml` are preconfigured for the approved base-IP pool
> (`.4`, `.10`, `.11`).
>
> **Important:** only one machine may use an approved base IP at a time.
> If two machines share the same IP simultaneously, DDS discovery and routing
> become undefined.

---

### Step 2 — Install ROS2 Humble

```bash
# Official one-liner (Ubuntu 22.04)
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install ros-humble-desktop python3-colcon-common-extensions -y
```

---

### Step 3 — Clone the repository

```bash
cd ~
git clone https://github.com/RoboticsGG/almondmatcha.git
cd almondmatcha
git checkout main          # production branch
# git checkout single-domain  # for POC measurement runs
```

---

### Step 4 — Build workspaces

Build in this order — `common_ifaces` must be built first:

```bash
# 1. Shared message interfaces
cd ~/almondmatcha/common_ifaces
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select msgs_ifaces action_ifaces services_ifaces

# 2. Base station workspace
cd ~/almondmatcha/ws_base
colcon build
```

---

### Step 5 — Configure environment

Add to `~/.bashrc` (run once, takes effect in new terminals):

```bash
cat >> ~/.bashrc << 'EOF'

# ROS2 + Almondmatcha rover
source /opt/ros/humble/setup.bash
source $HOME/almondmatcha/common_ifaces/install/setup.bash
source $HOME/almondmatcha/ws_base/install/setup.bash
export ROS_DOMAIN_ID=5
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/almondmatcha/ws_base/fastdds_base.xml
EOF
source ~/.bashrc
```

**Why `FASTRTPS_DEFAULT_PROFILES_FILE` is mandatory** — even with one NIC:

The `fastdds_base.xml` does three things that are always required:
1. Pins FastDDS to the rover Ethernet NIC (prevents accidental WiFi binding)
2. Adds `239.255.0.1:8650` to `metatrafficMulticastLocatorList` → FastDDS **joins** the
   STM32's SPDP multicast group; without this, STM32 topics are invisible
3. Lists rover unicast peer IPs for faster Linux-to-Linux discovery

Restart the daemon after setting the variable:
```bash
ros2 daemon stop && ros2 daemon start
```

---

### Step 6 — Set up SSH keys (required for `launch_field.sh`)

`launch_field.sh` SSH-launches nodes on RPi and Jetson without a password prompt.
If the keys are not set up, the script will hang at the auth step.

```bash
# Generate a key if you don't have one
ssh-keygen -t ed25519 -C "field-laptop" -f ~/.ssh/id_ed25519

# Authorise on RPi
ssh-copy-id curry@192.168.1.1

# Authorise on Jetson
ssh-copy-id yupi@192.168.1.5

# Test (should connect without a password)
ssh curry@192.168.1.1 hostname
ssh yupi@192.168.1.5  hostname
```

---

### Step 7 — Verify full setup

With the rover powered on and connected:

```bash
cd ~/almondmatcha
bash ws_base/tools/check_connectivity.sh
```

Expected output: all 5 checks pass (ping, SPDP multicast, multicast group, topic
discovery, data flow).  If STM32 topics fail, restart the daemon and retry:

```bash
ros2 daemon stop && ros2 daemon start
bash ws_base/tools/check_connectivity.sh
```

---

### Quick checklist

```
[ ] Ethernet interface assigned one approved base IP (.4 / .10 / .11)
[ ] ping 192.168.1.1 (RPi) replies
[ ] FASTRTPS_DEFAULT_PROFILES_FILE set in ~/.bashrc
[ ] ros2 daemon restarted after setting env var
[ ] SSH to curry@192.168.1.1 works without password
[ ] SSH to yupi@192.168.1.5 works without password
[ ] check_connectivity.sh passes all 5 steps
```

---

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
export ROS_DOMAIN_ID=5
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
echo $ROS_DOMAIN_ID    # Should be 5
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
export ROS_DOMAIN_ID=5
```

### Change mission parameters

Edit `src/mission_control/config/params.yaml`:
```yaml
mission_command_node:
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
export ROS_DOMAIN_ID=5
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
