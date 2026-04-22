# Quick Reference: System Launch & Configuration

**Date:** April 22, 2026  
**Status:** Production (`main`) — tri-domain runtime, no POC tracing toolchain  
**Network:** All systems connected via Gigabit Ethernet switch (192.168.1.0/24)

---

## STM32 Configuration Summary

### Memory Pool Settings (Both Boards)

```cpp
MAX_NUM_PARTICIPANTS = 15        // 11 active D5 + 4 margin
SPDP_WRITER_STACKSIZE = 4096     // Halved from 8192 (critical savings)
NUM_WRITERS_PER_PARTICIPANT = 20 // Per node (ws_base/Jetson are heavy)
NUM_READERS_PER_PARTICIPANT = 20 // Per node
```

**Memory Impact:**
- SPDP heap: ~61 KB (15 × 4096)
- **Headroom: 4 spare participant slots** for development

---

## Launch Sequence (25 seconds total)

**Network:** All systems connect via Ethernet switch before starting

### 1. Power on Ethernet Switch
```
Ensure 5+ port Gigabit Ethernet switch is powered on
Connect all systems: RPi, Jetson, 2×STM32, Base PC
```

### 2. STM32 Boards (0-10s)
```
Connect to switch → Power on both boards → Wait for "Discovery complete"
Expected: ~10 seconds (8s discovery + 2s init)
Monitor serial consoles (115200 baud) for confirmation
```

### 3. ws_rpi (10-15s)
```bash
# On RPi or via SSH: ssh curry@192.168.1.1
cd ~/almondmatcha/ws_rpi
./launch_rover_tmux.sh
# Wait 3-5 seconds before next system
```

### 4. ws_jetson (15-20s)
```bash
# On Jetson or via SSH: ssh yupi@192.168.1.5
ssh yupi@192.168.1.5
cd ~/almondmatcha/ws_jetson
./launch_headless.sh  # or ./launch_gui.sh
# Wait 3-5 seconds before next system
```

### 5. ws_base (20-25s)
```bash
# On base PC (192.168.1.10, username: yupi)
cd ~/almondmatcha/ws_base
./launch_base_tmux.sh
# System fully operational
```

---

## Participant Count

**Domain 5 (Control Network — visible to STM32):**

| System | Nodes | D5 Total |
|--------|-------|---------|
| STM32 boards | 2 | 2 |
| ws_rpi | 7 | 9 |
| ws_jetson | 1 (rover_kinematic_control) | 10 |
| ws_base | 1 (mission_command_node) | **11** |
| **Headroom** | - | **+1** |
| **STM32 capacity** | - | **12** |

**Domain 4 (Telemetry — not visible to STM32):**

| System | Nodes |
|--------|-------|
| ws_base | 1 (mission_monitoring_node_pc) |
| ws_jetson | 1 (rover_local_monitoring_node) |

---

## Key Delays

- **STM32 discovery:** 8 seconds (built-in)
- **Between launch phases:** 3-5 seconds
- **ws_jetson internal:** 2s → 3s (automatic)

---

## Quick Verification

```bash
# Check all systems reachable via switch
ping 192.168.1.1  # RPi
ping 192.168.1.5  # Jetson
ping 192.168.1.2  # Chassis STM32
ping 192.168.1.6  # Sensors STM32
ping 192.168.1.10 # Base (if connected)

# Check participant count in D5
export ROS_DOMAIN_ID=5
ros2 node list | wc -l  # Should be 11

# Check D4 telemetry nodes
export ROS_DOMAIN_ID=4
ros2 node list  # mission_monitoring_node_pc, rover_local_monitoring_node

# Check STM32 config
grep "MAX_NUM_PARTICIPANTS" ~/almondmatcha/mros2-mbed-*/platform/rtps/config.h
# Should show: 12

# Verify data flow (from any machine on switch)
ros2 topic hz /tpc_chassis_imu      # ~10 Hz
ros2 topic hz /tpc_chassis_sensors  # ~4 Hz

# Test multicast on switch
ros2 multicast send    # On one machine
ros2 multicast receive # On another - should see messages
```

---

## Documentation

- **Launch guide:** [docs/LAUNCH_INSTRUCTIONS.md](LAUNCH_INSTRUCTIONS.md)
- **Architecture:** [docs/ARCHITECTURE.md](ARCHITECTURE.md)
- **Domains:** [docs/DOMAINS.md](DOMAINS.md)

> Production note: Keep tracing/POC workflow out of `main`; use feature branches for profiling experiments.
