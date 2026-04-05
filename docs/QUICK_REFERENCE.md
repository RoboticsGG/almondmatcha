# Quick Reference: System Launch & Configuration

**Last updated:** April 4, 2026  
**Configuration:** Single-domain POC — all nodes on D5; 16 participants + 4 margin  
**Network:** All systems connected via Gigabit Ethernet switch (192.168.1.0/24)

---

## STM32 Configuration Summary

### Memory Pool Settings (Both Boards)

```cpp
MAX_NUM_PARTICIPANTS     = 20   // 16 D5 nodes + 4 margin
NUM_STATEFUL_WRITERS     = 3    // 2 SEDP + 1 app publisher (both boards)
NUM_STATEFUL_READERS     = 3    // 2 SEDP + 1 app sub (chassis); 2 for sensors
NUM_WRITERS_PER_PARTICIPANT = 8   // max publishers per remote node
NUM_READERS_PER_PARTICIPANT = 8   // max subscribers per remote node
NUM_WRITER_PROXIES_PER_READER = 22 // >= SPDP_MAX_NUMBER_FOUND_PARTICIPANTS (19)
NUM_READER_PROXIES_PER_WRITER = 15 // covers ~8 subscribers per topic
MAX_NUM_UNMATCHED_REMOTE_WRITERS = 20 // >= 16 actual participants
MAX_NUM_UNMATCHED_REMOTE_READERS = 25 // >= 16 participants + burst margin
SPDP_WRITER_STACKSIZE    = 4096 // halved from 8192
```

**Memory Impact:**
- SPDP heap: ~82 KB (20 × 4096)
- **OVERALL_HEAP_SIZE: ~100 KB** ((1+1+20+3) × 4096)
- **Headroom: 4 spare participant slots** for development

---

## Launch Sequence (25 seconds total)

**Network:** All systems connect via Ethernet switch before starting

### 1. Power on Ethernet Switch
```
Ensure 5+ port Gigabit Ethernet switch is powered on
Connect all systems: RPi, Jetson, 2×STM32, Base PC
```

### 2. STM32 Boards
```
Connect to switch → Power on both boards
Wait ~5 s for lwIP network stack init (not for discovery).
Discovery happens automatically once RPi nodes start sending SPDP announcements.
Monitor serial consoles (115200 baud) for confirmation.
```

### 3. ws_rpi (after STM32 boards are on network)
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

**Domain 5 (all nodes — single-domain POC):**

| System | Nodes | D5 Total |
|--------|-------|----------|
| STM32 boards | chassis + sensors | 2 |
| ws_rpi | 8 rover/GNSS nodes | 10 |
| ws_jetson | camera_stream, lane_detection, rover_kinematic_control, rover_local_monitoring | 14 |
| ws_base | 2 base nodes | **16** |
| **Headroom** | | **+4** |
| **STM32 capacity (`MAX_NUM_PARTICIPANTS`)** | | **20** |

> All nodes run on D5. No D4/D6 domain separation — this is the single-domain POC configuration.

---

## Key Delays

- **STM32 lwIP network init:** ~5 s (static IP assignment, UDP socket open)
- **SPDP endpoint matching:** automatic after all nodes are on network; typically <5 s once RPi nodes start
- **Between launch phases:** 3-5 seconds
- **ws_jetson internal:** 2s → 3s (automatic)

> **Note:** Starting STM32 before other nodes is not required for discovery correctness. SPDP is multicast — both sides announce periodically and converge regardless of order. Start STM32 early only to let its network stack initialize before RPi floods multicast.

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
ros2 node list | wc -l  # Should be 16 (all D5 nodes including STM32 boards)

# Check STM32 config
grep "MAX_NUM_PARTICIPANTS" ~/almondmatcha/mros2-mbed-*/platform/rtps/config.h
# Should show: 20 (single-domain)

# Verify data flow (from any machine on switch)
ros2 topic hz /tpc_chassis_imu      # ~10 Hz
ros2 topic hz /tpc_chassis_sensors  # ~4 Hz

# Test multicast on switch
ros2 multicast send    # On one machine
ros2 multicast receive # On another - should see messages
```

---

## Documentation

- **Memory configuration:** [docs/STM32_RTPS_MEMORY_CALCULATION.md](STM32_RTPS_MEMORY_CALCULATION.md)
- **Launch guide:** [docs/LAUNCH_INSTRUCTIONS.md](LAUNCH_INSTRUCTIONS.md)
- **Architecture:** [docs/ARCHITECTURE.md](ARCHITECTURE.md)
- **Domains:** [docs/DOMAINS.md](DOMAINS.md)
