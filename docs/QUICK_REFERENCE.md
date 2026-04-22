# Quick Reference: System Launch & Configuration

**Date:** June 2026  
**Status:** Production (`main`) — tri-domain D4/D5/D6, no POC tracing toolchain  
**Network:** All systems connected via Gigabit Ethernet switch (192.168.1.0/24)

---

## STM32 Configuration Summary

### Memory Pool Settings (Both Boards)

```cpp
MAX_NUM_PARTICIPANTS                 = 1    // local participant pool — STM32 creates exactly 1
SPDP_MAX_NUMBER_FOUND_PARTICIPANTS   = 30   // 12 nodes + daemons + ros2 CLI tools + margin
NUM_STATEFUL_WRITERS                 = 3    // 2 SEDP + 1 app publisher (both boards)
NUM_STATEFUL_READERS                 = 3    // 2 SEDP + 1 app sub (chassis); 2 for sensors
NUM_WRITERS_PER_PARTICIPANT          = 16   // max publishers per remote node
NUM_READERS_PER_PARTICIPANT          = 16   // max subscribers per remote node
NUM_WRITER_PROXIES_PER_READER        = 30   // must match SPDP_MAX
NUM_READER_PROXIES_PER_WRITER        = 30   // must match SPDP_MAX
THREAD_POOL_READER_STACKSIZE         = 8192 // CRITICAL — 4096 causes HardFault during SEDP burst
THREAD_POOL_WORKLOAD_QUEUE_LENGTH    = 40   // doubled from 20 to absorb discovery burst
SF_WRITER_HB_PERIOD_MS               = 1000 // 1s heartbeat
SPDP_RESEND_PERIOD_MS                = 500  // 500ms SPDP announcements
```

**Memory Impact:**
- `MAX_NUM_PARTICIPANTS=1` saves ~532 KB BSS vs old default of 15 (which caused startup crash)
- **Headroom: 12+ spare discovery slots** (SPDP_MAX=30 vs ~18 actual participants)

**embeddedRTPS Patches (5 total, applied automatically by `build.bash`):**

| Patch | Description |
|-------|-------------|
| 001 | AckNack bitmap overflow, `octetsToNextHeader==0` misparse, TopicData overflow |
| 002 | Recv buffer isolation, guard words, F767ZI pointer validation |
| 003 | MemoryPool diagnostic counters, SEDP_VERBOSE silenced |
| 004 | `PBUF_RAW` fix, SEDP locator fallback chain, SEDP immediate resend |
| 005 | `isMulticastAddress()` byte-order fix, `readLocatorIntoList()` precedence fix |

See [EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md) for full root-cause analysis.

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
Connect to switch → Power on both boards
Wait ~5 s for lwIP network stack initialization.
Discovery happens automatically once RPi nodes start sending SPDP.
Monitor serial consoles (115200 baud) to confirm banner printed.
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
cd ~/almondmatcha/ws_jetson
./launch_jetson_tmux.sh  # or ./launch_headless.sh
# Wait 3-5 seconds before next system
```

### 5. ws_base (20-25s)
```bash
# On base PC (192.168.1.10)
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
| ws_rpi | 8 | 10 |
| ws_jetson | 1 (rover_kinematic_control) | 11 |
| ws_base | 1 (mission_command_node) | **12** |
| **Headroom** | - | **+12 discovery slots** |
| **SPDP_MAX** | - | **30** |

**Domain 4 (Telemetry — not visible to STM32):**

| System | Nodes |
|--------|-------|
| ws_base | 1 (mission_monitoring_node_pc) |
| ws_jetson | 1 (rover_local_monitoring_node) |

---

## Key Delays

- **STM32 SPDP delay:** 3 seconds (inside task, non-blocking — `spin()` runs during wait)
- **Between launch phases:** 3-5 seconds
- **ws_jetson kinematic ctrl:** 4s startup delay (automatic, after RPi nodes are up)

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
ros2 node list | wc -l  # Should be 12

# Check D4 telemetry nodes
export ROS_DOMAIN_ID=4
ros2 node list  # mission_monitoring_node_pc, rover_local_monitoring_node

# Check STM32 config (both boards)
grep -E "MAX_NUM_PARTICIPANTS|SPDP_MAX|THREAD_POOL_READER" ~/almondmatcha/mros2-mbed-*/platform/rtps/config.h
# Should show: MAX_NUM_PARTICIPANTS=1, SPDP_MAX=30, THREAD_POOL_READER_STACKSIZE=8192

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
- **Topics:** [docs/TOPICS.md](TOPICS.md)
- **embeddedRTPS patches:** [docs/EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md)
- **STM32 config reference:** [docs/RTPS_CONFIG_REFERENCE.md](RTPS_CONFIG_REFERENCE.md)
- **Memory budget:** [docs/STM32_RTPS_MEMORY_CALCULATION.md](STM32_RTPS_MEMORY_CALCULATION.md)
- **All STM32 changes:** [docs/STM32_CHANGES_SUMMARY.md](STM32_CHANGES_SUMMARY.md)

> Production note: Keep tracing/POC workflow out of `main`; use feature branches for profiling experiments.
