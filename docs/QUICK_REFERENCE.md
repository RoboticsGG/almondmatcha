# Quick Reference: System Launch & Configuration

**Last updated:** April 11, 2026  
**Configuration:** Single-domain POC — all nodes on D5; 16 participants + 14 margin  
**Network:** All systems connected via Gigabit Ethernet switch (192.168.1.0/24)

---

## STM32 Configuration Summary

### Memory Pool Settings (Both Boards)

```cpp
MAX_NUM_PARTICIPANTS     = 1    // local participant pool — STM32 creates exactly 1
SPDP_MAX_NUMBER_FOUND_PARTICIPANTS = 30  // 16 nodes + 3 daemons + ros2 CLI tools + margin
NUM_STATEFUL_WRITERS     = 3    // 2 SEDP + 1 app publisher (both boards)
NUM_STATEFUL_READERS     = 3    // 2 SEDP + 1 app sub (chassis); 2 for sensors
NUM_WRITERS_PER_PARTICIPANT = 16  // max publishers per remote node (10 actual + margin)
NUM_READERS_PER_PARTICIPANT = 16  // max subscribers per remote node (11 actual + margin)
NUM_WRITER_PROXIES_PER_READER = 30 // 1 per remote participant, matches SPDP_MAX
NUM_READER_PROXIES_PER_WRITER = 30 // 1 per remote participant, matches SPDP_MAX
MAX_NUM_UNMATCHED_REMOTE_WRITERS = 2  // unused — static topology
MAX_NUM_UNMATCHED_REMOTE_READERS = 2  // unused — static topology
SPDP_WRITER_STACKSIZE    = 4096 // halved from 8192
THREAD_POOL_READER_STACKSIZE = 8192  // restored — 4096 causes HardFault during SEDP burst
THREAD_POOL_WORKLOAD_QUEUE_LENGTH = 40  // doubled from 20 — absorb discovery burst
SF_WRITER_HB_PERIOD_MS   = 1000 // 1s heartbeat (down from 4000)
SPDP_RESEND_PERIOD_MS    = 500  // 500ms SPDP announcements (down from 10000)
HISTORY_SIZE_STATEFUL     = 5   // reduced from 64 for memory efficiency
```

**Memory Impact:**
- MAX_NUM_PARTICIPANTS=1 saves ~212 KB BSS vs the old value of 20
- **OVERALL_HEAP_SIZE: ~28.7 KB** (thread stacks only)
- **Headroom: 14 spare discovery slots** (30 SPDP_MAX − 16 actual)

---

## embeddedRTPS Patches (5 total)

| Patch | Description | Runs Fixed |
|-------|-------------|------------|
| 001 | AckNack bitmap overflow, octetsToNextHeader, TopicData name overflow | 012–015 |
| 002 | Receive buffer isolation, guard words, F767ZI pointer validation | 012–015 |
| 003 | MemoryPool diagnostic counters, SEDP_VERBOSE silenced | 016–018 |
| 004 | PBUF_RAW fix, SEDP locator fallback chain, SEDP immediate resend | 019–022 |
| 005 | isMulticastAddress() byte-order fix, readLocatorIntoList() precedence | 019–022 |

See [docs/EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md) for full details.

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
# Pre-flight connectivity check (recommended — retries + SPDP check)
bash ws_base/tools/check_connectivity.sh

# Or manual checks:

# Check all systems reachable via switch
ping 192.168.1.1  # RPi
ping 192.168.1.5  # Jetson
ping 192.168.1.2  # Chassis STM32
ping 192.168.1.6  # Sensors STM32
ping 192.168.1.4  # Base PC

# Check STM32 SPDP multicast (should see packets from .2 and .6)
sudo timeout 5 tcpdump -i enp0s31f6 \
  'dst host 239.255.0.1 and udp port 8650' -nn -c 3

# Check participant count in D5
export ROS_DOMAIN_ID=5
export FASTRTPS_DEFAULT_PROFILES_FILE=~/almondmatcha/ws_base/fastdds_base.xml
ros2 node list | wc -l  # Should be 16 (all D5 nodes including STM32 boards)

# Verify data flow (from any machine on switch)
ros2 topic hz /tpc_chassis_imu      # ~10 Hz
ros2 topic hz /tpc_chassis_sensors  # ~4 Hz

# Check STM32 serial console for build banner
minicom -D /dev/ttyACM0 -b 115200  # sensors (or ttyACM1 for chassis)
```

---

## Documentation

- **Patch details:** [docs/EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md)
- **All changes summary:** [docs/STM32_CHANGES_SUMMARY.md](STM32_CHANGES_SUMMARY.md)
- **Memory configuration:** [docs/STM32_RTPS_MEMORY_CALCULATION.md](STM32_RTPS_MEMORY_CALCULATION.md)
- **Launch guide:** [docs/LAUNCH_INSTRUCTIONS.md](LAUNCH_INSTRUCTIONS.md)
- **Architecture:** [docs/ARCHITECTURE.md](ARCHITECTURE.md)
- **Domains:** [docs/DOMAINS.md](DOMAINS.md)
