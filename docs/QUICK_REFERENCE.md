# Quick Reference: System Launch & Configuration

**Date:** November 10, 2025  
**Status:** ✅ Optimized for 14 participants (12 active + 2 headroom)  
**Network:** All systems connected via Gigabit Ethernet switch (192.168.1.0/24)

---

## STM32 Configuration Summary

### Memory Pool Settings (Both Boards)

```cpp
MAX_NUM_PARTICIPANTS = 14        // Current 12 + headroom 2
SPDP_WRITER_STACKSIZE = 4096     // Halved from 8192 (critical savings)
NUM_WRITERS_PER_PARTICIPANT = 6  // Reduced from 10
NUM_READERS_PER_PARTICIPANT = 6  // Reduced from 10
```

**Memory Impact:**
- SPDP heap: 57 KB (was 131 KB) → **-74 KB saved** ✅
- Total heap: ~94 KB (was ~200 KB)
- **Available: ~106 KB free** for application use
- **Headroom: 2 spare participant slots** for development

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
# On RPi or via SSH (192.168.1.1)
cd ~/almondmatcha/ws_rpi
./launch_rover_tmux.sh
# Wait 3-5 seconds before next system
```

### 4. ws_jetson (15-20s)
```bash
# On Jetson or via SSH (192.168.1.5)
ssh yupi@192.168.1.5
cd ~/almondmatcha/ws_jetson
./launch_headless.sh  # or ./launch_gui.sh
# Wait 3-5 seconds before next system
```

### 5. ws_base (20-25s)
```bash
# On base PC (192.168.1.10 or similar)
cd ~/almondmatcha/ws_base
./launch_base_tmux.sh
# System fully operational
```

---

## Participant Count

| System | Nodes | Total |
|--------|-------|-------|
| STM32 boards | 2 | 2 |
| ws_rpi | 5 | 7 |
| ws_jetson | 3 | 10 |
| ws_base | 2 | **12** |
| **Headroom** | - | **+2** |
| **Capacity** | - | **14** |

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

# Check participant count
export ROS_DOMAIN_ID=5
ros2 node list | wc -l  # Should be ≤ 14

# Check STM32 config
grep "MAX_NUM_PARTICIPANTS" ~/almondmatcha/mros2-mbed-*/platform/rtps/config.h
# Should show: 14

# Verify data flow (from any machine on switch)
ros2 topic hz /tpc_chassis_imu      # ~10 Hz
ros2 topic hz /tpc_chassis_sensors  # ~4 Hz

# Test multicast on switch
ros2 multicast send    # On one machine
ros2 multicast receive # On another - should see messages
```

---

## Next Steps

1. **Rebuild STM32 firmware:**
   ```bash
   cd ~/almondmatcha/mros2-mbed-chassis-dynamics && ./build.bash
   cd ~/almondmatcha/mros2-mbed-sensors-gnss && ./build.bash
   ```

2. **Flash both boards** (ST-Link or mass storage)

3. **Test launch sequence** following timing above

4. **Verify serial logs** - no `[Memory pool] resource limit exceed`

---

## Documentation

- **Full details:** `docs/STM32_MEMORY_POOL_FIX.md`
- **Launch guide:** `docs/LAUNCH_SEQUENCE_GUIDE.md`
- **Architecture:** `docs/ARCHITECTURE.md`
- **Domains:** `docs/DOMAINS.md`
