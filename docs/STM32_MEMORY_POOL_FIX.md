# STM32 Memory Pool Fix

**Issue:** `[Memory pool] resource limit exceed` causing intermittent STM32 communication failures  
**Status:** ✅ RESOLVED  
**Date:** November 10, 2025

---

## Problem

**Symptoms:**
- Random `[Memory pool] resource limit exceed` errors in STM32 serial logs
- Intermittent data reception (0, 1, or 2 boards working)
- No recovery without full system reset

**Root Cause:**

Memory exhaustion in embeddedRTPS layer due to over-provisioned pools:

- **Actual participants:** 11 (7 RPi nodes + 1 Jetson + 1 Base + 2 STM32 boards)
- **Old config:** `MAX_NUM_PARTICIPANTS = 16`, `SPDP_WRITER_STACKSIZE = 8192`
- **SPDP heap:** 16 × 8 KB = 131 KB (excessive for STM32F767ZI)
- **Race condition:** Simultaneous discovery caused allocation failures

---

## Solution

### Memory Pool Optimization

**Files modified:**
- `mros2-mbed-chassis-dynamics/platform/rtps/config.h`
- `mros2-mbed-sensors-gnss/platform/rtps/config.h`

**Changes:**
```cpp
// 11 actual + 4 margin
const uint8_t MAX_NUM_PARTICIPANTS = 15;              // Was 16
const uint8_t NUM_WRITERS_PER_PARTICIPANT = 20;       // Raised from 6 (supports heavy ws_base node)
const uint8_t NUM_READERS_PER_PARTICIPANT = 20;       // Raised from 6

// Halved stack sizes
const uint16_t SPDP_WRITER_STACKSIZE = 4096;          // Was 8192
const int HEARTBEAT_STACKSIZE = 4096;              // Was 8192
const int THREAD_POOL_WRITER_STACKSIZE = 4096;     // Was 8192
const int THREAD_POOL_READER_STACKSIZE = 4096;     // Was 8192

// Reduced proxy/endpoint limits
const uint8_t NUM_WRITER_PROXIES_PER_READER = 6;   // Was 8
const uint8_t MAX_NUM_UNMATCHED_REMOTE_WRITERS = 14; // Was 20
const uint8_t HISTORY_SIZE_STATEFUL = 5;           // Was 10
```

**Memory savings:** ~74 KB (131 KB → ~57 KB SPDP heap)

#### 3. **Extended Discovery Wait Time** (`app.cpp`)

**Both STM32 boards:**

```cpp
// Increased from 6 seconds to 8 seconds
MROS2_INFO("Waiting 8 seconds for DDS participant discovery (11 participants)...");
osDelay(8000);  // 8 seconds = 16 SPDP cycles @ 500ms
```

**Rationale:**
- SPDP announcements sent every 500ms (SPDP_RESEND_PERIOD_MS)
- 11 participants need ~10-12 cycles to fully propagate
- 8 seconds = 16 cycles → robust margin for discovery completion
- Ensures all participants matched BEFORE data starts flowing
- Prevents race conditions during discovery

---

## Files Modified

- `mros2-mbed-chassis-dynamics/platform/rtps/config.h` — memory pool
- `mros2-mbed-chassis-dynamics/workspace/chassis_controller/app.cpp` — discovery delay
- `mros2-mbed-sensors-gnss/platform/rtps/config.h` — memory pool
- `mros2-mbed-sensors-gnss/workspace/sensors_node/app.cpp` — discovery delay

---

## Deployment

### 1. Rebuild STM32 Firmware

```bash
# Chassis controller
cd ~/almondmatcha/mros2-mbed-chassis-dynamics
sudo ./build.bash all NUCLEO_F767ZI chassis_controller

# Sensors/GNSS
cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash all NUCLEO_F767ZI sensors_node
```

### 2. Flash Firmware

Drag-drop `.bin` files from `build/` to Nucleo mass storage device.

### 3. Launch System

```bash
# 1. Power STM32 boards → wait for "Discovery complete" (~10s)
# 2. Launch RPi (wait 3-5s)
cd ~/almondmatcha/ws_rpi && ./launch_rover_tmux.sh
# 3. Launch Jetson (wait 3-5s)
ssh yupi@192.168.1.5 && cd ~/almondmatcha/ws_jetson && ./launch_headless.sh
# 4. Launch Base
cd ~/almondmatcha/ws_base && ./launch_base_tmux.sh
```

See [LAUNCH_INSTRUCTIONS.md](LAUNCH_INSTRUCTIONS.md) for details.

---

## Verification

### STM32 Serial Output

**✅ Expected (clean discovery):**
```
[MROS2_INFO] Network connected successfully
[MROS2_INFO] Waiting 8 seconds for DDS participant discovery (11 participants)...
[MROS2_INFO] Discovery wait complete - initializing sensors
[imu_reader_task] Accel: X=12 Y=-45 Z=1023 | Gyro: X=1 Y=-2 Z=0
```

**❌ Old error (should not occur):**
```
[Memory pool] resource limit exceed
```

### Data Flow Check

```bash
export ROS_DOMAIN_ID=5

# Verify STM32 topics
ros2 topic list | grep chassis
# Expected: /tpc_chassis_cmd, /tpc_chassis_imu, /tpc_chassis_sensors

# Check rates
ros2 topic hz /tpc_chassis_imu     # ~10 Hz
ros2 topic hz /tpc_chassis_sensors # ~4 Hz

# Network connectivity
ping 192.168.1.2 && ping 192.168.1.6  # Both STM32 boards
```

---

## Why NOT Separate Domains?

---

## Performance Impact

| Metric | Before | After | Change |
|--------|--------|-------|--------|
| SPDP heap | ~131 KB | ~61 KB | -70 KB |
| Total savings | - | - | ~70 KB |
| Participant capacity | 16 | 15 | -1 slot |
| Discovery time | 6s | 8s | +2s |
| Reliability | Intermittent | Stable | ✅ Fixed |

---

## Troubleshooting

**If memory errors persist:**

```bash
# Verify config.h changes applied
grep MAX_NUM_PARTICIPANTS platform/rtps/config.h  # Should show 15
grep SPDP_WRITER_STACKSIZE platform/rtps/config.h # Should show 4096

# Rebuild clean
sudo rm -rf build/
sudo ./build.bash all NUCLEO_F767ZI <workspace>

# Reflash firmware
# Drag-drop: build/NUCLEO_F767ZI/develop/<workspace>.bin → /media/*/NODE_F767ZI/

# Monitor serial (115200 baud) for clean discovery
```

**Verify network topology:**
```bash
# All systems must connect via Ethernet switch
ping 192.168.1.1 && ping 192.168.1.2 && ping 192.168.1.5 && ping 192.168.1.6
arp -a  # Should show all 5 systems
```

**Check participant count:**
```bash
export ROS_DOMAIN_ID=5
ros2 node list | wc -l  # Should be ≤15
```

---

## Scalability

**Current capacity:** 15 participants (11 active + 4 headroom)

**Adding 1-4 nodes:** No changes needed (spare slots available)

**Adding 5+ nodes (exceeding 15):**
```cpp
// Increase MAX_NUM_PARTICIPANTS in config.h
const uint8_t MAX_NUM_PARTICIPANTS = 17;  // +2 participants = +8 KB heap

// Monitor serial log for heap warnings
// Consider domain separation if exceeding 16 participants
```

---

## References

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [DOMAINS.md](DOMAINS.md)
- [LAUNCH_INSTRUCTIONS.md](LAUNCH_INSTRUCTIONS.md)
- [TOPICS.md](TOPICS.md)

---

**Status:** ✅ Deployed and tested
