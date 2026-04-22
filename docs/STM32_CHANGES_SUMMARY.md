# STM32 Firmware Changes Summary

**Branch:** `main` (production)  
**Base:** mROS-base/mros2 v0.5.4  
**Boards:** NUCLEO-F767ZI × 2 — chassis (192.168.1.2) and sensors (192.168.1.6)

This document summarises all changes from the upstream mROS2 v0.5.4 baseline to the
current production firmware state. For root-cause analysis of the embeddedRTPS bugs,
see [EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md).

---

## 1. embeddedRTPS Patches (5 patches)

Applied automatically by `build.bash` at firmware build time.
Patches live in `platform/patches/`.

| Patch | Root cause | Crash symptom |
|-------|-----------|---------------|
| `001-rtps-deserialization-fixes.patch` | `octetsToNextHeader=0` misparse; AckNack bitmap overflow; TopicData name overflow | HardFault in `doCopyAndMoveOn()` with garbage R1 |
| `002-rtps-hardfault-prevention.patch` | No isolation between recv buffer and lwIP pbuf; no pointer validation | Same crash; intermittent after Patch 001 |
| `003-discovery-pool-diagnostic.patch` | MemoryPool error lacks context; SEDP log spam fills UART buffer | `ros2 topic info` shows `Publisher count: 0` |
| `004-spdp-pbuf-and-sedp-locator-fallback.patch` | `PBUF_TRANSPORT` corrupts CDR payload; SEDP locator fallback missing; SEDP not sent immediately | STM32 invisible to FastDDS |
| `005-multicast-address-detection-fix.patch` | `isMulticastAddress()` byte-order bug; operator precedence bug in `readLocatorIntoList()` | Multicast locator silently discarded; SEDP to wrong port |

---

## 2. `platform/rtps/config.h` Tuning

All values changed from mROS2 defaults to production values.

| Parameter | Default | Production | Reason |
|-----------|---------|------------|--------|
| `MAX_NUM_PARTICIPANTS` | `15` | `1` | Default allocates 15×~35 KB = 525 KB BSS → exceeds 512 KB SRAM |
| `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS` | `14` | `30` | 14 exhausted by daemons + CLI tools; overflow → silent drop |
| `NUM_WRITER_PROXIES_PER_READER` | `6` | `30` | 6 < 11 D5 nodes → MemoryPool panic during SEDP burst |
| `NUM_READER_PROXIES_PER_WRITER` | `6` | `30` | Same |
| `THREAD_POOL_READER_STACKSIZE` | `4096` | `8192` | 4096 → HardFault at 99.6% stack depth during SEDP burst |
| `THREAD_POOL_WORKLOAD_QUEUE_LENGTH` | `20` | `40` | 20 → drop during 3-node simultaneous SPDP burst |
| `SF_WRITER_HB_PERIOD_MS` | `2000` | `1000` | Faster heartbeat for SEDP reliability |
| `HISTORY_SIZE_STATEFUL` | `64` | `5` | Deep history not needed; saves BSS |
| `NUM_WRITERS_PER_PARTICIPANT` | varies | `16` | Sized for 10 actual publishers + margin |
| `NUM_READERS_PER_PARTICIPANT` | varies | `16` | Sized for 11 actual subscribers + margin |

Full reference: [RTPS_CONFIG_REFERENCE.md](RTPS_CONFIG_REFERENCE.md).

---

## 3. `build.bash` Fixes

Original `build.bash` used `mbed-tools deploy` which is broken in Docker and pulls
stale embedded dependencies. Replaced with explicit dependency management.

| Fix | Before | After |
|-----|--------|-------|
| mbed-os acquisition | `mbed-tools deploy` (broken) | `git clone --depth 1 https://github.com/ARMmbed/mbed-os.git -b mbed-os-6.17.0` |
| mros2 acquisition | `mbed-tools deploy` | `git clone --depth 1 https://github.com/mROS-base/mros2-mbed.git mros2` |
| embeddedRTPS submodule | Not initialized | `git -C mros2 submodule update --init` |
| Patch application | None | Loop over `platform/patches/*.patch` with idempotent `--reverse` check |
| Docker safe.directory | Missing → `git: dubious ownership` | `git config --global --add safe.directory '*'` in `DOCKERCMD_PRE` |
| msgs_ifaces sync | Manual step | Automatic `rsync` from `common_ifaces/msgs_ifaces/` |
| File ownership | All files owned by root after Docker run | `chown -R $SUDO_USER build/ mros2/ mbed-os/ platform/` |

**Chassis board only:** Added `sed -i 's/chassis_im_u\.hpp/chassis_imu.hpp/g' platform/templates.hpp`
to fix a typo in the generated CMake template (`chassis_im_u.hpp` → `chassis_imu.hpp`).

---

## 4. Application Firmware Changes (`app.cpp`)

### Startup Banner (Both Boards)

New multi-line banner at boot prints board identity and key config values via `MROS2_INFO()`:

**Chassis (192.168.1.2):**
```
[MROS2] ========================================
[MROS2]   Firmware: chassis-dynamics
[MROS2]   Board IP: 192.168.1.2
[MROS2]   Platform: NUCLEO_F767ZI
[MROS2]   Domain:   5
[MROS2]   SPDP max: 30  HB: 1000 ms  SPDP: 500 ms
[MROS2]   Node: RoverWithIMU
[MROS2]   Pub:  tpc_chassis_imu (10 Hz)
[MROS2]   Sub:  tpc_chassis_cmd
[MROS2] ========================================
```

**Sensors (192.168.1.6):**
```
[MROS2] ========================================
[MROS2]   Firmware: sensors-gnss
[MROS2]   Board IP: 192.168.1.6
[MROS2]   Platform: NUCLEO_F767ZI
[MROS2]   Domain:   5
[MROS2]   SPDP max: 30  HB: 1000 ms  SPDP: 500 ms
[MROS2]   Node: mros2_node_sensors
[MROS2]   Pub:  tpc_chassis_sensors (4 Hz)
[MROS2]   Sub:  (none)
[MROS2] ========================================
```

This replaces the old single-line `[mros2] starting...` with no config context.

### Discovery Wait — Non-Blocking

| | Before | After |
|-|--------|-------|
| Location | `main()` before `spin()` | Inside IMU reader / sensors task |
| Duration | 8 seconds | 3 seconds |
| Effect | `spin()` blocked during wait — incoming SPDP ignored | `spin()` running during wait — STM32 responds to SPDP from t=0 |

**Impact:** STM32 now participates in DDS discovery from firmware start. The 3-second
internal delay is just to let lwIP initialize before the first SPDP announcement.

### Print-Once Sensor Output

Both boards previously printed IMU/sensor data on every loop iteration, flooding the
serial console at 100 Hz and saturating the UART buffer. A `_first_print_done` flag
now limits sensor output to the **first sample only** (boot confirmation), then goes
silent unless a new firmware debug session is started.

### GNSS Task Rate (Sensors Board)

Reduced from 10 Hz → 2 Hz. RTK GNSS fixes arrive at ~1 Hz so 10 Hz polling was
redundant and added UART bus contention.

### Node Name Fix (Sensors Board)

Node name changed from stale `mros2_node_sensors_d6` → `mros2_node_sensors`.
The `_d6` suffix was from an earlier single-domain prototype and had no runtime
effect but created confusing topic-info output.

---

## 5. lwIP Configuration

From `mbed_app.json`:

```json
"lwip.pbuf-pool-size": 32
```

Default is 16. During DDS discovery, 30+ RTPS packets arrive simultaneously.
Pool size 16 caused `pbuf_alloc()` to return NULL → silently dropped SPDP packets
→ STM32 never discovered all peers.

---

## 6. FastDDS Interface Selection (Multi-NIC Hosts)

**Context:** Base PC (192.168.1.10) and Jetson (192.168.1.5) have multiple network
interfaces (Ethernet, loopback, possibly Wi-Fi). FastDDS selects the DDS transport
interface from its routing table. On machines where the default route is NOT the
192.168.1.x Ethernet interface, DDS multicast discovery will fail to reach the STM32 boards.

**Symptom:** `ros2 topic list` shows the topics, but `ros2 topic info /tpc_chassis_imu`
shows `Publisher count: 0` — FastDDS is receiving SPDP on one interface but trying
to send SEDP on a different interface.

**Solution:** Create a FastDDS profile XML and point `FASTRTPS_DEFAULT_PROFILES_FILE`
to it before launching ROS2 nodes:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udpv4_rover</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>192.168.1.10</address>  <!-- Base PC; use 192.168.1.5 on Jetson -->
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="rover_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udpv4_rover</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <metatrafficMulticastLocatorList>
        <locator><udpv4><port>8650</port><address>239.255.0.1</address></udpv4></locator>
      </metatrafficMulticastLocatorList>
    </rtps>
  </participant>
</profiles>
```

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_rover.xml
```

These XML files are **not checked into the repository** because the interface address
is machine-specific. Recommended location: `~/.ros/fastdds_rover.xml`.

The `metatrafficMulticastLocatorList` entry (`239.255.0.1:8650`) is what Patch 004/005
fixed on the STM32 side — both sides must agree on the multicast address for SEDP.

---

**See Also:** [EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md) · [RTPS_CONFIG_REFERENCE.md](RTPS_CONFIG_REFERENCE.md) · [STM32_RTPS_MEMORY_CALCULATION.md](STM32_RTPS_MEMORY_CALCULATION.md)
# STM32 Firmware Changes Summary — Single-Domain POC

**Branch:** `single-domain`  
**Last updated:** April 12, 2026  
**Boards:** 2× NUCLEO-F767ZI (chassis-dynamics @ 192.168.1.2, sensors-gnss @ 192.168.1.6)  
**Upstream base:** mROS-base/mros2 v0.5.4 (commit `de70e01`)

This document summarizes all changes made to mros2, embeddedRTPS, lwIP
configuration, and application firmware to make STM32 boards communicate
with Linux FastDDS (ROS2 Humble) in the single-domain POC (all nodes on D5).

---

## 1. embeddedRTPS Patches (5 total)

Patches live in `platform/patches/` and are applied by `build.bash` after
cloning the mros2 submodule. Same patches used in both firmware trees.

### Patch 001 — Deserialization fixes

**Problem:** HardFault during DDS discovery burst (runs 012–015).
Corrupted `currentPos` pointer from mispardsed RTPS submessages.

**Changes (3 bugs):**

| Bug | File | Fix |
|-----|------|-----|
| `octetsToNextHeader == 0` misparse | `MessageReceiver.cpp` | Set `nextPos = size` when last submessage uses 0-length encoding (RTPS spec §8.3.3.2.3) |
| TopicData name overflow | `TopicData.cpp` | Bounds-check `topicNameLength`/`typeNameLength` against `MAX_TOPICNAME_LENGTH`/`MAX_TYPENAME_LENGTH` before `ucdr_deserialize_array_char()` |
| AckNack bitmap overflow | `types.h`, `MessageTypes.cpp` | Expand `SequenceNumberSet::bitMap` from `array<uint32_t,1>` to `array<uint32_t,8>` (256-bit per RTPS spec); add `numBits > 256` guard |

### Patch 002 — HardFault prevention (defense-in-depth)

**Problem:** Same HardFault as patch 001 — deterministic `memcpy` crash on
the 2nd submessage from FastDDS SPDP/SEDP packets. Patch 001 reduced
frequency but crash persisted.

**Changes (3 layers):**

| Layer | File | Fix |
|-------|------|-----|
| Receive buffer isolation | `MessageReceiver.h/.cpp` | Copy incoming packet from lwIP pbuf into a 1536 B BSS buffer before parsing — decouples from pbuf lifecycle |
| Guard-word corruption detection | `MessageReceiver.h/.cpp` | Bracket buffer with `0xDEADC0DE` guard words; check after each submessage callback; abort if corrupted |
| Pointer validation | `MessageTypes.cpp` | `isValidReadPtr()` validates source address against F767ZI memory map (DTCM/SRAM1/SRAM2/Flash) before `memcpy` |

### Patch 003 — Discovery pool diagnostic

**Problem:** `[MemoryPool] RESSOURCE LIMIT EXCEEDED` with no context
(runs 016–018). SPDP_MAX was 19 but `ros2` CLI tools created ephemeral
participants that exhausted the pool.

**Changes:**

| File | Fix |
|------|-----|
| `MemoryPool.h` | Added `size` and `used` counters to the error message: `RESSOURCE LIMIT EXCEEDED (size=30, used=30)` |
| `Log.h` | Set `SEDP_VERBOSE=0` — silences hundreds of serial log lines during SEDP burst that saturated UART and blocked the reader thread |

### Patch 004 — SPDP packet corruption + SEDP discovery fixes

**Problem:** `ros2 topic info` shows Publisher count: 0 (runs 019–022).
STM32 SPDP packets were malformed and SEDP never completed.

**Changes (3 bugs):**

| Bug | File | Fix |
|-----|------|-----|
| SPDP packet corruption | `PBufWrapper.h` | `PBUF_TRANSPORT` → `PBUF_RAW`. lwIP `PBUF_TRANSPORT` reserves 50 bytes for network headers inside the pbuf, corrupting CDR payload offset. `PBUF_RAW` reserves 0 bytes. |
| Missing SEDP locator fallback | `SPDPAgent.cpp` | Added 3-level fallback in `addProxiesForBuiltInEndpoints()`: (1) metatraffic unicast, (2) metatraffic multicast, (3) derived from default unicast `port - 1` |
| Missing SEDP immediate resend | `SPDPAgent.cpp` | Call `setAllChangesToUnsent()` on `sedpPubWriter` + `sedpSubWriter` after `addProxiesForBuiltInEndpoints()` — sends SEDP immediately instead of waiting for next HB cycle |

### Patch 005 — Multicast address detection (ROOT CAUSE)

**Problem:** Patch 004's Fallback 1 (metatraffic multicast) never fired
because the multicast locator list was always empty. Two bugs in locator
parsing silently discarded `PID_METATRAFFIC_MULTICAST_LOCATOR` from
FastDDS SPDP.

**Changes (2 bugs):**

| Bug | File | Fix |
|-----|------|-----|
| `isMulticastAddress()` byte-order | `UdpDriver.cpp` | `(addr.addr >> 28) == 14` assumed host byte order. lwIP stores `ip4_addr_t` in little-endian: `transformIP4ToU32(239,255,0,1) = 0x0100FFEF`, `>> 28 = 0` not 14. Fix: `ip4_addr_ismulticast(&addr)` — lwIP's own byte-order-correct macro. |
| `readLocatorIntoList()` operator precedence | `ParticipantProxyData.cpp` | `ret && isSameSubnet() \|\| isMulticastAddress()` parsed as `(ret && isSameSubnet()) \|\| isMulticastAddress()`. With Bug A always returning false, multicast locators were discarded. Fix: `ret && (isSameSubnet() \|\| isMulticastAddress())` |

**Effect:** These two bugs were the true root cause of all Publisher count: 0
failures from runs 019 onwards. With both fixed, `m_metatrafficMulticastLocatorList`
is correctly populated, Patch 004's Fallback 1 fires, and SEDP reaches FastDDS
via 239.255.0.1:8650.

---

## 2. config.h Tuning (tracked in git, not patch files)

Both boards' `platform/rtps/config.h` values changed from upstream defaults:

| Parameter | Upstream | New | Rationale |
|-----------|----------|-----|-----------|
| `MAX_NUM_PARTICIPANTS` | 1 | **1** | Local pool only — STM32 creates 1 participant. Was temporarily 20 (wasted 212 KB BSS). |
| `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS` | 5 | **30** | 16 nodes + 3 daemons + ros2 CLI tools + margin. Was 19 (caused pool overflow). |
| `NUM_WRITERS_PER_PARTICIPANT` | 8 | **16** | 10 actual remote publishers + margin |
| `NUM_READERS_PER_PARTICIPANT` | 8 | **16** | 11 actual remote subscribers + margin |
| `NUM_WRITER_PROXIES_PER_READER` | 15 | **30** | 1 per remote participant, matches SPDP_MAX |
| `NUM_READER_PROXIES_PER_WRITER` | 15 | **30** | Same |
| `MAX_NUM_UNMATCHED_REMOTE_WRITERS` | 20 | **2** | Unused on static topology — saves BSS |
| `MAX_NUM_UNMATCHED_REMOTE_READERS` | 20 | **2** | Same |
| `THREAD_POOL_READER_STACKSIZE` | 4096 | **8192** | 4096 causes HardFault during deep SEDP callback chains |
| `THREAD_POOL_WORKLOAD_QUEUE_LENGTH` | 20 | **40** | Absorb discovery burst when 3 Linux nodes join simultaneously |
| `SF_WRITER_HB_PERIOD_MS` | 4000 | **1000** | Faster SEDP convergence — reduces worst-case re-delivery latency |
| `SPDP_RESEND_PERIOD_MS` | 10000 | **500** | Faster discovery — 20× improvement in SPDP announcement rate |
| `HISTORY_SIZE_STATEFUL` | 64 | **5** | Memory efficiency — STM32 topics have low throughput |
| `SPDP_LEASE_DURATION` | {100,0} | **{100,0}** | Unchanged — 100 seconds |
| `DOMAIN_ID` | 0 | **5** | Single-domain POC |

---

## 3. lwIP Configuration (via mbed_app.json)

| Parameter | Default | Set to | Rationale |
|-----------|---------|--------|-----------|
| `lwip.pbuf-pool-size` | 8 | **32** | Absorb SPDP/SEDP packet burst during 16-participant discovery. ~19.7 KB RAM (well within 371 KB free heap). Increased from 20→32 to prevent `pbuf_alloc()` → NULL during full POC launch. |
| `lwip.socket-max` | 4 | **8** | Enough for RTPS multicast + unicast sockets |
| `lwip.udp-socket-max` | 4 | **8** | Same |

**Per-pbuf cost:** ~616 bytes (struct 16B + buffer 592B + MEMP overhead).
Pool at 32 entries costs ~19.7 KB total.  Both boards report ~371 KB
(chassis) / ~368 KB (sensors) heap free, so 32 pbufs is well within margins.

---

## 4. FastDDS XML Configuration (Linux side)

Three XML files provide the same DDS transport configuration for each Linux
workspace: `fastdds_base.xml`, `fastdds_rover.xml`, `fastdds_jetson.xml`.

**Key settings that affect STM32 interoperability:**

| Setting | Value | Why |
|---------|-------|-----|
| UDPv4 `interfaceWhiteList` | Per-machine rover Ethernet IP | Prevents FastDDS from using WiFi interface |
| `useBuiltinTransports` | `false` | Only use the explicitly configured UDPv4 transport |
| `metatrafficMulticastLocatorList` | 239.255.0.1:8650 | FastDDS joins SPDP multicast for domain 5 — required because embeddedRTPS only supports multicast discovery |
| `initialPeersList` | Multicast 239.255.0.1:8650 + all unicast peer IPs | Belt-and-suspenders: multicast for STM32, unicast for Linux-to-Linux |

**Critical interaction:** When `metatrafficMulticastLocatorList` is set,
FastDDS uses multicast-only metatraffic and omits `PID_METATRAFFIC_UNICAST_LOCATOR`
from its SPDP. It also does NOT bind metatraffic unicast ports (8660, 8662,
etc.). This is why Patch 005 was essential — the STM32 must correctly parse
the multicast locator to know where to send SEDP.

---

## 5. Application Firmware Changes

### Chassis board (`mros2-mbed-chassis-dynamics/workspace/chassis_controller/app.cpp`)

| Change | Purpose |
|--------|---------|
| Build banner macros (`FW_BUILD_DATE`, `FW_BUILD_TIME`, `FW_BOARD_NAME`, `FW_BOARD_IP`) | Identify firmware version on serial console |
| RTPS config printout at boot | Show domain, SPDP period, HB period, SPDP_MAX on serial |
| Discovery note comments | Document why `mros2::spin()` must not be delayed |
| `osDelay(4000)` before `memory_reporter_start()` | Delay JSON memory reporter so first-sample print appears before the memory stream |
| IMU task 3s SPDP wait | Wait for multicast discovery before first publish |
| IMU task stack 2048 → 4096 | Prevent stack overflow during mros2 publish (CDR + embeddedRTPS + lwIP UDP send) |

### Sensors board (`mros2-mbed-sensors-gnss/workspace/sensors_node/app.cpp`)

| Change | Purpose |
|--------|---------|
| Same build banner + RTPS config printout | Consistent diagnostics |
| 3s SPDP wait before main loop | Wait for multicast discovery before first publish |
| `memory_reporter_start()` moved inside first-sample `if` block | Start reporter only after first successful publish — cleaner boot output |

### Memory reporter (`memory_reporter.h`)

Both boards include a shared header that creates a background thread
emitting periodic JSON memory snapshots (`{"type":"STM32_MEM",...}`)
over USB serial at 1-second intervals.  Used by `collect_stm32_memory.py`
on the base PC for POC measurements.

---

## 6. Connectivity Check Script

**File:** `ws_base/tools/check_connectivity.sh`

Pre-flight script that verifies the full communication stack without
launching the POC experiment.

| Step | Check | Method |
|------|-------|--------|
| 1 | Network reachability | `ping` all 5 nodes |
| 2 | SPDP multicast from STM32 | `tcpdump` for packets from .2/.6 to 239.255.0.1:8650 |
| 3 | FastDDS multicast group membership | `ip maddr show` for 239.255.0.1 |
| 4 | ROS2 topic discovery | `ros2 topic list` with up to 3 retries (8s/10s/14s timeout) |
| 5 | Data flow verification | `ros2 topic echo --once` — both topics checked **in parallel** with up to 3 retries |

### Why steps 4 and 5 need retries

Each `ros2` CLI invocation creates an **ephemeral FastDDS participant** that
must complete the full SPDP→SEDP discovery cycle with both STM32 boards:

```
CLI participant SPDP announce  ──►  STM32 receives on 239.255.0.1:8650
                                         │
               STM32 processes SPDP, adds remote participant
               STM32 sends SEDP via 239.255.0.1:8650 (multicast)
                                         │
         FastDDS receives SEDP  ◄──  endpoint matching
                                         │
                                   topics visible
```

**Worst-case timing per attempt:**
- Wait for next STM32 SPDP: 0–500 ms (SPDP_RESEND_PERIOD_MS)
- STM32 processes SPDP + sends SEDP: ~instant
- SEDP heartbeat-ACKNACK cycle: 0–1000 ms (SF_WRITER_HB_PERIOD_MS)
- FastDDS processes SEDP: ~instant
- **Total: up to ~3.5 seconds**

**Why the sequential approach failed:**

The initial step 5 checked each topic **sequentially** — `ros2 topic echo`
for topic A, then topic B. Each invocation is a separate process with its
own DDS participant. When topic A's process exits, its participant
disappears without sending SPDP dispose (100s lease). Topic B's new
participant must rediscover the STM32 boards from scratch. But the rapid
participant churn (step 4 + step 5 retries = 7+ ephemeral participants in
~30s) saturates the STM32's `THREAD_POOL_WORKLOAD_QUEUE_LENGTH=40`:

1. Each new participant triggers `addNewRemoteParticipant()` + SEDP setup
2. Each departed participant's stale SPDP entry occupies a slot for 100s
3. The threadpool processes pending work for all participants (alive + stale)
4. By the 5th–7th participant, the queue backs up and SEDP exchange for
   the newest participant is delayed past the timeout

**Fix:** Step 5 now runs both `ros2 topic echo --once` checks in **parallel**
(backgrounded processes). Both participants start discovery simultaneously,
reducing participant churn. This brought consistent pass rate from ~50% to
100% across 5+ consecutive runs.

**Compounding factors:**
- **lwIP pbuf pool exhaustion:** `PBUF_POOL_SIZE=20` can drop packets during
  burst discovery, requiring a fresh SPDP cycle
- **No SPDP jitter:** STM32 boards resend SPDP at exactly 500 ms intervals
  with no randomization — synchronized multicast can collide on the wire
- **ThreadPool queue saturation:** 40 slots can be temporarily full during
  concurrent multi-node discovery, causing `dropped packet` events

---

## 7. POC Experiment Launch Timing

**File:** `ws_base/launch_poc_experiment.sh` + per-machine subscripts

The full POC creates 16 DDS participants on D5 (8 RPi + 4 Jetson + 2 Base +
2 STM32). All 14 Linux participants must complete SPDP→SEDP with both STM32
boards. The launch sequence staggers participants to avoid saturating the
STM32's thread pool queue and lwIP pbuf pool:

### Stagger delays

| Machine | Nodes | Per-node delay | Total | Notes |
|---------|-------|----------------|-------|-------|
| RPi | 8 | **2 s** (was 1 s) | ~16 s | Doubled to halve SPDP burst rate |
| Jetson | 4 | 3+2+2 s | ~7 s | Unchanged — already well-spaced |
| Base PC | 2 | minimal | ~2 s | Only 2 nodes — low burst |

### Inter-machine delays in master script

| Transition | Delay | Rationale |
|------------|-------|-----------|
| RPi → Jetson | **20 s** (was 1 s) | Wait for all 8 RPi nodes to finish SPDP. RPi stagger = 16 s + 4 s margin. |
| Jetson → Base | **10 s** (was 1 s) | Wait for all 4 Jetson nodes to finish SPDP. Jetson stagger = 7 s + 3 s margin. |
| Base → topic probe | **5 s** (was 2 s) | Let 2 base nodes announce before probing STM32 topics. |

**Total node launch window: ~35 s** (was ~4 s). The extra 30 s of startup
time prevents overlapping discovery bursts that saturated the STM32's
`THREAD_POOL_WORKLOAD_QUEUE_LENGTH=40` and caused `wait_stm32_topics` to
spin indefinitely.

### Step 3 — `wait_stm32_topics()` fix

**Problem:** The old implementation used sequential `ros2 topic hz` calls —
each creating an ephemeral DDS participant with a fresh SPDP→SEDP cycle.
This was the same participant churn problem fixed in `check_connectivity.sh`.

**Fix:** Replaced with parallel `ros2 topic echo --once` (same technique as
`check_connectivity.sh` step 5). Both topics are checked simultaneously in
background processes, sharing a single discovery window. Up to 6 retries
with increasing timeout (12/18/24/30/36/42 s) to handle worst-case discovery
convergence during full 16-participant POC.

---

## 8. Build System

Both firmware trees use Docker-based builds:

```bash
cd ~/almondmatcha/mros2-mbed-chassis-dynamics
sudo ./build.bash rebuild NUCLEO_F767ZI chassis_controller

cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash rebuild NUCLEO_F767ZI sensors_node
```

Flash via OpenOCD with ST-LINK:
```bash
# Sensors board (serial number known)
sudo openocd -f interface/stlink.cfg \
  -c "adapter serial 066DFF3932504E3043014542" \
  -f target/stm32f7x.cfg \
  -c "program build/NUCLEO_F767ZI/sensors_node/sensors_node.bin 0x08000000 verify reset exit"

# Chassis board (use other serial or default if only one ST-LINK connected)
sudo openocd -f interface/stlink.cfg \
  -f target/stm32f7x.cfg \
  -c "program build/NUCLEO_F767ZI/chassis_controller/chassis_controller.bin 0x08000000 verify reset exit"
```

---

## 9. Verified Working State

As of April 12, 2026, with all 5 patches applied:

- `ros2 topic list` shows `/tpc_chassis_imu`, `/tpc_chassis_sensors`, `/tpc_chassis_cmd`
- `ros2 topic info /tpc_chassis_imu` → Publisher count: 1
- `ros2 topic info /tpc_chassis_sensors` → Publisher count: 1
- `ros2 topic echo /tpc_chassis_sensors` → data flowing (28.5V, encoders, current)
- `ros2 topic echo /tpc_chassis_imu` → data flowing (accel_z ≈ 1003 ≈ 1g)
- SPDP multicast visible from both boards via `tcpdump`
- No HardFaults, no MemoryPool overflow, stable heap/stack
