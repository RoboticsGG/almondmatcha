# Bug Investigation Log — Almondmatcha Rover Firmware

**System:** ROS2 Humble (rmw_fastrtps_cpp) + mROS2-mbed v0.5.4 on STM32 NUCLEO-F767ZI  
**Period:** November 2025 – May 2026  
**Boards:** chassis-dynamics (192.168.1.2), sensors-gnss (192.168.1.6)

This document records the investigation and resolution of all significant firmware
and system bugs found during development of the distributed ROS2 rover.  It is
written for reproducibility — someone encountering the same hardware/software stack
should be able to diagnose equivalent problems faster.

---

## 1 — STM32 HardFault During DDS Discovery Burst

**Dates:** November – December 2025  
**Commits:** `bf454bd`, `c1600f1`, `50c080f`, `6c56281`, `ff8f15f`, `c245144`, `5bdde37`  
**Patches:** `001` through `005` in `platform/patches/`

### Symptom

Both STM32 boards hard-faulted every time three or more Linux nodes joined domain 5
simultaneously (RPi, Jetson, base PC). The crash was perfectly reproducible — runs
012 through 015 produced identical register dumps.

```
HardFault Handler triggered
PC: 0x080197AF  LR: 0x080195AF   <- doCopyAndMoveOn() in MessageTypes.cpp
R0: 0x2003E36C  R1: 0xD7087EC3   <- R1 = source pointer passed to memcpy
R5: 2           R6: 1            <- copy size 2 bytes, 2nd submessage
```

Both boards crashed at the same logical point but with board-specific garbage
pointers: `0xD7087EC3` (chassis) and `0x4C8B0E83` (sensors). These addresses
fall in FMC peripheral space — clearly wrong for a data packet pointer.

**Key diagnostic observation:** R1 was *identical* across four independent runs.
Because the stack base shifts between rebuilds, a stack-derived corruption would
produce different addresses each time. Identical values meant the garbage pointer
was absolute — it came from CDR payload bytes being misinterpreted as a memory
address.

Stack depth at crash: 184 of 8192 bytes (2.2%). Not a stack overflow.

### Investigation

The crash was inside `doCopyAndMoveOn()`, which memcpy's RTPS submessage fields
using a `currentPos` pointer that advances through an incoming packet buffer.
R6 = 1 across all dumps, meaning the crash always occurred on the **second
submessage** in the packet — after the first had been processed successfully.

Processing the first submessage fires a deep callback chain:
`SPDPAgent::handleSPDPPackage → processProxyData → addProxiesForBuiltInEndpoints`

After those callbacks returned, `msgInfo.data` (the base pointer for the second
submessage) was corrupted to an FMC address.

The crash did **not** occur on the multi-domain `main` branch where nodes were spread
across domains 4, 5, and 6 — only when all nodes shared domain 5 and triggered a
larger simultaneous discovery burst.

### Root Cause Chain (5 patches, applied together)

**Bug A — `octetsToNextHeader == 0` misparse (Patch 001, PRIMARY CAUSE)**

The RTPS spec §8.3.3.2.3 allows the last submessage to set `octetsToNextHeader = 0`
meaning "extends to end of message." FastDDS uses this for SEDP DATA packets.

The original code advanced `nextPos` by only 4 bytes when the field was 0. The
subsequent loop iteration re-read CDR payload bytes (SEDP endpoint IP addresses) as
a submessage header. `currentPos` then landed at an IP address value —
`0xD7087EC3` and `0x4C8B0E83` are exactly the bytes of the endpoint locator IPs
present in every SEDP packet from the rover LAN.

Fix: set `msgInfo.nextPos = msgInfo.size` when `octetsToNextHeader == 0`.

**Bug B — AckNack bitmap overflow (Patch 001)**

`SequenceNumberSet::bitMap` was `array<uint32_t, 1>` (4 bytes). The RTPS spec allows
256-bit AckNack bitmaps; FastDDS always sends 256-bit. Every AckNack reception wrote
28 bytes past the end of a 4-byte field.

Fix: expand to `array<uint32_t, 8>`. Add `numBits > 256` bounds check.

**Bug C — TopicData name buffer overflow (Patch 001)**

`readFromUcdrBuffer()` passed wire-length `topicNameLength` directly to
`ucdr_deserialize_array_char()` without checking against `MAX_TOPICNAME_LENGTH` (40).
A FastDDS node with a name longer than 40 chars overflowed the buffer.

Fix: bounds-check before deserialize; skip oversized names.

**Bug D — Receive buffer not isolated from pbuf lifecycle (Patch 002)**

`MessageReceiver` parsed directly from the lwIP `pbuf` payload pointer. If lwIP
reused or freed the pbuf while SPDP callbacks were running (all on the same
thread), the parser's `currentPos` pointed into freed memory.

Fix: copy packet into a dedicated 1536-byte BSS buffer at `processMessage()` entry.
Packets larger than 1536 bytes (impossible in non-fragmented RTPS) are rejected.

**Bug E — SPDP packet corruption: `PBUF_TRANSPORT` offset (Patch 004)**

`lwip_pbuf_alloc(PBUF_TRANSPORT)` prepends a 50-byte transport header inside the
payload area. FastDDS received SPDP PDUs with 50 bytes of garbage before the CDR
payload.

Fix: use `PBUF_RAW`. CDR payload starts at offset 0.

**Bug F — `isMulticastAddress()` byte-order bug (Patch 005, FINAL CAUSE of "Publisher count: 0")**

Even after Patches 001–004 stopped the crash, `ros2 topic info` showed
`Publisher count: 0`. SEDP discovery never completed.

The culprit was in `UdpDriver.cpp`:

```cpp
// WRONG — assumes host byte order
return (addr.addr >> 28) == 14;
```

For `239.255.0.1`, lwIP stores the address in network byte order on little-endian
ARM: `transformIP4ToU32(239,255,0,1) = 0x0100FFEF`. Shifting right by 28 gives 0,
not 14. The multicast check always returned false, so `PID_METATRAFFIC_MULTICAST_LOCATOR`
entries in FastDDS SPDP announcements were silently discarded.

Fix: `ip4_addr_ismulticast(&addr)` — lwIP's own byte-order-correct predicate.

**Bug G — Operator precedence in `readLocatorIntoList()` (Patch 005)**

```cpp
// WRONG — && binds tighter than ||
ret && locator.isSameSubnet() || locator.isMulticastAddress()

// Correct
ret && (locator.isSameSubnet() || locator.isMulticastAddress())
```

With Bug F always returning false for multicast, only the `isSameSubnet()` path
worked — and the metatraffic multicast locator (239.255.0.1) is on a different
subnet from 192.168.1.x. Both bugs together meant `m_metatrafficMulticastLocatorList`
was always empty, and SEDP was sent to a wrong derived port.

**Combined effect of both Patch 005 bugs:** multicast locators were stored correctly
→ SEDP went to 239.255.0.1:8650 → FastDDS heard it → `Publisher count: 1` ✓

---

## 2 — RTPS Memory Pool Exhaustion

**Date:** November 2025  
**Commits:** `2f3d7fa`, `893a02b`, `24bda75`  
**File:** `platform/rtps/config.h`

### Symptom

```
[MemoryPool] RESSOURCE LIMIT EXCEEDED
```

Appeared intermittently during discovery. Some nodes were matched, others were not.
Recovery required a full system reset.

### Investigation

Three independent pool limits were exhausted:

**SPDP_MAX_NUMBER_FOUND_PARTICIPANTS = 19 (original default = 14):**
`ros2` CLI tools (`ros2 topic list`, `ros2 node list`) each create a short-lived
DDS participant. With a router daemon, two SBCs, STM32 boards, and CLI tool
participants, the count exceeded 19. When the table was full, STM32 silently
dropped new SPDP announcements. The original error message `[MemoryPool] RESSOURCE
LIMIT EXCEEDED` gave no context (no size, no type). Patch 003 added `(size=30,
used=30)` so the source was immediately visible.

**NUM_WRITER_PROXIES_PER_READER = 6 and NUM_READER_PROXIES_PER_WRITER = 6:**
Each D5 publisher needs a proxy slot on every subscriber's reader, and vice versa.
With 11 D5 nodes, 6 slots per writer/reader was exhausted during SEDP burst.

**THREAD_POOL_READER_STACKSIZE = 4096 → HardFault:**
The reader thread call chain during SEDP burst consumed ~4080 of 4096 bytes (99.6%).
During the pool error path, a recursive resend triggered the fatal overflow.

Stack at HardFault, bottom to top:
```
Thread_PoolThread()            ~100 B
  workload.callback()           ~80 B
    DataReader::onNewData()     ~60 B
      processDataMsg()         ~120 B
        deserialize()          ~200 B
          allocateFromPool()    ~60 B
            [pool error path] variable
```

### Fix

| Parameter | Before | After |
|-----------|--------|-------|
| `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS` | 14 → 19 | 30 |
| `NUM_WRITER_PROXIES_PER_READER` | 6 | 30 |
| `NUM_READER_PROXIES_PER_WRITER` | 6 | 30 |
| `THREAD_POOL_READER_STACKSIZE` | 4096 | 8192 |
| `MAX_NUM_PARTICIPANTS` | 15 | 1 |

The last entry deserves explanation. The default `MAX_NUM_PARTICIPANTS = 15`
allocates a *pool* of 15 local participant objects, each containing its own proxy
arrays:

```
15 × (16 writers × 30 proxies × ~80 B) ≈ 576 KB > 512 KB SRAM
```

The board hard-faulted at boot before lwIP initialized. Setting
`MAX_NUM_PARTICIPANTS = 1` (one local participant, which is all mROS2 ever
creates) saves ~532 KB of BSS.

---

## 3 — lwIP pbuf Pool Exhausted During Discovery

**Date:** November 2025  
**File:** `mbed_app.json` (`lwip.pbuf-pool-size`)

### Symptom

Intermittent missed SPDP announcements during burst. Topics appeared eventually
but with a 10–30 second delay. Some participants were never fully discovered.

### Investigation

During DDS discovery, 30+ RTPS packets arrive simultaneously. Each packet requires
a `pbuf` allocation. The default `lwip.pbuf-pool-size = 16` exhausted quickly.
When `pbuf_alloc()` returned NULL, the packet was silently dropped — no error, no
log. The symptom was indistinguishable from a discovery timing problem.

The fix was identified by cross-referencing lwIP source: `udp_input()` calls
`pbuf_alloc(PBUF_RAW)` for each incoming datagram. With pool_size = 16 and 30+
simultaneous SPDP packets, 14+ packets were dropped each discovery cycle.

### Fix

```json
"lwip.pbuf-pool-size": 32
```

Pool size 32 covers the worst-case simultaneous burst with margin. Values above 64
begin consuming lwIP heap from the same 512 KB SRAM pool.

---

## 4 — FastDDS Binding to WiFi on Dual-NIC Host

**Date:** November 2025  
**File:** `ws_base/fastdds_base.xml`

### Symptom

`ros2 topic list` on the base PC showed no STM32 topics. The STM32 boards showed
all other nodes discovered. The base PC showed the STM32 as discovered.
Network-level capture showed SPDP multicast from STM32 arriving on `enp0s31f6`
(192.168.1.4, rover Ethernet) but not on `wlp4s0` (WiFi).

### Investigation

FastDDS selects its network interface using the default route. The base PC's default
route was via `wlp4s0` (WiFi, for internet access). FastDDS announced its SPDP
locators with the WiFi IP (`10.x.x.x`) rather than the rover LAN IP (`192.168.1.4`).
STM32 boards received the SPDP from a non-routable address and could not send unicast
SEDP back.

Verified with tcpdump:
```bash
# SPDP from STM32 arrives on Ethernet (port 8650 = 7400 + 250×5 for domain 5)
sudo tcpdump -i enp0s31f6 'dst host 239.255.0.1 and udp port 8650' -nn -c 3
```

### Fix

Created `ws_base/fastdds_base.xml` pinning the network interface by subnet:

```xml
<transport_descriptors>
  <transport_descriptor>
    <transport_id>CustomUDPTransport</transport_id>
    <type>UDPv4</type>
    <interfaceWhiteList>
      <address>192.168.1.</address>  <!-- matches rover Ethernet only -->
    </interfaceWhiteList>
  </transport_descriptor>
</transport_descriptors>
```

Required env var (add to `~/.bashrc`):
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/almondmatcha/ws_base/fastdds_base.xml
```

The ROS2 daemon must be restarted after setting the variable — it reads the profile
at startup and caches the selected interface for the session.

---

## 5 — NMEA Data Loss: UnbufferedSerial Drops Bytes

**Date:** April 2026  
**Commit:** `38b8a7b`  
**File:** `mros2-mbed-sensors-gnss/workspace/sensors_node/gnss_reader.cpp`

### Symptom

NMEA sentences from the u-blox GNSS module arrived truncated or partially missing.
`$GNGGA` sentences were cut mid-field; latitude/longitude parsing returned NaN.
The problem was intermittent and worse during periods of heavy ROS2 DDS traffic.

### Investigation

`gnss_reader.cpp` used `UnbufferedSerial` — a non-buffered UART that requires the
application to read bytes as they arrive with no hardware FIFO backup beyond the
single-byte UART shift register. NMEA sentences arrive at 9600 baud (nominally
~0.96 ms per byte). When the ROS2 spin loop or RTPS callbacks held the CPU for
>1 ms, incoming bytes were overrun.

The issue was confirmed by adding a byte-drop counter: every period of DDS
activity (especially SEDP heartbeat cycles) coincided with truncated sentences.

### Fix

Switched to `BufferedSerial` which uses a DMA-backed or IRQ-driven ring buffer,
decoupling the read timing from the application loop. Buffer size set to 256 bytes
(sufficient for 3–4 complete NMEA sentences at 9600 baud).

```cpp
// Before
UnbufferedSerial gnss_uart(PA_9, PA_10, 9600);

// After
BufferedSerial gnss_uart(PA_9, PA_10, 9600);
```

---

## 6 — lwIP UDP Statistics Always Zero

**Date:** May 9, 2026  
**Commit:** `bb0bbc1`  
**Files:** `mbed_app.json` (both boards), `memory_reporter.h` (both boards), `lwipopts.h` (both boards)

### Symptom

`udp_recv`, `udp_xmit`, and `udp_drop_pct` were always 0 in the STM32 serial JSON,
even during active DDS communication. The `cpu_busy_pct` field was also always 0.

### Investigation: Bug A — Invalid `mbed_app.json` Configuration Key

`mbed_app.json` contained:
```json
"lwip.stats-enabled": true
```

This looks like a valid Mbed configuration key but is not — no `mbed_lib.json` in
the lwIP stack defines `lwip.stats-enabled`. Mbed silently ignores unknown keys.
The result was that `lwipopts.h`'s hardcoded `#define LWIP_STATS 0` remained in
effect, and the entire `#if _MR_LWIP_STATS` block in `memory_reporter.h` was
compiled out. All UDP stat fields were assigned to zero at compile time.

### Investigation: Bug B — Wrong Struct Field Name

Even if `LWIP_STATS` had been enabled, `memory_reporter.h` referenced:
```c
lwip_stats.udp.sent   // field does not exist
```

The correct lwIP `stats_proto` struct field for transmitted datagrams is:
```c
lwip_stats.udp.xmit   // incremented at lwip_udp.c:907 via UDP_STATS_INC(udp.xmit)
```

`recv` and `drop` were already correct field names.

### Fix

Three changes required:

1. `mbed_app.json` (both boards): remove `"lwip.stats-enabled": true`, add
   `"LWIP_STATS=1"` to the `macros[]` array so the compiler sees `-DLWIP_STATS=1`.

2. `lwipopts.h` (both boards): wrap `#define LWIP_STATS 0` in `#ifndef LWIP_STATS`
   so the command-line macro is not silently overridden by the vendor header.

3. `memory_reporter.h` (both boards): `lwip_stats.udp.sent` → `lwip_stats.udp.xmit`.

---

## 7 — CPU Busy Percentage Always Zero

**Date:** May 9, 2026  
**Commit:** `2414ecb`, `4d7bd19`  
**File:** `memory_reporter.h` (both boards)

### Symptom

`cpu_busy_pct` reported 0.0 continuously even during high-load periods (SEDP bursts,
active motor control). `cpu_idle_pct` was 100.0 or clamped near it.

### Investigation

The formula in `memory_reporter.h`:

```cpp
uint64_t d_up    = mbed_uptime();
uint64_t d_idle  = mbed_time_idle();
uint64_t d_sleep = mbed_time_sleep();
uint64_t d_deep  = mbed_time_deep_sleep();

uint64_t d_off   = d_idle + d_sleep + d_deep;
float cpu_busy   = (float)(d_up - d_off) / d_up * 100.0f;
```

`mbed_time_idle()` is defined in `mbed_power_mgmt.c` as:
```c
return (sleep_time + deep_sleep_time);
```

It already returns the sum of sleep and deep sleep. The formula was therefore
triple-counting every sleep period: `d_off = d_idle + d_sleep + d_deep`
= `(sleep + deep_sleep) + sleep + deep_sleep` = `2×sleep + 2×deep_sleep`.

Since `d_off` was always >= `d_up`, the clamp set `d_off = d_up`, giving
`cpu_busy = 0.0f` every time.

### Fix

```cpp
// Correct: idle_time already covers all sleep
float cpu_busy_pct = (float)(d_up - d_idle) / d_up * 100.0f;
```

Also changed format specifier from `%.1f` to `%.2f` for sub-1% resolution — nodes
spending ~1 ms per 250 ms cycle are genuinely 0.4% busy and integer math rounded
that to 0.

---

## 8 — Serial Output Collision Between Tasks

**Date:** May 9, 2026  
**Commit:** `4d7bd19`  
**Files:** `memory_reporter.h` (both boards), `app.cpp` (both boards)

### Symptom

The JSON serial output from the STM32 memory reporter was interleaved with the
heartbeat `[HB#...]` print lines from the main application task:

```
{"type":"STM32_STATS","node":"Rover[HB#42 imu] chassis: X=12 Y=-3 Z=998
WithIMU","ts_ms":15042,...}
```

The `collect_stm32_memory.py` JSON parser rejected these mixed lines, recording
`[CORRUPT_JSON]` markers in the CSV.

### Investigation

`memory_reporter.h` ran a background thread (`_mr_task`) that wrote a JSON line to
`stdout` via `fwrite()`. The main task's `MROS2_DEBUG` heartbeat print used
`printf()`. Both calls ultimately wrote to the same UART handle via the Mbed stdio
layer, but neither held a lock. With two RTOS threads at different priorities,
preemption could occur mid-write.

### Fix

Added a `Mutex` to `memory_reporter.h` and a lock/unlock API:

```cpp
Mutex _mr_stdout_mtx;
inline void mr_serial_lock()   { _mr_stdout_mtx.lock(); }
inline void mr_serial_unlock() { _mr_stdout_mtx.unlock(); }
```

Both `_mr_task()` and the `MROS2_DEBUG` heartbeat blocks in `app.cpp` acquire the
mutex before writing. This prevents interleaving.

---

## 9 — Static Mutex Per-Translation-Unit Bug

**Date:** May 2026  
**Commit:** `512053f` (single-domain), `a1f620a` (multi-domain)  
**File:** `memory_reporter.h` (both boards)

### Symptom

After Bug 8 was fixed (Commit `4d7bd19`), interleaved JSON output still appeared
occasionally, specifically when `gnss_reader.cpp` was printing NMEA debug lines
simultaneously with `_mr_task()`. The mutex appeared to have no effect between
these two modules.

### Investigation

`_mr_stdout_mtx` was declared as `static Mutex` at file scope inside a header:

```cpp
// memory_reporter.h
static Mutex _mr_stdout_mtx;
```

In C++, `static` at namespace scope inside a header means *each translation unit
that includes the header gets its own independent copy of the variable*. The
`memory_reporter.h` was included by both `app.cpp` and `gnss_reader.cpp`.
`app.cpp`'s `_mr_stdout_mtx` and `gnss_reader.cpp`'s `_mr_stdout_mtx` were two
completely separate `Mutex` objects — calling `mr_serial_lock()` in `gnss_reader.cpp`
locked a different mutex than `_mr_task()` in `app.cpp`. No actual mutual exclusion.

The bug was dormant in `memory_reporter.h` from the moment the mutex was introduced;
it only surfaced when `gnss_reader.cpp` started calling `mr_serial_lock()`.

### Fix

Replace the static variable with a function-local static, which has exactly-once
initialization across all translation units (C++11 §6.7):

```cpp
// Before — per-TU copy (wrong)
static Mutex _mr_stdout_mtx;
inline void mr_serial_lock()   { _mr_stdout_mtx.lock(); }
inline void mr_serial_unlock() { _mr_stdout_mtx.unlock(); }

// After — single instance via function-local static (correct)
inline Mutex& mr_serial_mutex() { static Mutex m; return m; }
inline void mr_serial_lock()   { mr_serial_mutex().lock(); }
inline void mr_serial_unlock() { mr_serial_mutex().unlock(); }
```

Also updated `_mr_task()` to call `mr_serial_lock()`/`mr_serial_unlock()` instead
of direct `_mr_stdout_mtx.lock()`.

The function-local static is initialized the first time `mr_serial_mutex()` is
called. Every subsequent call returns a reference to the same object regardless of
which `.cpp` file calls it.

---

## Summary Table

| # | Bug | Root Cause | When Found | Commit(s) |
|---|-----|-----------|-----------|-----------|
| 1 | STM32 HardFault (discovery burst) | `octetsToNextHeader=0` misparse + 6 related bugs | Nov 2025 | `bf454bd`…`5bdde37` |
| 2 | RTPS pool exhaustion | SPDP_MAX=14, proxy pool=6, reader stack=4096 | Nov 2025 | `2f3d7fa`, `893a02b` |
| 3 | lwIP pbuf pool exhausted | Default pool size 16 < burst packet count | Nov 2025 | in `mbed_app.json` |
| 4 | FastDDS binds to WiFi | Default route selects wrong NIC on dual-NIC host | Nov 2025 | `fastdds_base.xml` |
| 5 | NMEA data loss | `UnbufferedSerial` drops bytes during DDS activity | Apr 2026 | `38b8a7b` |
| 6 | UDP stats always 0 | Invalid mbed config key + wrong struct field name | May 2026 | `bb0bbc1` |
| 7 | CPU% always 0 | Triple-counted sleep time in busy-% formula | May 2026 | `2414ecb` |
| 8 | Serial output collision | No mutex between RTOS tasks writing to UART | May 2026 | `4d7bd19` |
| 9 | Mutex per-TU isolation failure | `static` in header = per-TU copies, not shared | May 2026 | `512053f` |

---

**Full embeddedRTPS patch details:** [EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md)  
**Current config values:** [RTPS_CONFIG_REFERENCE.md](RTPS_CONFIG_REFERENCE.md)  
**Memory budget worksheet:** [STM32_RTPS_MEMORY_CALCULATION.md](STM32_RTPS_MEMORY_CALCULATION.md)
