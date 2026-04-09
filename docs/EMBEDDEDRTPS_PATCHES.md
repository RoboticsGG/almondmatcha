# embeddedRTPS Patches — Root Cause Analysis & Fixes

## Patch files

| File | Description |
|------|-------------|
| `platform/patches/001-rtps-deserialization-fixes.patch` | AckNack bitmap overflow, `octetsToNextHeader==0` misparse, TopicData name overflow |
| `platform/patches/002-rtps-hardfault-prevention.patch` | Receive buffer isolation, guard-word corruption detection, NUCLEO-F767ZI pointer validation |
| `platform/patches/003-discovery-pool-diagnostic.patch` | MemoryPool error message with size/used counters; SEDP_VERBOSE silenced |
| `platform/patches/004-spdp-pbuf-and-sedp-locator-fallback.patch` | SPDP packet corruption fix (PBUF_RAW); SEDP locator fallback chain; SEDP immediate resend |
| `platform/patches/005-multicast-address-detection-fix.patch` | `isMulticastAddress()` byte-order bug; `readLocatorIntoList()` operator precedence bug |

Applied to `mros2/embeddedRTPS` (cloned from mROS-base/mros2 v0.5.4) by
`build.bash` after clone. Same patches used in both firmware trees.

---

## Crash signature (runs 012–015, ALL identical)

Both STM32 boards (NUCLEO-F767ZI) HardFault during DDS discovery burst when
3 Linux nodes join domain 5 (RPi, Jetson, base PC).  The crash is in
`memcpy` ← `doCopyAndMoveOn()` in `MessageTypes.cpp:33`.

| Register | Chassis (all runs) | Sensors (all runs) | Meaning |
|----------|--------------------|--------------------|---------|
| R1 (src) | 0xD7087EC3 | 0x4C8B0E83 | Garbage pointer (corrupted `currentPos`) |
| R5 (size) | 2 | 4 | Copy size (uint16_t / uint32_t field) |
| R6 | 1 | 1 | Likely loop iteration (2nd submessage) |
| R7 | 0x2003E36C | 0x2003E93C | Static data ptr (this/MessageReceiver) |
| LR | 0x080197AF | 0x080195AF | doCopyAndMoveOn return-from-memcpy |
| UFSR | 0x0100 | 0x0000 | UNALIGNED (chassis) |
| BFSR | 0x00 | 0x82 | PRECISERR+BFARVALID (sensors) |
| BFAR | — | 0x4C8B0E83 | BusFault address (sensors) |

**Key observation:** R1 is IDENTICAL across runs 012–015 despite stack position
and firmware shifting between rebuilds.  R0/R4/SP shift in lockstep with the
stack base (heap allocation).  R7 is a fixed BSS address.  This means the
corrupted pointer value is ABSOLUTE — not stack-relative.

**Stack depth at crash:** 184 bytes of 8192 (2.2%).  No stack overflow.
Peak depth during SPDP/SEDP callback chain is ~550 bytes (6.7% of 8192).

**Fault types:**
- Chassis: UFSR=0x0100 → UNALIGNED UsageFault (odd address 0xD7087EC3)
- Sensors: BFSR=0x82 → Precise data BusFault (unmapped address 0x4C8B0E83)

The crash is deterministic because the same FastDDS SPDP/SEDP payloads arrive
every run (same nodes, same topics, same IPs).  The corrupted `data` pointer
in `MessageProcessingInfo` causes `currentPos` to land at the same garbage
address each time.

---

## Root cause hypothesis (in progress)

The `msgInfo.data` pointer in `processMessage()` becomes corrupted between the
1st and 2nd iterations of the submessage parsing loop.  R6=1 across all crash
dumps suggests the crash always occurs on the **2nd submessage**.

The 1st submessage (likely SPDP DATA) is processed successfully, firing deep
callback chains (SPDPAgent::handleSPDPPackage → processProxyData →
addProxiesForBuiltInEndpoints → addNewMatchedWriter).  After these callbacks
return, `msgInfo.data` has been corrupted to a value that, when combined with
`nextPos`, produces an address in the FMC/peripheral memory range.

**Possible corruption mechanisms under investigation:**
1. **Compiler register allocation:** If the compiler stores `msgInfo.data` in a
   callee-saved register (R6-R11) and a callee function fails to properly
   save/restore it.  R7=0x2003E36C (static address) is constant — it is NOT
   the data pointer.
2. **Callback side-effect on stack:** The SPDP callback chain acquires mutexes
   and modifies participant/endpoint data structures.  A subtle bug in one of
   these (e.g., writing past a MemoryPool slot) could corrupt the reader
   thread's stack frame.
3. **Single-domain participant exhaustion:** With ALL nodes on domain 5, the
   number of SPDP/SEDP messages during discovery burst is significantly higher
   than in the multi-domain (main branch) setup that didn't crash.  Although
   MAX_NUM_PARTICIPANTS=20 and MemoryPool bounds are checked, resource
   exhaustion could trigger unexpected code paths.

**This crash did NOT occur on the multi-domain `main` branch** where nodes
were spread across domains 0, 5, and 10.

---

## Patch 002 — HardFault prevention (defense-in-depth, NUCLEO-F767ZI only)

**Files:** `include/rtps/messages/MessageReceiver.h`,
`src/messages/MessageReceiver.cpp`, `src/messages/MessageTypes.cpp`

**Also:** `platform/rtps/config.h` — `THREAD_POOL_WORKLOAD_QUEUE_LENGTH`
increased from 20 to 40 (tracked in git, not part of the patch file).

Three layers protect against the deterministic HardFault during discovery
burst:

### Layer 1 — Receive buffer isolation

`MessageReceiver` gets a 1536-byte dedicated receive buffer (`m_recvBuffer`)
in BSS.  `processMessage()` copies the incoming packet from the lwip pbuf
payload into this buffer before parsing.  This decouples the parser from the
pbuf lifecycle — even if the pbuf is freed/reused by another thread, the data
remains valid.

The buffer is large enough for any Ethernet-size RTPS message.  Messages
exceeding 1536 bytes are rejected (these should never appear in non-
fragmented unicast RTPS traffic).

### Layer 2 — Guard-word corruption detection

Two volatile `uint32_t` guard words (`m_guardPre`, `m_guardPost`) bracket the
receive buffer.  Both are set to `0xDEADC0DE` at the start of
`processMessage()`.  After each submessage callback returns — the exact point
where corruption was observed in runs 012–015 — the guards are checked.  If
either has changed, processing is aborted immediately.

### Layer 3 — NUCLEO-F767ZI pointer validation

`isValidReadPtr()` in `MessageTypes.cpp` validates that the source pointer
passed to `doCopyAndMoveOn()`/`memcpy` falls within the STM32F767ZI memory
map:

| Region | Address range | Size |
|--------|--------------|------|
| DTCM RAM | 0x20000000 – 0x2001FFFF | 128 KB |
| SRAM1 | 0x20020000 – 0x2007BFFF | 368 KB |
| SRAM2 | 0x2007C000 – 0x2007FFFF | 16 KB |
| Flash | 0x08000000 – 0x081FFFFF | 2 MB |

No external SDRAM/FMC.  Any pointer outside these ranges (e.g., the crash
value 0xD7087EC3 in FMC bank space) is caught and the copy is skipped.

### Queue length increase

`THREAD_POOL_WORKLOAD_QUEUE_LENGTH` doubled from 20 to 40 in both boards'
`config.h`.  This gives the incoming packet circular buffer more room during
discovery bursts when 3 Linux nodes simultaneously announce SPDP/SEDP
endpoints.  Each slot holds one `PacketInfo` with a `PBufWrapper` reference.

---

## Patch 001 — Deserialization fixes

### Bug A — `octetsToNextHeader == 0` mishandling

**File:** `src/messages/MessageReceiver.cpp`

The RTPS spec allows the last submessage to set `octetsToNextHeader = 0`
("extends to end of message").  The original code advanced `nextPos` by only
4 bytes → subsequent loop iterations re-interpreted body data as submessage
headers, causing cascading misparse.

**Fix:** Set `msgInfo.nextPos = msgInfo.size` when `octetsToNextHeader == 0`.
Also added overflow guard and fixed `processDataSubmessage()` size underflow.

**Note:** This was initially believed to be the primary root cause but the
crash persisted through run_015 unchanged.  The fix is still correct and
prevents a different class of misparse crashes.

### Bug B — TopicData name buffer overflow

**File:** `src/discovery/TopicData.cpp`

`readFromUcdrBuffer()` passed unclamped `topicNameLength`/`typeNameLength`
(from wire) to `ucdr_deserialize_array_char()`.  If a FastDDS node registers
topic/type names longer than `MAX_TOPICNAME_LENGTH` (40) or
`MAX_TYPENAME_LENGTH` (60), buffer overflow occurs.

**Fix:** Added bounds check; oversized names are skipped.

### Bug C — AckNack bitmap overflow

**Files:** `include/rtps/common/types.h`, `src/messages/MessageTypes.cpp`

`SequenceNumberSet::bitMap` was `std::array<uint32_t, 1>` (4 bytes) but
FastDDS sends up to 256-bit AckNack bitmaps per RTPS spec.

**Fix:** Expanded to `std::array<uint32_t, 8>`, fixed formula, added
`numBits > 256` bounds check.

---

## Upstream embeddedRTPS gap analysis

The mros2 fork (`de70e01`) diverged from upstream
`embedded-software-laboratory/embeddedRTPS` at commit `1410a87` (May 2022).
The upstream has 8+ commits the fork is missing:

| Commit | Date | Description | Relevant? |
|--------|------|-------------|-----------|
| `00ed575` | 2024-04 | **fix: make locks not unlock immediately** | NO — mros2 fork already uses named Lock variables |
| `866a71f` | 2023-08 | Feature/discovery rework | MAYBE — major discovery changes |
| `2e6ca4b` | 2023-02 | Randomized GUIDs, History >256 | LOW |
| `96060d9` | 2022-11 | `at()` bounds check (REVERTED in 9f28d30) | NO |
| `1fd03aa` | 2022-08 | Consistent Mutex Usage | LOW — mros2 has correct usage |
| `c05ca9f` | 2022-08 | Skip full mempool buckets | MAYBE |
| `4f49cf9` | 2022-08 | Feature/endpoint deletion | LOW |

**Recommendation:** Do NOT bump the embeddedRTPS version directly.  The mros2
fork adds fragmented message support (`9ac80c3`) not present upstream, and
the discovery rework (`866a71f`) is a large, risky change.  Instead,
cherry-pick specific fixes as needed via patches.

---

## Patch 003 — Discovery pool diagnostic + SEDP verbose silencing

**Files:** `include/rtps/storages/MemoryPool.h`, `include/rtps/utils/Log.h`

### MemoryPool error message

The original `printf("[MemoryPool] RESSOURCE LIMIT EXCEEDED \n")` gave no
context about which pool or how full it was.  The fix adds size and used
counters:

```
[MemoryPool] RESSOURCE LIMIT EXCEEDED (size=30, used=30)
```

This was essential for diagnosing the `m_remoteParticipants` overflow in
runs 016–018 (SPDP_MAX was 19 but `ros2 topic hz` CLI tools each create
ephemeral DDS participants, exhausting the pool).

### SEDP_VERBOSE silenced

`SEDP_VERBOSE` changed from 1 to 0.  The SEDP verbose path prints each
endpoint match over serial UART at 115200 baud.  With 15 topics × 16 remote
nodes = ~50 endpoint pairs, the SEDP burst produces hundreds of log lines
within 5 seconds at startup, saturating the UART and blocking the reader
thread.  Silencing it has no functional effect and reduces discovery latency.

---

## Patch 004 — SPDP packet corruption + SEDP discovery fixes

**Files:** `include/rtps/storages/PBufWrapper.h`, `src/discovery/SPDPAgent.cpp`

This patch addresses three bugs that together prevented DDS discovery from
completing between STM32 boards and Linux FastDDS nodes.

### Bug A — SPDP packet corruption (PBUF_TRANSPORT → PBUF_RAW)

**File:** `include/rtps/storages/PBufWrapper.h`

**Symptom:** `ros2 topic info /tpc_chassis_imu` shows Publisher count: 0 and
`ros2 node list` does not show the STM32 `rover_node`.  A Python UDP
multicast receiver revealed the STM32 SPDP payload had 50 stale bytes
prefixed before the CDR encapsulation (`00 03 00 00`).

**Root cause:** `PBufWrapper` allocated all pbufs with `PBUF_TRANSPORT`.
lwIP's `pbuf_alloc(PBUF_TRANSPORT, size, PBUF_POOL)` reserves space for
network headers in every pbuf: 14 (ETH) + 20 (IP) + 8 (UDP) + 8 alignment =
50 bytes.  The reserved space is counted in `tot_len`, so
`PBufWrapper::spaceUsed()` returned 50 for an empty buffer.
`PBufWrapper::append()` called `pbuf_take_at(pbuf, data, length, 50)`,
writing the CDR payload at byte offset 50.  The first 50 bytes were stale
network headers from recycled lwIP pool buffers.

**Fix:** `PBUF_TRANSPORT` → `PBUF_RAW`.  `PBUF_RAW` reserves zero header
space, so `spaceUsed()` returns 0 for an empty buffer and `pbuf_take_at()`
writes at offset 0.

### Bug B — Missing metatraffic multicast locator fallback in SEDP

**File:** `src/discovery/SPDPAgent.cpp` — `addProxiesForBuiltInEndpoints()`

**Symptom:** SPDP from FastDDS does not contain `PID_METATRAFFIC_UNICAST_LOCATOR`
(0x0032).  `addProxiesForBuiltInEndpoints()` finds no metatraffic unicast
locator, returns `false`, and no SEDP builtin proxies are created.  The STM32
never sends unicast SEDP, and FastDDS never receives STM32 endpoint
announcements.

**Root cause (original):** `ws_base/fastdds_base.xml` sets explicit
`metatrafficMulticastLocatorList` (239.255.0.1:8650).  When this is
configured, FastDDS uses multicast-only metatraffic and omits
`PID_METATRAFFIC_UNICAST_LOCATOR` from its SPDP.  FastDDS also does NOT
bind any metatraffic unicast ports (8660, 8662, etc.) — confirmed by
`ss -ulnp` showing no listeners on those ports.

`addProxiesForBuiltInEndpoints()` only ever checked
`m_metatrafficUnicastLocatorList`.  FastDDS DOES include
`PID_METATRAFFIC_MULTICAST_LOCATOR` (0x0033) in SPDP, which embeddedRTPS
correctly parses into `m_metatrafficMulticastLocatorList` — but this field
was dead data, never read by any consumer.

**Fix:** Three-level fallback chain in `addProxiesForBuiltInEndpoints()`:

1. **Primary:** `m_metatrafficUnicastLocatorList` — standard RTPS unicast
   (isSameSubnet)
2. **Fallback 1:** `m_metatrafficMulticastLocatorList` — for FastDDS with
   explicit multicast metatraffic config (isMulticastAddress).  The STM32 now
   sends its SEDP to 239.255.0.1:8650, where all FastDDS nodes listen.
3. **Fallback 2:** Derive from `m_defaultUnicastLocatorList` with
   `port - 1` (D3−D1=1 in the RTPS port formula) — last-resort for DDS
   implementations that advertise neither metatraffic locator PID.

**Note:** Fallback 1 was non-functional until Patch 005.  `readLocatorIntoList()`
silently discarded multicast locators during SPDP parsing due to two bugs in
`isMulticastAddress()` and operator precedence (see Patch 005).  As a result,
`m_metatrafficMulticastLocatorList` was always empty and Fallback 1 never
fired.  Execution fell through to Fallback 2, which derived port 8660 — a port
no process listened on.  Patch 005 makes Fallback 1 effective.

### Bug C — Missing SEDP immediate resend for new participants

**File:** `src/discovery/SPDPAgent.cpp` — `processProxyData()`

**Symptom:** Even after the locator fallback fix, the experiment script's
`ros2 topic hz` 4-second timeout sometimes expired before SEDP data
arrived.

**Root cause:** `StatefulWriter::addNewMatchedReader()` does NOT mark existing
history cache entries as unsent for the new reader proxy.  Discovery relied
entirely on the heartbeat→ACKNACK→resend cycle.  With `SF_WRITER_HB_PERIOD_MS`
at 2000 ms and `ros2 topic hz` timeout at 4 s, the cycle often did not
complete in time.

**Fix:** After calling `addProxiesForBuiltInEndpoints()`, immediately call
`setAllChangesToUnsent()` on both SEDP writers (`sedpPubWriter`,
`sedpSubWriter`) in addition to the existing `spdpWriter` call.  This
enqueues a resend of all existing SEDP endpoint announcements to the newly
matched reader proxies via the threadpool, without waiting for the next
heartbeat cycle.

---

## Patch 005 — Multicast address detection fix

**Files:** `src/communication/UdpDriver.cpp`,
`src/discovery/ParticipantProxyData.cpp`

**Symptom (runs 019–022):** `ros2 topic info` still shows Publisher count: 0
despite Patch 004's SEDP locator fallback chain being in place.  Patch 004's
Fallback 1 (`m_metatrafficMulticastLocatorList`) was intended to direct SEDP
to 239.255.0.1:8650, but the field was silently empty every run.

### Bug A — `isMulticastAddress()` byte-order bug

**File:** `src/communication/UdpDriver.cpp`

**Root cause:** The original check:

```cpp
bool UdpDriver::isMulticastAddress(ip4_addr_t addr) {
  return ((addr.addr >> 28) == 14);
}
```

assumes `addr.addr` is in host byte order with the first octet in the MSB.
lwIP's `transformIP4ToU32()` stores `ip4_addr_t` in **little-endian network
order**: the first (most-significant) IP octet is placed in the LSB of the
`uint32_t`.  For 239.255.0.1 on the STM32 (a little-endian ARM Cortex-M7):

```
transformIP4ToU32(239, 255, 0, 1) = (1<<24)|(0<<16)|(255<<8)|239 = 0x0100FFEF
0x0100FFEF >> 28 = 0   ← NOT 14
```

This function returned `false` for every multicast address, including
239.255.0.1.

**Fix:** Replace the manual bit-shift check with lwIP's own macro, which is
byte-order-correct by construction:

```cpp
bool UdpDriver::isMulticastAddress(ip4_addr_t addr) {
  return ip4_addr_ismulticast(&addr);
}
```

`ip4_addr_ismulticast` is defined in `<lwip/ip4_addr.h>` (transitively
included via the existing `<lwip/igmp.h>`) as:

```c
#define ip4_addr_ismulticast(addr1) \
  (((addr1)->addr & PP_HTONL(0xf0000000UL)) == PP_HTONL(0xe0000000UL))
```

`PP_HTONL` converts the constant to network byte order at compile time,
making the comparison byte-order-neutral.

### Bug B — Operator precedence in `readLocatorIntoList()`

**File:** `src/discovery/ParticipantProxyData.cpp`

**Root cause:** The condition that guards storing a parsed locator:

```cpp
if (ret && full_length_locator.isSameSubnet() ||
    full_length_locator.isMulticastAddress())
```

parsed by C++ precedence rules (`&&` binds tighter than `||`) as:

```cpp
if ((ret && full_length_locator.isSameSubnet()) ||
    full_length_locator.isMulticastAddress())
```

With Bug A making `isMulticastAddress()` always return `false`, the locator
was only stored when `ret && isSameSubnet()`.  For the multicast address
239.255.0.1, `isSameSubnet()` returns `false` (it is not on the same /24
subnet as the board's IP), so the `PID_METATRAFFIC_MULTICAST_LOCATOR` from
FastDDS's SPDP was silently discarded by the `else { return true; }` branch.

**Fix:** Add parentheses to enforce the intended grouping:

```cpp
if (ret && (full_length_locator.isSameSubnet() ||
            full_length_locator.isMulticastAddress()))
```

### Chain of failure (patch 004 → patch 005)

```
FastDDS SPDP  ──► PID_METATRAFFIC_MULTICAST_LOCATOR = 239.255.0.1:8650
                              │
                  readLocatorIntoList()
                  isMulticastAddress() == false  ← Bug A
                  isSameSubnet()       == false
                  → locator discarded             ← Bug B
                              │
            m_metatrafficMulticastLocatorList = [ EMPTY ]
                              │
   addProxiesForBuiltInEndpoints():
     Fallback 1: empty list → skip
     Fallback 2: derive port 8660 → nobody listens
                              │
         SEDP sent to 192.168.1.4:8660 → dropped
                              │
         Publisher count: 0  (runs 019–022)
```

With Patch 005 applied, `isMulticastAddress()` correctly identifies
239.255.0.1 → the locator is stored → Fallback 1 finds 239.255.0.1:8650
→ SEDP is sent to the multicast metatraffic port where all FastDDS nodes
listen → discovery completes.

---

## config.h changes (tracked in git, not in patch files)

These are per-board `platform/rtps/config.h` values changed from defaults:

| Parameter | Old value | New value | Rationale |
|-----------|-----------|-----------|-----------|
| `MAX_NUM_PARTICIPANTS` | 20 | 1 | STM32 creates exactly 1 local participant; original value wasted ~212 KB BSS |
| `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS` | 19 | 30 | 16 nodes + 3 ROS2 daemons + `ros2` CLI ephemeral participants; original caused MemoryPool overflow (runs 016–018) |
| `NUM_WRITER_PROXIES_PER_READER` | 15 | 30 | 1 proxy per remote participant; 30 matches SPDP_MAX |
| `NUM_READER_PROXIES_PER_WRITER` | 15 | 30 | Same |
| `MAX_NUM_UNMATCHED_REMOTE_WRITERS` | 20 | 2 | Unused on static topology; reduced to save BSS |
| `MAX_NUM_UNMATCHED_REMOTE_READERS` | 20 | 2 | Same |
| `THREAD_POOL_READER_STACKSIZE` | 4096 | 8192 | 4096 causes HardFault during deep SEDP burst call chains |
| `THREAD_POOL_WORKLOAD_QUEUE_LENGTH` | 20 | 40 | Absorb discovery burst; 3 Linux nodes join simultaneously |
| `SF_WRITER_HB_PERIOD_MS` | 4000 | 1000 | Faster SEDP convergence; reduces worst-case re-delivery latency |
| `SPDP_RESEND_PERIOD_MS` | 1000 | 500 | Faster initial discovery |
| `HISTORY_SIZE_STATEFUL` | 64 | 5 | Reduced; STM32 app topics have low throughput and low subscriber count |

---

## Build impact (cumulative across all patches)

| Patch | Flash delta | RAM delta |
|-------|------------|-----------|
| 001: Bug C (bitMap 1→8) | +128 | +2,944 BSS |
| 001: Bug A + B (bounds) | +64 | +0 |
| 002: Recv buffer + guards | ~+200 | +1,544 BSS (buffer + 2×guard) |
| 002: Pointer validation | ~+100 | +0 |
| 002: Queue 20→40 | +0 | ~+1,600 BSS (20 extra PacketInfo slots) |
| 003: MemoryPool printf | +32 | +0 |
| 004: PBUF_RAW | +0 | +0 (no code, changes allocation layer) |
| 004: SEDP locator fallbacks | +40 | +12 (static derivedMetaLoc) |
| 004: SEDP immediate resend | +24 | +0 |
| 005: isMulticastAddress fix | ~−8 | +0 (smaller code) |
| 005: readLocatorIntoList precedence | +0 | +0 (comment-only logic change) |
| **config.h MAX_NUM_PARTICIPANTS 20→1** | +0 | **−212,000 BSS** |
| **config.h pool/queue tuning** | +0 | −1,600 BSS (unmatched 20→2 ×2) |
| **Total patch net** | **~+580** | **~+6,100** |
| **Total with config changes** | **~+580** | **~−208,000 (net savings)** |

---

## Verification

After flashing, check `raw_serial_*.log` for:
- No `++ MbedOS Fault Handler ++` lines
- No `[MemoryPool] RESSOURCE LIMIT EXCEEDED` lines
- `stack_free` values should remain stable (not rapidly decreasing)
- `[MROS2LIB] publisher matched` / `subscriber matched` within ~30s of experiment start
- `ros2 topic info /tpc_chassis_imu` should show Publisher count: 1 after discovery
- `ros2 topic hz /tpc_chassis_imu` should show ~10 Hz data rate
- If pointer validation triggers (crash prevented), the board will silently
  drop the corrupted message and continue operating — topics may take longer
  to discover but the board will NOT reboot

### Patch 005 specific
- Run `ss -ulnp` on base PC before experiment — confirm port 8650 is open
  (FastDDS joins multicast), ports 8660/8662 are NOT open (expected, metatraffic
  unicast not bound with `metatrafficMulticastLocatorList` in XML)
- SEDP traffic should now reach FastDDS via multicast: `sudo tcpdump -i enp0s31f6
  'dst host 239.255.0.1 and udp port 8650' -nn` should show packets from both
  192.168.1.2 and 192.168.1.6 with length > 200 bytes (those are SEDP DATA)
- `ros2 topic info /tpc_chassis_imu` Publisher count: 1 (was 0 in runs 019–022)
