# embeddedRTPS Patches — HardFault Root Cause Analysis

## Patch files

- `platform/patches/001-rtps-deserialization-fixes.patch` — AckNack bitmap
  overflow, octetsToNextHeader==0 misparse, TopicData name overflow
- `platform/patches/002-rtps-hardfault-prevention.patch` — receive buffer
  isolation, guard-word corruption detection, NUCLEO-F767ZI pointer validation

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

## Build impact

| Patch | Flash delta | RAM delta |
|-------|------------|-----------|
| 001: Bug C (bitMap 1→8) | +128 | +2,944 BSS |
| 001: Bug A + B (bounds) | +64 | +0 |
| 002: Recv buffer + guards | ~+200 | +1,544 BSS (buffer + 2×guard) |
| 002: Pointer validation | ~+100 | +0 |
| 002: Queue 20→40 | +0 | ~+1,600 BSS (20 extra PacketInfo slots) |
| **Total** | **~+492** | **~+6,088** |

---

## Verification

After flashing, check `raw_serial_*.log` for:
- No `++ MbedOS Fault Handler ++` lines
- `stack_free` values should remain stable (not decreasing)
- Both chassis and sensors topics should be discovered within 5–30s
- If pointer validation triggers (crash prevented), the board will silently
  drop the corrupted message and continue operating — topics may take longer
  to discover but the board will NOT reboot
