# embeddedRTPS Patches — HardFault Root Cause Analysis

## Patch file

`platform/patches/001-rtps-deserialization-fixes.patch`

Applied to `mros2/embeddedRTPS` (cloned from mROS-base/mros2 v0.5.4) by
`build.bash` after clone. Same patch used in both firmware trees.

---

## Bug A — `octetsToNextHeader == 0` mishandling (PRIMARY ROOT CAUSE)

**File:** `src/messages/MessageReceiver.cpp`

### Symptom

Both STM32 boards (NUCLEO-F767ZI) HardFault during DDS discovery burst when
3 Linux nodes join (RPi, Jetson, base PC). Crash in `memcpy` ← `doCopyAndMoveOn()`
in `MessageTypes.cpp:33`. R1 register contains deterministic garbage addresses
(`0xD7087EC3` on chassis, `0x4C8B0E83` on sensors) — same across runs 012–014.

### Root cause

The RTPS spec allows the **last** submessage in a message to set
`octetsToNextHeader = 0`, meaning "extends to end of message". FastDDS uses
this for the last SEDP DATA submessage.

The `processSubmessage()` main loop unconditionally did:
```cpp
msgInfo.nextPos += submsgHeader.octetsToNextHeader + SubmessageHeader::getRawSize();
```

When `octetsToNextHeader == 0`, `nextPos` advanced by only 4 bytes — into the
**body** of the current submessage. The main loop continued, re-interpreting
body data as new submessage headers:

1. Body bytes happen to match a known `SubmessageKind` (e.g., HEARTBEAT=0x07)
2. Deserialization reads at the misaligned offset
3. `currentPos` traverses through SEDP endpoint data (IP addresses, locator
   structs) which are deterministic per-run
4. `currentPos` ends up at an unmapped address → `memcpy` faults

Why R1 was deterministic: the FastDDS SEDP payloads contain the same endpoint
info every run (same topics, IPs, ports), so the cascade always lands on the
same bytes.

### Fix

```cpp
if (submsgHeader.octetsToNextHeader == 0) {
    msgInfo.nextPos = msgInfo.size; // stop parsing
} else {
    DataSize_t advance = submsgHeader.octetsToNextHeader + SubmessageHeader::getRawSize();
    if (msgInfo.nextPos + advance > msgInfo.size) {
        msgInfo.nextPos = msgInfo.size; // prevent overflow
    } else {
        msgInfo.nextPos += advance;
    }
}
```

Also fixed `processDataSubmessage()` size computation which underflowed
(`0 - 24 + 4` → huge uint16_t) when `octetsToNextHeader == 0`. Now correctly
computes `size = getRemainingSize() - SubmessageData::getRawSize()`.

### Why the bitmap fix (Bug C) didn't help

The AckNack bitmap overflow was a genuine bug but **not** in the crash path.
The crash occurred during the misaligned cascade — the code never reached a
real AckNack deserialization with an oversized bitmap. The cascade hit a
Heartbeat-like byte sequence first, causing the `currentPos` pointer to reach
an invalid address during struct field reads.

---

## Bug B — TopicData name buffer overflow

**File:** `src/discovery/TopicData.cpp`

### Problem

`readFromUcdrBuffer()` deserialized `topicNameLength` and `typeNameLength`
from the wire (uint32_t) and passed them directly to `ucdr_deserialize_array_char()`
without clamping:

```cpp
ucdr_deserialize_uint32_t(&buffer, &topicNameLength);
ucdr_deserialize_array_char(&buffer, topicName, topicNameLength); // no bounds check
```

`topicName[40]` and `typeName[60]` are fixed-size arrays (per `Config::MAX_TOPICNAME_LENGTH`
and `Config::MAX_TYPENAME_LENGTH`). If a FastDDS node registers a topic/type name
longer than these limits, `memcpy` writes past the array into adjacent struct
fields or stack memory.

### Fix

Added bounds check before deserialization:
```cpp
if (topicNameLength > Config::MAX_TOPICNAME_LENGTH) {
    buffer.iterator += topicNameLength; // skip oversized name
    break;
}
```

Same for `typeNameLength`.

---

## Bug C — AckNack bitmap overflow (fixed in previous commit)

**File:** `include/rtps/common/types.h`, `src/messages/MessageTypes.cpp`,
`include/rtps/messages/MessageTypes.h`

### Problem

`SequenceNumberSet::bitMap` was `std::array<uint32_t, 1>` (4 bytes). FastDDS
sends up to 256-bit AckNack bitmaps per RTPS spec. `doCopyAndMoveOn()` copied
`4 * ((numBits/32)+1)` bytes into the 4-byte array → stack buffer overflow.

### Fix

- Expanded `bitMap` to `std::array<uint32_t, 8>` (256-bit capacity)
- Fixed bitmap size formula: `(n/32)+1` → `(n+31)/32`
- Added `numBits > 256` bounds check

---

## Incorrect prior diagnosis: stack overflow

The initial analysis (commit `d3c3236`) attributed the crash to ReaderThread
stack overflow (THREAD_POOL_READER_STACKSIZE was 4096). This was wrong.

Evidence from crash dumps:
- `SP = 0x2004E418`, `StackMem = 0x2004C4D0`, `StackSize = 0x2000` (8192)
- Stack used = `StackMem + StackSize - SP` = 0xB8 = **184 bytes** (2.2%)
- The 4096→8192 increase was harmless but unnecessary for this bug

The stack increase to 8192 is retained as safety margin for deep call chains
during SEDP burst processing.

---

## Build impact

| Fix | Flash delta | RAM delta |
|-----|------------|-----------|
| Bug C (bitMap 1→8) | +128 | +2,944 BSS |
| Bug A + B (bounds checks) | +64 | +0 |
| **Total** | **+192** | **+2,944** |

Both boards fit well within NUCLEO-F767ZI limits (2 MB Flash, 512 KB SRAM).

---

## Verification

After flashing, check `raw_serial_*.log` for:
- No `++ MbedOS Fault Handler ++` lines
- `stack_free` values should remain stable (not decreasing)
- Both `/tpc_chassis_imu` and `/tpc_chassis_sensors` topics should be discovered
  within 5–30s of Linux nodes launching
