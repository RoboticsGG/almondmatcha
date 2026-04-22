# RTPS Config Reference — STM32 embeddedRTPS Parameters

**Applies to:** `mros2-mbed-chassis-dynamics` and `mros2-mbed-sensors-gnss`  
**File:** `platform/rtps/config.h`  
**Firmware base:** mROS-base/mros2 v0.5.4

---

## Current Production Values (main branch)

| Parameter | Chassis (192.168.1.2) | Sensors (192.168.1.6) | Notes |
|-----------|----------------------|-----------------------|-------|
| `MAX_NUM_PARTICIPANTS` | `1` | `1` | Local participant only — STM32 never discovers STM32 |
| `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS` | `30` | `30` | 11 D5 nodes + daemons + CLI tools + margin |
| `NUM_STATEFUL_WRITERS` | `3` | `3` | 2 SEDP + 1 app publisher (`tpc_chassis_imu` / `tpc_sensors_data`) |
| `NUM_STATEFUL_READERS` | `3` | `2` | Chassis: 2 SEDP + 1 sub (`tpc_chassis_cmd`); Sensors: 2 SEDP only |
| `NUM_WRITERS_PER_PARTICIPANT` | `16` | `16` | Max publishers per remote participant (10 actual + margin) |
| `NUM_READERS_PER_PARTICIPANT` | `16` | `16` | Max subscribers per remote participant (11 actual + margin) |
| `NUM_WRITER_PROXIES_PER_READER` | `30` | `30` | 1 per remote participant, must match SPDP_MAX |
| `NUM_READER_PROXIES_PER_WRITER` | `30` | `30` | 1 per remote participant, must match SPDP_MAX |
| `MAX_NUM_UNMATCHED_REMOTE_WRITERS` | `2` | `2` | Unused — topology is static |
| `MAX_NUM_UNMATCHED_REMOTE_READERS` | `2` | `2` | Unused — topology is static |
| `MAX_NUM_READER_CALLBACKS` | `3` | `2` | Match `NUM_STATEFUL_READERS` |
| `THREAD_POOL_READER_STACKSIZE` | `8192` | `8192` | **CRITICAL — 4096 causes HardFault during SEDP burst** |
| `THREAD_POOL_WORKLOAD_QUEUE_LENGTH` | `40` | `40` | 20 → drop during burst; doubled to absorb SEDP storm |
| `SF_WRITER_HB_PERIOD_MS` | `1000` | `1000` | Was 2000 — faster heartbeat for reliability |
| `SPDP_RESEND_PERIOD_MS` | `500` | `500` | 500ms SPDP announcements |
| `HISTORY_SIZE_STATEFUL` | `5` | `5` | Reduced from 64 — DDS RELIABLE delivery doesn't need deep history here |

---

## Parameter Explanations

### Discovery Pool Parameters

#### `MAX_NUM_PARTICIPANTS` (pool size)
The number of participant slots in the embeddedRTPS internal memory pool.
This is **NOT** the maximum number of remote participants that can be discovered —
that is `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS`. This controls how many *local*
participant objects can be instantiated. Because each STM32 creates exactly one
mROS2 node, the minimum is 1.

**Memory impact:** Each participant uses ~200 KB of BSS for its writer/reader proxy arrays.
Setting this to 15 (old default) wastes ~2.8 MB — more than the F767ZI has.
`MAX_NUM_PARTICIPANTS=1` is mandatory.

#### `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS`
The size of the discovered-participant table. Every DDS participant that sends a
SPDP announcement (including ROS2 CLI tools, rmw_dds_common, daemons, etc.) occupies
one slot. If the table is full, new participants are silently dropped and their topic
data never arrives.

**In production (D5 domain):**
- 11 application nodes (ws_rpi×7, ws_base×1, ws_jetson×1, chassis STM32, sensors STM32)
- 4–6 daemon/daemon-router participants per SBC
- Ephemeral CLI participants (`ros2 topic list`, `ros2 node info`, etc.)

30 slots provides 14+ of headroom. Set to the nearest power of 2 for alignment.

#### `NUM_WRITER_PROXIES_PER_READER` / `NUM_READER_PROXIES_PER_WRITER`
How many remote endpoint proxies each local endpoint can hold. Each proxy represents
one remote DDS endpoint that has matched. Since `SPDP_MAX=30`, set both to 30 so
each endpoint can track all discovered remote participants.

**Former crash source:** Chassis was at 6, sensors at 14. With 19+ participants
discovering simultaneously, the proxy pool overflowed — a `MemoryPool` panic at
`RESSOURCE LIMIT EXCEEDED` during the SEDP burst.

### Thread and Stack Parameters

#### `THREAD_POOL_READER_STACKSIZE`
Stack size for the RTPS reader callback thread pool. This thread executes callbacks
passed from the lwIP network receive path into the RTPS reader context.

**4096 bytes causes HardFault.** During SEDP burst, nested calls from
`RTRProxy → onDataAvailable → deserialize → processDataMsg → allocateFromPool`
exhaust the stack. 8192 bytes provides ample headroom; memory budget still fits
within F767ZI constraints (see [STM32_RTPS_MEMORY_CALCULATION.md](STM32_RTPS_MEMORY_CALCULATION.md)).

#### `THREAD_POOL_WORKLOAD_QUEUE_LENGTH`
Length of the work queue fed by the lwIP receive ISR. When 3–5 remote nodes all
respond to SPDP simultaneously, 30–50 messages arrive within milliseconds.
A queue of 20 was too short — messages were dropped and SEDP retries were needed.
Queue of 40 absorbs the burst without drops.

#### `SF_WRITER_HB_PERIOD_MS`
How often stateful writers send Heartbeat messages. This drives SEDP reliability —
if the initial SEDP send is lost (Patch 004 fix), the writer retransmits after
this interval. 1000 ms is a balance between bandwidth and reliability.

### Endpoint Count Parameters

#### `NUM_STATEFUL_WRITERS` / `NUM_STATEFUL_READERS`
Must match the actual number of stateful endpoints created at runtime:

| Board | Writers | Readers |
|-------|---------|---------|
| Chassis | 3 (SEDP pub + SEDP sub + `tpc_chassis_imu`) | 3 (SEDP pub + SEDP sub + `tpc_chassis_cmd`) |
| Sensors | 3 (SEDP pub + SEDP sub + `tpc_sensors_data`) | 2 (SEDP pub + SEDP sub) |

SEDP endpoints are always 2 per participant (`sedpPubWriter` + `sedpSubWriter` and their
reader counterparts). Application endpoints add on top.

Over-allocating wastes MemoryPool slots. Under-allocating causes a panic on
participant creation.

#### `MAX_NUM_READER_CALLBACKS`
Pool of callback function pointers. Must be ≥ `NUM_STATEFUL_READERS`.
Set equal to `NUM_STATEFUL_READERS`.

---

## How Parameters Drive Memory Usage

The dominant BSS consumers are the proxy arrays. Formula:

```
PROXIES_BSS = (NUM_WRITER_PROXIES_PER_READER × NUM_STATEFUL_READERS
             + NUM_READER_PROXIES_PER_WRITER × NUM_STATEFUL_WRITERS)
             × sizeof(proxy_entry)

≈ (30×3 + 30×3) × 80 B ≈ 14,400 B ≈ 14 KB
```

With `MAX_NUM_PARTICIPANTS=1`, OVERALL_HEAP_SIZE (thread stacks) is ~28.7 KB.
Total RTPS overhead ≈ 43 KB. See [STM32_RTPS_MEMORY_CALCULATION.md](STM32_RTPS_MEMORY_CALCULATION.md)
for the complete budget.

---

## Pre-Tune Values vs. Production Values

The following changed from the mROS2 v0.5.4 defaults/initial configuration:

| Parameter | Initial (broken) | Production |
|-----------|-----------------|------------|
| `MAX_NUM_PARTICIPANTS` | `15` | `1` |
| `NUM_WRITER_PROXIES_PER_READER` | `6` | `30` |
| `NUM_READER_PROXIES_PER_WRITER` | `6` | `30` |
| `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS` | `14` | `30` |
| `THREAD_POOL_READER_STACKSIZE` | `4096` | `8192` |
| `THREAD_POOL_WORKLOAD_QUEUE_LENGTH` | `20` | `40` |
| `SF_WRITER_HB_PERIOD_MS` | `2000` | `1000` |
| `HISTORY_SIZE_STATEFUL` | `64` | `5` |

---

## Tuning Guidelines for Future Topology Changes

If nodes are added or removed from domain 5:

1. **Adding a node:** If D5 participant count exceeds ~22 (approaching SPDP_MAX=30),
   increase `SPDP_MAX`, `NUM_WRITER_PROXIES_PER_READER`, `NUM_READER_PROXIES_PER_WRITER`
   by the same delta, rebuild and reflash both boards.

2. **Adding a subscriber on Chassis:** Increment `NUM_STATEFUL_READERS` and
   `MAX_NUM_READER_CALLBACKS` by 1 each.

3. **Adding a publisher on Sensors:** Increment `NUM_STATEFUL_WRITERS` by 1.

4. **Never decrease** `THREAD_POOL_READER_STACKSIZE` below 8192.
   Never increase `THREAD_POOL_WORKLOAD_QUEUE_LENGTH` past 64 (F767ZI heap limit).

---

**See Also:** [EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md) · [STM32_RTPS_MEMORY_CALCULATION.md](STM32_RTPS_MEMORY_CALCULATION.md) · [STM32_CHANGES_SUMMARY.md](STM32_CHANGES_SUMMARY.md)
