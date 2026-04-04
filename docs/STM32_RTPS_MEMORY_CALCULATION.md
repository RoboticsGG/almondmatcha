# STM32 RTPS Memory Allocation — Calculation Guide

**Applies to:** `mros2-mbed-chassis-dynamics` and `mros2-mbed-sensors-gnss`  
**Firmware layer:** embeddedRTPS (`platform/rtps/config.h`)  
**Hardware:** STM32 Nucleo-F767ZI (2 MB flash, 512 KB SRAM)

> **Related docs:**
> - `POC_SINGLE_DOMAIN.md` — single-domain POC experiment guide
>
> **TL;DR of the crash (April 2026):** `MAX_NUM_PARTICIPANTS` was bumped from 15 → 20 to
> accommodate more D5 nodes, pushing `OVERALL_HEAP_SIZE` from 184 KB → 204 KB with the
> original `NUM_STATEFUL_WRITERS=28`. Fix: reduce `NUM_STATEFUL_WRITERS` to the board's
> own publisher count + the **2 SEDP internal writers** that embeddedRTPS always creates.
> **Minimum: `NUM_STATEFUL_WRITERS = 2 + app_publishers`.** Setting it lower than this
> silently prevents the app publisher from being created with no crash or error log.

---

## 1. How embeddedRTPS uses memory on Mbed OS

embeddedRTPS is a C++ RTPS implementation designed for constrained MCUs. It allocates
memory in two categories:

### 1a. OS Threads (stack memory, allocated at boot from heap)

Each of the following spawns an Mbed `Thread` object with a fixed stack:

| Thread type | Count | Stack constant | When created |
|---|---|---|---|
| Thread pool writer | `THREAD_POOL_NUM_WRITERS` | `THREAD_POOL_WRITER_STACKSIZE` | At RTPS init |
| Thread pool reader | `THREAD_POOL_NUM_READERS` | `THREAD_POOL_READER_STACKSIZE` | At RTPS init |
| SPDP writer (per participant slot) | `MAX_NUM_PARTICIPANTS` | `SPDP_WRITER_STACKSIZE` | At RTPS init |
| Stateful writer heartbeat | `NUM_STATEFUL_WRITERS` | `HEARTBEAT_STACKSIZE` | Per local publisher |

The `OVERALL_HEAP_SIZE` constant in `config.h` captures this:

```cpp
constexpr int OVERALL_HEAP_SIZE =
    THREAD_POOL_NUM_WRITERS * THREAD_POOL_WRITER_STACKSIZE +   // writer pool
    THREAD_POOL_NUM_READERS * THREAD_POOL_READER_STACKSIZE +   // reader pool
    MAX_NUM_PARTICIPANTS    * SPDP_WRITER_STACKSIZE +           // discovery threads
    NUM_STATEFUL_WRITERS   * HEARTBEAT_STACKSIZE;               // heartbeat threads
```

> **Critical:** `OVERALL_HEAP_SIZE` is the **minimum** heap consumed at startup before any
> RTPS proxy/pool structures are allocated. If this alone exceeds available heap, the board
> crashes with `unable to allocate thread stack` during `start creating participant`.

### 1b. Static memory pools (allocated from heap at RTPS init)

These are flat arrays of structs allocated once. They do not spawn threads.

| Pool | Sized by | Purpose |
|---|---|---|
| Participant proxy table | `MAX_NUM_PARTICIPANTS` | Track every remote DDS participant |
| Writer-per-participant | `MAX_NUM_PARTICIPANTS × NUM_WRITERS_PER_PARTICIPANT` | Remote publisher endpoints |
| Reader-per-participant | `MAX_NUM_PARTICIPANTS × NUM_READERS_PER_PARTICIPANT` | Remote subscriber endpoints |
| Writer proxy (per local reader) | `NUM_STATEFUL_READERS × NUM_WRITER_PROXIES_PER_READER` | Match table for each local sub |
| Reader proxy (per local writer) | `NUM_STATEFUL_WRITERS × NUM_READER_PROXIES_PER_WRITER` | Match table for each local pub |
| Unmatched remote writers | `MAX_NUM_UNMATCHED_REMOTE_WRITERS` | Buffer discovery bursts |
| Unmatched remote readers | `MAX_NUM_UNMATCHED_REMOTE_READERS` | Buffer discovery bursts |

---

## 2. STM32F767ZI memory budget

```
Total SRAM:              512 KB
  - Mbed OS kernel:      ~80 KB  (RTOS scheduler, TCBs, event flags)
  - mros2 node/spin:     ~30 KB
  - lwIP stack:          ~40 KB  (pbuf pool, sockets, TCP/UDP tables)
  - Application threads: ~20 KB  (sensor tasks, motor task, reporter thread)
  ─────────────────────────────
  Available for RTPS:    ~342 KB  (theoretical upper bound)
  Safe working budget:   ~200 KB  (leave 140 KB headroom)
```

> These are estimates — actual lwIP and OS usage varies. Use the memory reporter
> (`collect_stm32_memory.py`) to verify `heap_free` stays above 100 KB at steady state.

---

## 3. Calculation worksheet

### Step 1 — Count this board's own endpoints

embeddedRTPS **always** creates 2 internal SEDP stateful writers (`sedpPubWriter`,
`sedpSubWriter`) and 2 internal SEDP stateful readers before any application endpoint.
These consume pool slots. The minimum values are:

```
NUM_STATEFUL_WRITERS = 2 (SEDP internal) + number of app publishers
NUM_STATEFUL_READERS = 2 (SEDP internal) + number of app subscribers
```

| Board | App publishers | App subscribers | `NUM_STATEFUL_WRITERS` | `NUM_STATEFUL_READERS` |
|---|---|---|---|---|
| Chassis | 1 (`tpc_chassis_imu`) | 1 (`tpc_chassis_cmd`) | **3** | **3** |
| Sensors | 1 (`tpc_chassis_sensors`) | 0 | **3** | **2** |

> **If you set `NUM_STATEFUL_WRITERS < 2 + app_publishers`**, `createWriter()` returns
> `nullptr` for the app publisher and the board silently stops publishing. There is no
> crash — just silent topic absence.

### Step 2 — Count total domain participants (MAX_NUM_PARTICIPANTS)

List every DDS node visible on the network on the same DOMAIN_ID:

**Single-domain POC (D5, all nodes):**

| Workspace | Nodes | Count |
|---|---|---|
| `ws_rpi` | gnss_spresense, gnss_ublox, gnss_mission_monitor, chassis_controller, chassis_imu, chassis_sensors, mission_monitoring, rover_monitoring | 8 |
| `ws_jetson` | camera_stream, lane_detection, rover_kinematic_control, rover_local_monitoring | 4 |
| `ws_base` | base monitoring nodes | 2 |
| STM32 boards | chassis + sensors | 2 |
| **Total** | | **16** |

```
MAX_NUM_PARTICIPANTS = total_participants + 4   # +4 margin for discovery jitter
```
→ 16 + 4 = 20 ✓

**Baseline multi-domain (D5 only, this board's domain):**

| Workspace | Nodes on D5 | Count |
|---|---|---|
| `ws_rpi` | all 8 rover nodes | 8 |
| `ws_base` | base monitoring | 2 |
| STM32 | both boards | 2 |
| **Total** | | **12** |

→ 12 + 3 = 15 (as used in baseline config)

### Step 3 — Calculate OVERALL_HEAP_SIZE

Plug current constant values into the formula:

```
OVERALL_HEAP_SIZE =
    THREAD_POOL_NUM_WRITERS × THREAD_POOL_WRITER_STACKSIZE
  + THREAD_POOL_NUM_READERS × THREAD_POOL_READER_STACKSIZE
  + MAX_NUM_PARTICIPANTS    × SPDP_WRITER_STACKSIZE
  + NUM_STATEFUL_WRITERS    × HEARTBEAT_STACKSIZE
```

**Chassis board (single-domain POC):**
```
= 1×4096 + 1×4096 + 20×4096 + 3×4096
= 4096 + 4096 + 81920 + 12288
= 102,400 B  (~100 KB)                ← fits well within budget
```

**Sensors board (single-domain POC):**
```
= 1×4096 + 1×4096 + 20×4096 + 3×4096
= 4096 + 4096 + 81920 + 12288
= 100,352 B  (~100 KB)                ← fits well within budget
```

**What crashed (April 2026 — wrong config):**
```
= 1×4096 + 1×4096 + 20×4096 + 28×4096
= 4096 + 4096 + 81920 + 114688
= 204,800 B  (200 KB)  ← OOM before any data structs allocated
```

**Rule of thumb:** Keep `OVERALL_HEAP_SIZE < 200 KB` (hard limit; current single-domain config is at ~100 KB).

### Step 4 — Size the proxy pools

These are per-local-endpoint tables — size them based on what this board actually needs to track:

**NUM_WRITERS_PER_PARTICIPANT** — how many publishers might each remote participant have?  
The heaviest node is `ws_base` with ~5 publishers. Using 20 matches main branch values and ensures no remote publisher is silently dropped.

**NUM_READERS_PER_PARTICIPANT** — how many subscribers might each remote participant have?  
The heaviest node is `ws_base` with ~6 subscribers. Using 20 matches main branch values.

**NUM_WRITER_PROXIES_PER_READER** — for each local stateful reader, how many remote writers it can track simultaneously.  
**Critical:** this is not just app subscribers — the 2 internal SEDP readers use this table to track remote SEDP writers, one per domain participant. With 20 participants you need ≥ 20 proxies. Using 28 matches main branch values. Setting it too small (e.g. 5) causes SEDP to miss participant endpoint advertisements — the local publisher never learns about remote subscribers and sends data to no one (root cause of the sensors board rx failure, April 2026).

**NUM_READER_PROXIES_PER_WRITER** — for each local publisher, how many remote subscribers can match it?  
`tpc_chassis_imu` / `tpc_chassis_sensors` are subscribed by ~8 nodes (mission_monitoring, rover_monitoring, ws_base monitors, etc.). Using 28 matches main branch values.

### Step 5 — Size the unmatched discovery queues

During the 8-second discovery window all 15+ participants announce themselves simultaneously.
`MAX_NUM_UNMATCHED_REMOTE_WRITERS` / `MAX_NUM_UNMATCHED_REMOTE_READERS` are ring-buffer slots
for announcements that arrive before the local match table is ready.

```
MAX_NUM_UNMATCHED_REMOTE_WRITERS ≥ MAX_NUM_PARTICIPANTS    (worst case: all arrive at once)
MAX_NUM_UNMATCHED_REMOTE_READERS ≥ MAX_NUM_PARTICIPANTS + a monitoring burst margin
```

60 and 80 are used for the sensors board (chassis uses 60 and 25). These are well above
the 20-participant limit and match main branch values. If `[MemoryPool] resource limit
exceed` appears in the serial log at boot, increase these by 8 and rebuild.

---

## 4. Current values and reasoning (single-domain POC)

### Chassis board (`mros2-mbed-chassis-dynamics/platform/rtps/config.h`)

| Constant | Value | Reasoning |
|---|---|---|
| `NUM_STATEFUL_WRITERS` | 3 | 2 SEDP internal + 1 app publisher (tpc_chassis_imu) |
| `NUM_STATEFUL_READERS` | 3 | 2 SEDP internal + 1 app subscriber (tpc_chassis_cmd) |
| `MAX_NUM_PARTICIPANTS` | 20 | 16 actual D5 nodes + 4 margin |
| `NUM_WRITERS_PER_PARTICIPANT` | 20 | max publishers per remote node; matches main |
| `NUM_READERS_PER_PARTICIPANT` | 20 | max subscribers per remote node; matches main |
| `NUM_WRITER_PROXIES_PER_READER` | 28 | SEDP reader needs 1 proxy per participant (≥ 20); matches main |
| `NUM_READER_PROXIES_PER_WRITER` | 28 | tpc_chassis_imu subscribers + margin; matches main |
| `MAX_NUM_UNMATCHED_REMOTE_WRITERS` | 60 | 3× participants; matches main |
| `MAX_NUM_UNMATCHED_REMOTE_READERS` | 25 | ≥ 16 participants + monitoring burst |
| `HEARTBEAT_STACKSIZE` | 4096 | Halved from 8192; sufficient for embeddedRTPS heartbeat task |
| `SPDP_WRITER_STACKSIZE` | 4096 | Halved from 8192; critical for keeping OVERALL_HEAP_SIZE in budget |
| `OVERALL_HEAP_SIZE` (computed) | **100 KB** | Well within 200 KB safe budget |

### Sensors board (`mros2-mbed-sensors-gnss/platform/rtps/config.h`)

| Constant | Value | Reasoning |
|---|---|---|
| `NUM_STATEFUL_WRITERS` | 3 | 2 SEDP internal + 1 app publisher (tpc_chassis_sensors) |
| `NUM_STATEFUL_READERS` | 2 | 2 SEDP internal + 0 app subscribers = minimum |
| `MAX_NUM_PARTICIPANTS` | 20 | Same as chassis |
| `NUM_WRITERS_PER_PARTICIPANT` | 20 | max publishers per remote node; matches main |
| `NUM_READERS_PER_PARTICIPANT` | 20 | max subscribers per remote node; matches main |
| `NUM_WRITER_PROXIES_PER_READER` | 28 | SEDP reader tracks 1 proxy per participant (≥ 20); was 5 — root cause of rx failure |
| `NUM_READER_PROXIES_PER_WRITER` | 28 | tpc_chassis_sensors subscribers + margin; matches main |
| `MAX_NUM_UNMATCHED_REMOTE_WRITERS` | 60 | 3× participants; matches main |
| `MAX_NUM_UNMATCHED_REMOTE_READERS` | 80 | ws_base monitoring burst; matches main |
| `OVERALL_HEAP_SIZE` (computed) | **100 KB** | Well within budget |

---

## 5. How to verify on hardware

After flashing, open minicom on each board and look for the memory reporter lines:

```bash
minicom -b 115200 -D /dev/ttyACM0    # sensors board
minicom -b 115200 -D /dev/ttyACM1    # chassis board
```

During the 10-second discovery window you should see `heap_used` rising. After discovery
it flattens. At steady state:

```
{"type":"STM32_MEM","node":"chassis","ts_ms":15000,
 "heap_used":42000,"heap_max":54000,"heap_free":170000,"alloc_fail":0,"stack_free":448}
```

**Pass criteria:**
| Field | Must be |
|---|---|
| `alloc_fail` | `0` — any non-zero value means a pool is exhausted |
| `heap_free` | > 100,000 B (100 KB) at steady state |
| `heap_max` | Stops rising after ~12 s (discovery complete) |
| `stack_free` | > 256 B on the reporter thread |

If `alloc_fail` becomes non-zero or `heap_free` drops below 50 KB, refer to the
troubleshooting table below.

---

## 6. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `unable to allocate thread stack` at boot | `OVERALL_HEAP_SIZE` exceeds available heap | Reduce `NUM_STATEFUL_WRITERS` or `SPDP_WRITER_STACKSIZE` |
| `[MemoryPool] resource limit exceed` at boot | Unmatched discovery queues too small | Increase `MAX_NUM_UNMATCHED_REMOTE_WRITERS/READERS` by 5 |
| `[MemoryPool] resource limit exceed` at steady state | Proxy tables too small | Increase `NUM_READER_PROXIES_PER_WRITER` or `NUM_WRITERS_PER_PARTICIPANT` |
| `alloc_fail` > 0 in memory reporter | Heap fragmentation or pool exhaustion | Check `heap_free`; if > 100 KB, pool sizing is wrong; if < 50 KB, reduce stack sizes |
| `heap_max` keeps rising past 15 s | Memory leak in proxy allocation | Check if `MAX_NUM_PARTICIPANTS` is large enough; a realloc loop can mimic a leak |
| Board reboots in a loop (`Reboot count reached maximum`) | Fatal assertion in embeddedRTPS | Usually a thread stack or pool OOM; check serial output in first 2 seconds before crash |

---

## 7. Recalculating for a different topology

If the domain participant count changes (new nodes added, topology changed), recalculate:

1. List all DDS nodes visible on this board's `DOMAIN_ID`
2. `MAX_NUM_PARTICIPANTS = count + 4`
3. `OVERALL_HEAP_SIZE = (1+1+MAX_NUM_PARTICIPANTS+NUM_STATEFUL_WRITERS) × 4096`
4. Verify `OVERALL_HEAP_SIZE < 200,000` (200 KB hard limit; 100 KB is current target)
5. If it exceeds 200 KB, reduce `NUM_STATEFUL_WRITERS` to `2 (SEDP) + local app publishers`
   (minimum: 3 for chassis with 1 app publisher, 3 for sensors with 1 app publisher)
6. If still over budget, reduce `SPDP_WRITER_STACKSIZE`
   (minimum safe value tested is 3072 B)
7. Rebuild, flash, verify with memory reporter that `alloc_fail = 0`

> **Do not copy ws_base or ws_rpi node endpoint counts into the STM32 config.**
> `NUM_STATEFUL_WRITERS` and `NUM_STATEFUL_READERS` describe **this board's own** publishers
> and subscribers — not the total across the domain.

---

## 8. Change history

### Nov 2025 — Original memory pool fix

**Symptom:** `[Memory pool] resource limit exceed` in the serial log, causing intermittent
topic failures on one or both STM32 boards. Required a full system reset to recover.

**Root cause:** `MAX_NUM_PARTICIPANTS=16` with `SPDP_WRITER_STACKSIZE=8192` allocated
16 × 8 KB = 131 KB just for SPDP discovery threads, leaving insufficient heap under Mbed OS + lwIP.

**Changes made:**

| Constant | Before | After | Reason |
|---|---|---|---|
| `MAX_NUM_PARTICIPANTS` | 16 | 15 | 11 actual nodes + 4 margin; freed 8 KB SPDP stack |
| `SPDP_WRITER_STACKSIZE` | 8192 | 4096 | Halved; sufficient for embeddedRTPS discovery task |
| `HEARTBEAT_STACKSIZE` | 8192 | 4096 | Same |
| `THREAD_POOL_WRITER_STACKSIZE` | 8192 | 4096 | Same |
| `THREAD_POOL_READER_STACKSIZE` | 8192 | 4096 | Same |
| `NUM_WRITERS_PER_PARTICIPANT` | 6 | 20 | Raised to accommodate ws_base (heavy publisher) |
| `NUM_READERS_PER_PARTICIPANT` | 6 | 20 | Same |
| Discovery wait (`osDelay`) | 6 s | 8 s | 8 s = 16 SPDP cycles @ 500 ms; ensures full propagation |

**Result:** SPDP thread heap reduced from ~131 KB → ~61 KB. No further `[Memory pool]` errors.

---

### Apr 2026 — Single-domain OOM crash and fix

**Symptom:** Both STM32 boards crashed during RTPS init with `unable to allocate thread stack`.
No subscriber data received.

**Root cause:** `MAX_NUM_PARTICIPANTS` raised 15 → 20 (Jetson nodes joining D5 in the
single-domain topology) while `NUM_STATEFUL_WRITERS` remained at 28:

```
(1+1+20+28) × 4096 = 204,800 B  ← exceeded available heap
```

Also discovered: `NUM_STATEFUL_WRITERS=2` on the sensors board caused `createWriter()` to
return `nullptr` silently — embeddedRTPS always reserves 2 stateful writer slots for SEDP
(`sedpPubWriter`, `sedpSubWriter`) before any application publisher. With only 2 slots,
`tpc_chassis_sensors` was never actually advertised. No crash, no error log — just silent
topic absence.

**Changes made:**

| Board | `NUM_STATEFUL_WRITERS` | `NUM_STATEFUL_READERS` |
|---|---|---|
| Chassis | 28 → 3 | 32 → 3 |
| Sensors | 28 → 3 | 32 → 2 |

Rule applied: `NUM_STATEFUL_WRITERS = 2 (SEDP) + app publishers`.

**Result:** `OVERALL_HEAP_SIZE` reduced to ~100 KB on both boards. Verified working on hardware.
