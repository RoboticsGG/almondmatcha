# embeddedRTPS `config.h` Parameter Reference

**File:** `platform/rtps/config.h` in `mros2-mbed-chassis-dynamics` and `mros2-mbed-sensors-gnss`  
**Hardware:** STM32 Nucleo-F767ZI (512 KB SRAM, 2 MB flash)  
**Stack:** mros2 → embeddedRTPS → lwIP → Mbed OS

> For the heap-size calculation and crash history, see `docs/STM32_RTPS_MEMORY_CALCULATION.md`.

---

## Overview

embeddedRTPS is a header-only C++ RTPS implementation for constrained MCUs. Because dynamic
allocation on a bare-metal RTOS is risky, almost everything is backed by **compile-time fixed
arrays**. `config.h` sets the sizes of those arrays and the stack sizes of the OS threads they
create.

There are four categories of parameters:

| Category | Parameters | Effect |
|---|---|---|
| Domain identity | `DOMAIN_ID`, `VENDOR_ID`, `BASE_GUID_PREFIX` | Which DDS domain this board joins |
| Endpoint pool sizes | `NUM_STATEFUL_*`, `NUM_STATELESS_*`, `MAX_NUM_*` | Static array capacities |
| OS thread stacks | `*_STACKSIZE`, `THREAD_POOL_NUM_*` | Heap consumed at boot |
| RTPS protocol timing | `SF_WRITER_HB_PERIOD_MS`, `SPDP_*`, `*_PRIO` | Discovery and reliability behaviour |

---

## 1. Domain Identity

### `DOMAIN_ID`
```cpp
const uint8_t DOMAIN_ID = 5;
```
The DDS domain number this board participates in. Boards on different domain IDs cannot
see each other's topics. Valid range: 0–232 over UDP. Both STM32 boards are always on D5
(rover control domain) regardless of branch.

### `VENDOR_ID`
```cpp
const VendorId_t VENDOR_ID = {13, 37};
```
Two-byte vendor identifier embedded in every RTPS packet header. Identifies embeddedRTPS
to other DDS implementations. Do not change.

### `BASE_GUID_PREFIX`
```cpp
const GuidPrefix_t BASE_GUID_PREFIX{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13};
```
12-byte prefix used to construct the GUID of this participant. The last byte differs
between chassis (`13`) and sensors (`12`) boards to prevent GUID collisions on the same
network. Do not change; if two participants have the same prefix they silently ignore
each other's messages.

---

## 2. Endpoint Pool Sizes

These are compile-time array capacities. Setting them too small causes **silent runtime
failures** (publisher/subscriber never created) or `[MemoryPool] resource limit exceed`
log lines. Setting them too large wastes SRAM or — for stateful writers — consumes heap
via OS threads at boot.

### `NUM_STATEFUL_WRITERS`
```cpp
const uint8_t NUM_STATEFUL_WRITERS = 3;   // single-domain chassis/sensors
// const uint8_t NUM_STATEFUL_WRITERS = 28;  // multi-domain/main
```
**Size of the local stateful-writer pool.** A stateful writer is the RTPS entity behind
each reliable ROS2 publisher. Two are always created internally by embeddedRTPS for SEDP
(Simple Endpoint Discovery Protocol) before any application publisher:

- `sedpPubWriter` — announces this participant's publishers to the network
- `sedpSubWriter` — announces this participant's subscribers to the network

**Minimum safe value: `2 (SEDP) + number of app publishers`.**  
If the value is exactly 2 and the board has 1 app publisher, `createWriter()` returns
`nullptr` and the publisher is silently never created — no crash, no log, just absent topic.

**Memory impact:** Each stateful writer spawns one **heartbeat OS thread** consuming
`HEARTBEAT_STACKSIZE` bytes from the heap at RTPS init. This is the primary lever for
controlling `OVERALL_HEAP_SIZE`.

| Branch | Chassis | Sensors | Reason |
|---|---|---|---|
| `single-domain` | 3 | 3 | 2 SEDP + 1 app publisher; required to stay in heap budget with `MAX_NUM_PARTICIPANTS=20` |
| `multi-domain` / `main` | 28 | 28 | Over-provisioned from original main config; same SEDP floor applies (minimum 3) but 184 KB still fits heap at `MAX_NUM_PARTICIPANTS=15` |

### `NUM_STATEFUL_READERS`
```cpp
const uint8_t NUM_STATEFUL_READERS = 3;   // chassis
const uint8_t NUM_STATEFUL_READERS = 2;   // sensors
```
**Size of the local stateful-reader pool.** A stateful reader is the RTPS entity behind
each reliable ROS2 subscriber. Two are always created internally for SEDP:

- `sedpPubReader` — listens for publisher announcements from other participants
- `sedpSubReader` — listens for subscriber announcements from other participants

**Minimum safe value: `2 (SEDP) + number of app subscribers`.**  
Unlike stateful writers, stateful readers do **not** spawn OS threads per slot — they use
the shared thread pool. So increasing this does not consume extra heap thread stacks.

| Board | App subscribers | Minimum | Set to |
|---|---|---|---|
| Chassis | 1 (`tpc_chassis_cmd`) | 3 | 3 (single-domain) / 32 (multi-domain) |
| Sensors | 0 | 2 | 2 (both branches) |

### `NUM_STATELESS_WRITERS` / `NUM_STATELESS_READERS`
```cpp
const uint8_t NUM_STATELESS_WRITERS = 4;
const uint8_t NUM_STATELESS_READERS = 4;
```
Pool sizes for best-effort (unreliable) RTPS writers and readers. SPDP uses one stateless
writer (`SPDPAgent`) to broadcast participant announcements. The value 4 provides margin
for any additional best-effort endpoints. Do not reduce below 1 (SPDP needs one slot).

### `MAX_NUM_PARTICIPANTS`
```cpp
const uint8_t MAX_NUM_PARTICIPANTS = 20;   // single-domain
// const uint8_t MAX_NUM_PARTICIPANTS = 15;  // multi-domain
```
**Maximum number of remote DDS participants this board tracks simultaneously.**  
Each slot corresponds to one remote ROS2 node process (one `rclpy`/`rclcpp` node = one
DDS participant). This controls:

1. The **SPDP writer thread count** — embeddedRTPS creates one `SPDP_WRITER_STACKSIZE`
   thread per slot at init. This is the largest contributor to `OVERALL_HEAP_SIZE`.
2. The **participant proxy table** — a static array of structs, one per participant slot.

**Setting this too low** causes participants beyond the limit to be silently ignored —
their topics never appear, no error. Set it to `(actual node count) + margin`.

| Branch | Actual D5 nodes | Max participants | Margin |
|---|---|---|---|
| `single-domain` | 16 (rpi×8, jetson×4, base×2, stm32×2) | 20 | +4 |
| `multi-domain` | 12 (rpi×8, base×2, stm32×2) | 15 | +3 |

**Crash threshold:** Raising `MAX_NUM_PARTICIPANTS` while keeping `NUM_STATEFUL_WRITERS=28`
(multi-domain scale) overflows the heap budget. The crash that triggered this doc:
`MAX_NUM_PARTICIPANTS` 15→20 with `NUM_STATEFUL_WRITERS=28` → `OVERALL_HEAP_SIZE` 184→204 KB
→ `unable to allocate thread stack`.

### `NUM_WRITERS_PER_PARTICIPANT`
```cpp
const uint8_t NUM_WRITERS_PER_PARTICIPANT = 8;   // both branches
```
**Maximum number of publishers (writers) each remote participant may have.**  
Used to size the per-participant writer endpoint table:
`total_slots = MAX_NUM_PARTICIPANTS × NUM_WRITERS_PER_PARTICIPANT`.

When a remote participant announces more publishers than this limit, the excess are silently
dropped — those topics are invisible to this board. Set to the max publisher count of the
heaviest remote node.

| Node | Publishers | Notes |
|---|---|---|
| `ws_base` monitoring | ~5 | Heaviest in this system |
| `ws_rpi` nodes | 1–4 | Most have 1–2 |
| STM32 boards | 1 each | |

Both branches use 8 (safe margin over worst case ~5). Note: this array is stored as
`std::array<Writer*, 8>` (pointer array, 4 bytes each) in every Participant slot, so the
BSS cost of increasing it is small. But with `MAX_NUM_PARTICIPANTS=20` keep it low.

### `NUM_READERS_PER_PARTICIPANT`
```cpp
const uint8_t NUM_READERS_PER_PARTICIPANT = 8;   // both branches
```
Same structure as `NUM_WRITERS_PER_PARTICIPANT` but for subscribers. Set to the max
subscriber count of the heaviest remote node (`ws_base` with ~6). Both branches use 8.

### `NUM_WRITER_PROXIES_PER_READER`
```cpp
const uint8_t NUM_WRITER_PROXIES_PER_READER = 22;   // both boards
```
**For each local stateful reader, how many remote writers it can track simultaneously.**  
This is the match table for a local subscriber — it records which remote publishers have
matched with this subscription.

**Critical for discovery:** the 2 internal SEDP stateful readers use this table to track
remote SEDP writers — one slot per remote domain participant. With
`SPDP_MAX_NUMBER_FOUND_PARTICIPANTS=19`, at least 19 writer proxies per SEDP reader are
needed. Set to 22 (= 19 + 3).

**This pool does NOT live per-Participant.** It lives in each `StatefulReader` which is
stored in `Domain.m_statefulReaders[]` (a flat array sized by `NUM_STATEFUL_READERS=3`).
Each +1 costs only `NUM_STATEFUL_READERS × sizeof(WriterProxy) ≈ 44` bytes total — safe
to size generously.

Setting this too small caused a silent publish failure (April 2026): sensors board had
`NUM_WRITER_PROXIES_PER_READER=5` — SEDP readers could track only 5 remote SEDP writers.
With 16 participants, 11 were missed including the RPi `tpc_chassis_sensors` subscriber.
Publisher’s reader-proxy list stayed empty and sent data to no one.

### `NUM_READER_PROXIES_PER_WRITER`
```cpp
const uint8_t NUM_READER_PROXIES_PER_WRITER = 15;   // both boards
```
**For each local stateful writer, how many remote readers it can track simultaneously.**  
This is the match table for a local publisher — it records which remote subscribers have
matched with this publication and tracks their acknowledgment state for reliable delivery.

`tpc_chassis_imu` and `tpc_chassis_sensors` are subscribed by: mission_monitoring,
rover_monitoring, base PC monitoring nodes, and rover_kinematic_control — roughly 6–8
nodes depending on configuration. 15 covers all expected single-domain D5 subscribers.

### `MAX_NUM_UNMATCHED_REMOTE_WRITERS`
```cpp
const uint8_t MAX_NUM_UNMATCHED_REMOTE_WRITERS = 20;   // both boards
```
**Ring-buffer depth for remote writer announcements that arrive before the local match
table is ready.**  When all domain participants start and send SPDP announcements,
some endpoint announcements may arrive before this board has set up the corresponding
local reader. `MAX_NUM_UNMATCHED_REMOTE_WRITERS` is the queue depth for those pending
announcements.

If this is too small during a burst, the overflow is silently dropped — the subscribing
endpoint never matches and the topic stays invisible. Symptom: `[MemoryPool] resource limit
exceed` in the serial log during boot.

Safe lower bound: `≥ MAX_NUM_PARTICIPANTS` (worst case: all arrive at once) → use 20.

> **BSS cost warning:** This pool is stored in `SEDPAgent`, which is embedded inline in
> **every** `Participant` slot (all 20) in `Domain`'s static BSS. `sizeof(TopicDataCompressed)
> ≈ 60 B`, so each +1 here costs `20 × 60 = 1.2 KB` of static SRAM. The main branch uses
> 60 (at `MAX_NUM_PARTICIPANTS=15`, total = 54 KB). Copying that value to single-domain
> (`MAX_NUM_PARTICIPANTS=20`) would add `20 × 40 × 60 = 48 KB` of extra BSS and causes
> SRAM overflow.

### `MAX_NUM_UNMATCHED_REMOTE_READERS`
```cpp
const uint8_t MAX_NUM_UNMATCHED_REMOTE_READERS = 25;   // both boards
```
Equivalent queue for remote **reader** announcements. 25 ≥ 16 actual D5 participants
with margin for monitoring subscription bursts.

> Same BSS cost warning as `MAX_NUM_UNMATCHED_REMOTE_WRITERS` above. Main branch uses
> 80 at `MAX_NUM_PARTICIPANTS=15` (total 72 KB). At 20 participants that would add 66 KB
> of extra BSS. Keep at 25.

### `MAX_NUM_READER_CALLBACKS`
```cpp
const uint8_t MAX_NUM_READER_CALLBACKS = 3;   // chassis
const uint8_t MAX_NUM_READER_CALLBACKS = 2;   // sensors
```
Maximum number of reader callback registrations. Set to the number of app subscribers
plus the SEDP internal readers. Chassis has 1 app subscriber + 2 SEDP = 3; sensors has
0 app subscribers + 2 SEDP = 2.

### `HISTORY_SIZE_STATELESS` / `HISTORY_SIZE_STATEFUL`
```cpp
const uint8_t HISTORY_SIZE_STATELESS = 2;
const uint8_t HISTORY_SIZE_STATEFUL = 5;
```
Per-endpoint message history cache depth (number of messages held in the reliability
buffer). Stateless (best-effort) needs minimal history. Stateful (reliable) keeps 5 to
handle re-transmission requests. Reducing these saves SRAM; increasing helps with
high-rate reliable topics but consumes more.

### `MAX_TYPENAME_LENGTH` / `MAX_TOPICNAME_LENGTH`
```cpp
const uint8_t MAX_TYPENAME_LENGTH = 60;
const uint8_t MAX_TOPICNAME_LENGTH = 40;
```
Fixed char-array sizes for storing the ROS2 type name and topic name strings in each
endpoint descriptor. If a topic name or type name exceeds these lengths, `createWriter()`
/ `createReader()` returns `nullptr`. All project topic names and type names are well
within these limits.

### `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS`
```cpp
const uint8_t SPDP_MAX_NUMBER_FOUND_PARTICIPANTS = 19;   // single-domain (MAX-1)
// const uint8_t SPDP_MAX_NUMBER_FOUND_PARTICIPANTS = 14; // multi-domain (MAX-1)
```
Maximum number of **remote** participants tracked by the SPDP agent. Always set to
`MAX_NUM_PARTICIPANTS - 1` (excludes self). Controls the size of the SPDP remote
participant table.

### `SPDP_MAX_NUM_LOCATORS`
```cpp
const uint8_t SPDP_MAX_NUM_LOCATORS = 5;
```
Maximum number of network locators (IP:port pairs) stored per remote participant. Each
DDS participant may advertise multiple locators (unicast, multicast, meta-traffic). 5 is
sufficient for all observed ROS2 implementations. Do not reduce below 2.

### `MAX_NUM_UDP_CONNECTIONS`
```cpp
const int MAX_NUM_UDP_CONNECTIONS = 10;
```
lwIP UDP PCB (protocol control block) pool size — maximum simultaneously-open UDP sockets
used by embeddedRTPS. One connection per active topic endpoint. 10 covers 1 publisher +
1 subscriber + SPDP + SEDP with margin. Do not reduce below 6.

---

## 3. OS Thread Stack Sizes

These are consumed from the Mbed heap **at RTPS init**, before any application code runs.
Their sum (weighted by the thread count they control) is captured in `OVERALL_HEAP_SIZE`.

```
OVERALL_HEAP_SIZE =
    THREAD_POOL_NUM_WRITERS × THREAD_POOL_WRITER_STACKSIZE
  + THREAD_POOL_NUM_READERS × THREAD_POOL_READER_STACKSIZE
  + MAX_NUM_PARTICIPANTS    × SPDP_WRITER_STACKSIZE        ← largest term
  + NUM_STATEFUL_WRITERS    × HEARTBEAT_STACKSIZE
```

### `HEARTBEAT_STACKSIZE`
```cpp
const int HEARTBEAT_STACKSIZE = 4096;
```
Stack size for each stateful-writer heartbeat thread. One thread is created per
`NUM_STATEFUL_WRITERS` slot at init. Original embeddedRTPS value was 8192 B; halved to
4096 B after testing confirmed it is sufficient for this project's heartbeat callback
(simple `sendHeartbeat()` with no deep call stack).

### `THREAD_POOL_WRITER_STACKSIZE` / `THREAD_POOL_READER_STACKSIZE`
```cpp
const int THREAD_POOL_WRITER_STACKSIZE = 4096;
const int THREAD_POOL_READER_STACKSIZE = 4096;
```
Stack sizes for the single shared writer-thread-pool thread and reader-thread-pool thread.
These threads dispatch outgoing/incoming RTPS messages. Halved from 8192 B; sufficient
for the message dispatch path in this project.

### `SPDP_WRITER_STACKSIZE`
```cpp
const uint16_t SPDP_WRITER_STACKSIZE = 4096;
```
**The most impactful stack size constant.** One SPDP writer OS thread is created **per
`MAX_NUM_PARTICIPANTS` slot** at init. With `MAX_NUM_PARTICIPANTS=20`, this alone accounts
for 20 × 4096 = 80 KB of heap before any application code. Halved from 8192 B.

> Minimum tested: 3072 B (with careful call-stack profiling). Do not reduce further
> without re-profiling — SPDP participant announcement runs the longest call chain.

### `THREAD_POOL_NUM_WRITERS` / `THREAD_POOL_NUM_READERS`
```cpp
const int THREAD_POOL_NUM_WRITERS = 1;
const int THREAD_POOL_NUM_READERS = 1;
```
Number of threads in the writer/reader thread pool. One thread each is sufficient because
this board has only 1 publisher and at most 1 subscriber — no parallelism needed. Each
consumes its respective `_STACKSIZE` bytes at boot.

### `THREAD_POOL_WRITER_PRIO` / `THREAD_POOL_READER_PRIO`
```cpp
const int THREAD_POOL_WRITER_PRIO = 24;
const int THREAD_POOL_READER_PRIO = 24;
```
Mbed OS thread priority for the pool threads. 24 = `osPriorityNormal` in Mbed CMSIS-RTOS.
Setting too high starves application threads; too low delays outgoing messages.

### `THREAD_POOL_WORKLOAD_QUEUE_LENGTH`
```cpp
const int THREAD_POOL_WORKLOAD_QUEUE_LENGTH = 20;
```
Depth of the internal work-item queue fed to the thread pool. Each queued item is one
pending send or receive operation. 20 is sufficient for burst handling during discovery.

### `SPDP_WRITER_PRIO`
```cpp
const uint8_t SPDP_WRITER_PRIO = 24;
```
Mbed OS thread priority for the per-participant SPDP writer threads (one per
`MAX_NUM_PARTICIPANTS` slot). Same level as the pool threads.

---

## 4. RTPS Protocol Timing

### `SF_WRITER_HB_PERIOD_MS`
```cpp
const uint16_t SF_WRITER_HB_PERIOD_MS = 2000;
```
Heartbeat send period for stateful writers (reliable publishers), in milliseconds. The
heartbeat is a small control message sent periodically to inform subscribers that the
writer is alive and to trigger re-transmission requests for any lost messages. 2000 ms
(2 s) is appropriate for the low-rate sensor topics in this project (10 Hz IMU,
1 Hz GNSS). Reduce for high-rate or latency-sensitive reliable topics.

### `SPDP_RESEND_PERIOD_MS`
```cpp
const uint16_t SPDP_RESEND_PERIOD_MS = 500;
```
How often SPDP participant announcements are re-broadcast, in milliseconds. Lower values
accelerate discovery (new participants are found faster) at the cost of more network
traffic. 500 ms = 2 announcements/second during liveliness maintenance. The DDS spec
default is 1000 ms; 500 ms was chosen for faster convergence when nodes join the network.

### `SPDP_CYCLECOUNT_HEARTBEAT`
```cpp
const uint8_t SPDP_CYCLECOUNT_HEARTBEAT = 2;
```
Number of SPDP resend cycles to skip between liveliness checks. With
`SPDP_RESEND_PERIOD_MS=500`, liveliness is checked every `2 × 500 = 1000 ms`. If a
remote participant misses enough checks, it is considered lost and removed from the table.

### `SPDP_DEFAULT_REMOTE_LEASE_DURATION`
```cpp
const Duration_t SPDP_DEFAULT_REMOTE_LEASE_DURATION = {100, 0};
```
Default lease duration (seconds, nanoseconds) assumed for a remote participant if none is
advertised in their SPDP announcement. 100 seconds = consider remote participant lost if
no announcement received for 100 s. Usually overwritten by the remote's actual value.

### `SPDP_MAX_REMOTE_LEASE_DURATION`
```cpp
const Duration_t SPDP_MAX_REMOTE_LEASE_DURATION = {180, 0};
```
Hard cap on the lease duration accepted from any remote participant. Even if a remote
participant claims it will announce every 300 s, this board treats 180 s as the maximum
wait before declaring it absent. Prevents stale entries from blocking participant slots
for too long.

### `SPDP_LEASE_DURATION`
```cpp
const Duration_t SPDP_LEASE_DURATION = {100, 0};
```
The lease duration this board advertises to remote participants in its own SPDP
announcements. Remote boards wait up to 100 s after missing an announcement before
removing this board from their participant tables. 100 s matches the default remote
lease above for symmetric behaviour.

---

## 5. The `OVERALL_HEAP_SIZE` Formula

```cpp
constexpr int OVERALL_HEAP_SIZE =
    THREAD_POOL_NUM_WRITERS * THREAD_POOL_WRITER_STACKSIZE +
    THREAD_POOL_NUM_READERS * THREAD_POOL_READER_STACKSIZE +
    MAX_NUM_PARTICIPANTS    * SPDP_WRITER_STACKSIZE +
    NUM_STATEFUL_WRITERS   * HEARTBEAT_STACKSIZE;
```

This is the **total heap consumed by OS thread stacks at RTPS init**, before any proxy
structs or application data. It is a compile-time constant used only as a documentation
reference — Mbed allocates threads individually and the board crashes with
`unable to allocate thread stack` if the heap is insufficient.

### Current values

| Configuration | `MAX_NUM_PARTICIPANTS` | `NUM_STATEFUL_WRITERS` | `OVERALL_HEAP_SIZE` | Status |
|---|---|---|---|---|
| `single-domain` (chassis + sensors) | 20 | 3 | **100 KB** | ✓ working |
| `multi-domain` / `main` (both boards) | 15 | 28 | **184 KB** | ✓ working |
| Crash case (April 2026) | 20 | 28 | **204 KB** | ✗ OOM at boot |

### How to recalculate when topology changes

1. Count all DDS nodes visible on this board's `DOMAIN_ID`
2. Set `MAX_NUM_PARTICIPANTS = count + margin (3–5)`
3. Compute `OVERALL_HEAP_SIZE = (1+1+MAX_NUM_PARTICIPANTS+NUM_STATEFUL_WRITERS) × 4096`
4. Target < 200 KB
5. If over budget: reduce `NUM_STATEFUL_WRITERS` to `2 (SEDP) + local app publishers`
6. If still over: reduce `SPDP_WRITER_STACKSIZE` (minimum tested: 3072 B)
7. Flash and verify `alloc_fail = 0` in memory reporter JSON

---

## 6. Quick-reference: what to change for common tasks

| Goal | Parameter to change | Notes |
|---|---|---|
| Add a new reliable publisher to this board | Increase `NUM_STATEFUL_WRITERS` by 1 | Adds 1 heartbeat thread × 4 KB heap |
| Add a new reliable subscriber to this board | Increase `NUM_STATEFUL_READERS` by 1 | No extra thread |
| Add more ROS2 nodes to the domain | Increase `MAX_NUM_PARTICIPANTS` | Adds N × 4 KB SPDP threads |
| Board silently not publishing a topic | Verify `NUM_STATEFUL_WRITERS ≥ 2 + app_publishers` | Silent `createWriter()` nullptr |
| Board silently not subscribing to a topic | Verify `NUM_STATEFUL_READERS ≥ 2 + app_subscribers` | Silent `createReader()` nullptr |
| `[MemoryPool] resource limit exceed` at boot | Increase `MAX_NUM_UNMATCHED_REMOTE_WRITERS/READERS` by 10 | Discovery burst overflow |
| `[MemoryPool] resource limit exceed` at runtime | Increase `NUM_READER_PROXIES_PER_WRITER` or `NUM_WRITERS_PER_PARTICIPANT` | Match table overflow |
| `unable to allocate thread stack` | Reduce `NUM_STATEFUL_WRITERS` or `SPDP_WRITER_STACKSIZE` | See formula above |
| Faster initial discovery | Reduce `SPDP_RESEND_PERIOD_MS` | More network traffic |
| Reduce heartbeat network traffic | Increase `SF_WRITER_HB_PERIOD_MS` | Slower re-transmission detection |
