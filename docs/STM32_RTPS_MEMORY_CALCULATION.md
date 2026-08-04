# STM32 RTPS Memory Budget Calculation

**Board:** NUCLEO-F767ZI (STM32F767ZI) — 512 KB SRAM, 2 MB Flash  
**Applies to:** `mros2-mbed-chassis-dynamics` and `mros2-mbed-sensors-gnss`

---

## Quick Facts

| Item | Value |
|------|-------|
| Total SRAM | 512 KB (DTCM 128 KB + SRAM1 368 KB + SRAM2 16 KB) |
| D5 domain participant count | ~19–23 (12 app nodes + daemons + margin) |
| SPDP_MAX (discovery slots) | 30 (7–11 slots of headroom vs. peak observed) |
| MAX_NUM_PARTICIPANTS | 1 (local pool only) |
| THREAD_POOL_READER_STACKSIZE | 8192 bytes (CRITICAL — 4096 crashes) |
| OVERALL_HEAP_SIZE | ~28.7 KB (thread stacks) |
| RTPS proxy BSS | ~14 KB |

---

## Domain 5 Participant Inventory

> Reconciled 2026-07-27 against the actual node set — the previous version of
> this table used placeholder names (`gnss_node`, `chassis_cmd_node`,
> `mission_executive_node`, `imu_processor_node`, `odometry_node`,
> `safety_monitor_node`, `state_machine_node`) that predated the real
> implementation and undercounted by one (missing `rover_monitoring_node`,
> which has been a real running D5 participant all along — this table had
> not been updated to include it). See [ARCHITECTURE.md](ARCHITECTURE.md),
> [DOMAINS.md](DOMAINS.md), [TOPICS.md](TOPICS.md) for the current node/topic
> reference.

| Node (display name) | Actual DDS participant name | Process | Host |
|------|------|---------|------|
| chassis controller | `rover_node` | STM32 | 192.168.1.2 |
| sensors node | `mros2_node_sensors_d6` | STM32 | 192.168.1.6 |
| gnss_spresense_node | `gnss_spresense_node` | ws_rpi | 192.168.1.1 |
| gnss_ublox_node | `gnss_ublox_node` | ws_rpi | 192.168.1.1 |
| gnss_mission_monitor_node | `gnss_mission_monitor_node` | ws_rpi | 192.168.1.1 |
| chassis_controller_node | `chassis_controller_node` | ws_rpi | 192.168.1.1 |
| chassis_imu_node | `chassis_imu_node` | ws_rpi | 192.168.1.1 |
| chassis_sensors_node | `chassis_sensors_node` | ws_rpi | 192.168.1.1 |
| mission_monitoring_node_rpi | `mission_monitoring_node_rpi` | ws_rpi | 192.168.1.1 |
| rover_monitoring_node | `rover_monitoring_node` | ws_rpi | 192.168.1.1 |
| mission_command_node | `mission_command_node` | ws_base | 192.168.1.10 |
| rover kinematic control (D5 side) | `rover_kinematic_control_d5` | ws_jetson | 192.168.1.5 |

The STM32 boards' actual DDS participant names diverge from their
directory/display names (`rover_node`, `mros2_node_sensors_d6`) — see
`app.cpp` in each firmware workspace. `rover_kinematic_control_node.py` runs
two `rclpy` contexts in one process: the D5-side participant is named
`rover_kinematic_control_d5`; the D6-side one (not counted in this D5
inventory) is `rover_kinematic_control`.

Total: **12 application participants** in D5.

Plus per-SBC overhead at runtime:
- DDS daemon router: ~1 per SBC (3 SBCs → 3)
- rmw_dds_common SystemPublisher: ~1 per process
- Ephemeral `ros2` CLI tool participants (variable)

→ Peak observed: ~19–23 participants (this empirical range was measured
against the real running system, which already included all 12 nodes above —
only the static table was out of date, not the measurement). SPDP_MAX=30
provides 7–11 spare slots.

---

## Memory Budget

### Thread Stack Budget (OVERALL_HEAP_SIZE)

```
OVERALL_HEAP_SIZE = mROS2 threads + Mbed application threads
```

mROS2 thread stacks (from `config.h`):
```
SPDP_WRITER_STACKSIZE          =  4096 bytes
THREAD_POOL_READER_STACKSIZE   =  8192 bytes  (2 threads) = 16 384 bytes
                                 ─────────────
mROS2 total                    = 20 480 bytes ≈ 20 KB
```

Application threads:
```
IMU reader task   = 4096 bytes
App main task     = 4096 bytes (Mbed RTOS default)
                  ────────────
Application total = 8192 bytes ≈ 8 KB
```

Total: **≈ 28.7 KB** against a Mbed RTOS heap budget of ~100 KB.

> **Note:** The mbed_app.json key `MBED_CONF_RTOS_THREAD_STACK_SIZE` controls the
> default task stack. It does not affect THREAD_POOL_READER_STACKSIZE which is set
> directly in config.h.

### BSS Budget (Static Data)

Dominant consumers in embeddedRTPS:

```
MessageReceiver::m_recvBuffer  =  1536 bytes  (Patch 002 isolation buffer)
Proxy arrays:
  NUM_READER_PROXIES_PER_WRITER × NUM_STATEFUL_WRITERS × ~80 B
    = 30 × 3 × 80 =  7200 bytes
  NUM_WRITER_PROXIES_PER_READER × NUM_STATEFUL_READERS × ~80 B
    = 30 × 3 × 80 =  7200 bytes
SPDP participant table:
  SPDP_MAX × ~200 B = 30 × 200 = 6000 bytes
                              ─────────────
Total approx BSS               ≈ 22 KB
```

Mbed BSS total (firmware image): typically 60–80 KB for this project.
F767ZI SRAM available: 512 KB. Memory is NOT a constraint at these values.

### Why `MAX_NUM_PARTICIPANTS=1` Is Mandatory

The old default `MAX_NUM_PARTICIPANTS=15` allocated a *participant pool* of 15 objects,
each containing their own writer-proxy arrays. Size per participant:

```
NUM_WRITERS_PER_PARTICIPANT × NUM_WRITER_PROXIES_PER_READER × ~80 B
  = 16 × 30 × 80 = 38400 bytes ≈ 38 KB per participant
```

With 15 participants: `15 × 38 KB ≈ 570 KB > 512 KB SRAM`. The board hard-faults during
startup before lwIP initializes. Setting `MAX_NUM_PARTICIPANTS=1` saves ~532 KB.

---

## Critical Stack Depth Analysis

### THREAD_POOL_READER_STACKSIZE = 4096 → HardFault

The reader thread executes the call chain:

```
Thread_PoolThread()                     [~100 B]
  └─ workload.callback()               [~80 B]
     └─ DataReader::onDataAvailable()  [~60 B]
        └─ processDataMsg()            [~120 B]
           └─ deserialize()            [~200 B]
              └─ allocateFromPool()    [~60 B]
                 └─ ... (pool error path) [variable]
```

During SEDP burst, the pool error path triggers recursion into the SEDP resend path.
Total observed stack depth just before HardFault: ~4080/4096 bytes (99.6%).

With `THREAD_POOL_READER_STACKSIZE=8192`:
- Measured depth: ~3200 bytes (39%)
- Margin: ~5000 bytes

---

## Memory Scaling Worksheet

Use this when adding nodes to domain 5:

```
SPDP_MAX = ceil(expected_participants × 1.5)  [round up to multiple of 4]
NUM_WRITER_PROXIES_PER_READER = SPDP_MAX
NUM_READER_PROXIES_PER_WRITER = SPDP_MAX

BSS_check = (NUM_WRITER_PROXIES_PER_READER × NUM_STATEFUL_READERS
           + NUM_READER_PROXIES_PER_WRITER × NUM_STATEFUL_WRITERS)
           × 80  [bytes]

Should be < 40 KB to stay well within 512 KB SRAM.
```

Current (12 nodes, SPDP_MAX=30):
```
BSS_check = (30×3 + 30×3) × 80 = 14 400 bytes  ✓
```

If D5 grows to 20 nodes (SPDP_MAX=36):
```
BSS_check = (36×3 + 36×3) × 80 = 17 280 bytes  ✓
```

---

## lwIP Configuration

From `mbed_app.json` (`mros2-mbed-chassis-dynamics/mbed_app.json`):

```json
"lwip.pbuf-pool-size": 32
```

The default lwIP pbuf pool is 16, which exhausts during a DDS discovery burst
(30+ packets arrive simultaneously from 3+ SBCs). Size 32 provides adequate headroom
for peak burst. Increasing beyond 64 consumes lwIP heap from the same 512 KB pool.

`PBUF_RAW` (Patch 004) ensures the pbuf is allocated with zero transport-header
offset — required for RTPS CDR payloads starting at byte 0.

---

**See Also:** [RTPS_CONFIG_REFERENCE.md](RTPS_CONFIG_REFERENCE.md) · [EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md) · [STM32_CHANGES_SUMMARY.md](STM32_CHANGES_SUMMARY.md)
