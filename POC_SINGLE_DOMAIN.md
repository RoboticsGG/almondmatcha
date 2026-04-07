# POC — Single ROS2 Domain ID=5: Measurement Guide

**Branch:** `single-domain`  
**Goal:** Measure how collapsing D4+D5+D6 → all D5 affects message latency, publish jitter, socket buffer pressure, interface bandwidth on the Linux SBCs (RPi + Jetson), and heap/stack footprint on both STM32 Nucleo boards.

---

## Topology Change Summary

| | Baseline (main) | POC (this branch) |
|---|---|---|
| Domain 4 | monitoring (Base + Jetson) | ✗ removed |
| Domain 5 | 11 participants (rover control) | **15 participants** (all nodes) |
| Domain 6 | vision (Jetson localhost) | ✗ removed |
| camera/lane Images@30fps on network | no | **yes** (stress factor) |
| STM32 MAX_NUM_PARTICIPANTS | 15 | **20** (5 margin) |
| STM32 discovery wait | 8 s | **10 s** |

---

## What is Measured

Five independent collectors run in parallel. Each targets a different layer of the system.

### 1. ROS2 message jitter and end-to-end latency — `collect_latency.py` (on SBC)

Source: `rclpy` subscriber callback wall-clock time (`time.time()` — NTP epoch float).

| Metric | Column | Topics | Meaning |
|---|---|---|---|
| **Inter-arrival interval** | `interval_ms` | all | Time between consecutive messages on the same topic. Std-dev of this is the primary jitter metric — high std-dev means DDS scheduling is unstable. |
| **End-to-end latency** | `latency_ms` | `/tpc_telemetry_relay` only | `recv_time − header.stamp`. Measures how long a publisher-stamped message takes to reach the subscriber. Only available for message types that carry `header.stamp`. |
| Receive wall-clock | `recv_time_s` | all | Absolute NTP-aligned timestamp; used as the sync anchor in `--merge` mode. |

> **Why only `/tpc_telemetry_relay` for latency?** It is the only project message type whose
> definition includes a `header.stamp` field. All other message types (`ChassisIMU`,
> `ChassisSensors`, etc.) carry raw sensor values only — the publisher timestamp is not
> embedded in the payload, so latency cannot be computed from the message content alone.

### 2. Network interface bandwidth and socket buffer pressure — `collect_net_stats.py` (on SBC)

Source: `/proc/net/dev` (interface counters) + `/proc/net/udp` (per-socket kernel buffers), sampled every 0.5 s.

| Metric | Column | Meaning |
|---|---|---|
| **RX / TX bandwidth** | `rx_bps`, `tx_bps` | Bytes per second on `eth0`. Indicates how much extra traffic the domain consolidation adds (especially camera frames at 30 FPS joining D5). |
| **Interface packet drops** | `rx_drop`, `tx_drop` | Cumulative kernel-level drops at the NIC ring buffer. Non-zero = the SBC is falling behind at the driver layer. Should stay 0. |
| Interface errors | `rx_errs`, `tx_errs` | Hardware-level CRC / frame errors. Should stay 0. |
| **UDP socket count** | `udp_sockets` | Number of open UDP sockets. DDS opens one socket per topic per participant; an increase vs baseline reveals the cost of extra participants in one domain. |
| **Max per-socket RX buffer fill** | `max_rx_queue` | Bytes sitting unread in the largest UDP receive buffer on the SBC (`/proc/net/udp`). Non-zero means the application layer is not draining the kernel buffer fast enough — backpressure. |
| Total RX / TX queue | `total_rx_queue`, `total_tx_queue` | Sum across all UDP sockets. Complements `max_rx_queue`. |
| SS UDP total | `ss_udp_total` | Cross-check from `ss -s`; total UDP socket count from the kernel's socket layer. |

> **Why `max_rx_queue` matters:** When DDS adds more topics to a single domain the kernel
> must demultiplex more UDP multicast streams on the same interface. If `max_rx_queue` grows
> non-zero in the POC but stays zero in the baseline, the extra participants are creating
> buffer pressure — a precursor to dropped DDS messages.

### 3. STM32 RTPS heap and stack footprint — `collect_stm32_memory.py` (base PC, USB serial)

Source: `memory_reporter.h` firmware thread, emits one `{"type":"STM32_MEM",...}` JSON line per second over UART.

| Metric | Column | Meaning |
|---|---|---|
| **Heap used** | `heap_used` (bytes) | Current mbed heap allocation. Rises during RTPS discovery as participant proxy structs are allocated. |
| **Heap high-water mark** | `heap_max` (bytes) | Peak allocated since boot. Set during the 10-second discovery window and never released — the key comparison figure between baseline and POC. |
| Heap free | `heap_free` (bytes) | `total_heap − heap_used`. Negative trend = leak risk. |
| **Alloc failures** | `alloc_fail` | Count of failed `malloc` calls. Any non-zero value means the RTPS pool is exhausted — critical failure indicator. |
| Stack free (minimum) | `stack_free` (bytes) | Minimum free stack across all RTOS threads. Non-zero headroom required; too-small value risks stack overflow under domain-consolidation load. |
| Board uptime | `ts_ms` | STM32 uptime in ms (no RTC). Used for relative timing within a single board run; `wall_clock` (base PC NTP time) is used for cross-source alignment. |

> **Why the discovery window is critical:** `MAX_NUM_PARTICIPANTS` was raised from 15 → 20.
> Each additional remote participant causes the RTPS stack to allocate writer/reader proxy
> structs in a statically-bounded pool. If the POC's `heap_max` exceeds the baseline's by
> more than the pool-size increase predicts, it indicates unexpected allocation elsewhere
> (e.g. larger RTPS history caches under higher topic count).

### 4. Per-topic DDS bandwidth — `collect_topic_bw.py` (base PC)

Source: rclpy subscriber that auto-discovers every D5 topic, serializes each received
message with `serialize_message()` (CDR encoding — same as the DDS wire format), and
accumulates byte counts over 1-second intervals.

| Metric | Column | Meaning |
|---|---|---|
| **Bytes/s per topic** | `bps` | CDR wire-format bytes per second observed at the base PC subscriber. Identifies which topics dominate bandwidth so the net-stats totals can be decomposed. |
| Messages/s per topic | `msg_per_s` | Observed publish rate for each topic; cross-check against expected Hz. |
| Interval accumulation | `msg_count`, `bytes` | Raw counts for the 1-second bucket — useful for detecting gaps (zero count = publisher stopped). |

> **Why CDR size and not IP packet size?** `serialize_message()` returns the payload as
> FastDDS would place it on the wire before RTPS framing. It is consistent across DDS
> implementations and does not require packet capture. The `net_stats` rx/tx figures include
> all UDP overhead (RTPS headers, DDS built-in topics, multicast join traffic); subtracting
> the sum of per-topic `bps` from `net_stats.rx_bps` reveals the non-payload DDS overhead.


|---|---|---|---|
| ROS2 message timing | `collect_latency.py` | RPi, Jetson | Does single-domain increase jitter or latency? |
| Linux NIC / socket buffers | `collect_net_stats.py` | RPi, Jetson | Do extra participants fill UDP buffers or drop packets? |
| STM32 RTPS heap | `collect_stm32_memory.py` | Base PC (USB serial) | Does `MAX_NUM_PARTICIPANTS=20` leave enough heap headroom? |
| Per-topic bandwidth | `collect_topic_bw.py` | Base PC (ROS2 subscriber) | How many KB/s does each DDS topic contribute on the wire? |

---

## Building Each Workspace

### Interface package architecture

All three Linux workspaces share the same message definitions via symlinks into `common_ifaces/`:

```
common_ifaces/
  msgs_ifaces/      ← source of truth for all custom message types
  action_ifaces/    ← DesData action (used by ws_base and ws_rpi)
  services_ifaces/  ← SpdLimit service (used by ws_base and ws_rpi)

ws_base/src/
  msgs_ifaces  →  ../../common_ifaces/msgs_ifaces   (symlink)
  action_ifaces/          (real copy — built in ws_base)
  services_ifaces/        (real copy — built in ws_base)

ws_rpi/src/
  msgs_ifaces      →  ../../common_ifaces/msgs_ifaces   (symlink)
  action_ifaces    →  ../../common_ifaces/action_ifaces  (symlink)
  services_ifaces  →  ../../common_ifaces/services_ifaces (symlink)

ws_jetson/src/
  (no interface packages — uses only std_msgs and sensor_msgs)
```

**Critical rule:** `msgs_ifaces` must be built in **exactly one install path per machine**.
Building it in two workspaces simultaneously creates duplicate `.so` files in
`LD_LIBRARY_PATH`, which causes `Could not import rosidl_typesupport_c for package
msgs_ifaces` at runtime. The build scripts below enforce this.

**When you change any `.msg` file** in `common_ifaces/msgs_ifaces/msg/`, rebuild
`common_ifaces` cleanly first on every machine before rebuilding any workspace:
```bash
cd ~/almondmatcha/common_ifaces && rm -rf build/ install/ log/
colcon build --symlink-install --packages-select msgs_ifaces
```
Also run `sync_stm32_interfaces.sh` to push the change to the STM32 firmware.

---

### ws_base (base PC — x86_64)

`msgs_ifaces` is sourced from `common_ifaces/install/` and **not rebuilt** inside ws_base
(a `COLCON_IGNORE` file in `ws_base/src/msgs_ifaces/` prevents colcon from picking it up).

```bash
# 1 — build common_ifaces (msgs_ifaces only; run once or after any .msg change)
cd ~/almondmatcha/common_ifaces
colcon build --symlink-install --packages-select msgs_ifaces

# 2 — clean build ws_base (sources common_ifaces automatically)
cd ~/almondmatcha/ws_base
bash build_clean.sh
source install/setup.bash
```

Packages built inside ws_base: `action_ifaces`, `services_ifaces`, `mission_control`.

---

### ws_rpi (Raspberry Pi — arm64)

SSH into the RPi first. All three interface packages are symlinks to `common_ifaces/` and
are built together inside `ws_rpi/install/` — there is no separate `common_ifaces` install
on the RPi, so no duplicate risk.

```bash
ssh curry@192.168.1.1
cd ~/almondmatcha/ws_rpi

# Clean build (required after any .msg change or fresh clone)
bash build_clean.sh
source install/setup.bash

# Incremental build (after changing only application code)
bash build_inc.sh
source install/setup.bash
```

Packages built: `msgs_ifaces`, `action_ifaces`, `services_ifaces`, `chassis_control`,
`chassis_sensors`, `gnss_navigation`, `rover_monitoring`, `rover_bringup`.

---

### ws_jetson (Jetson — arm64)

SSH into the Jetson first. No custom interface packages — only Python nodes using
`std_msgs` and `sensor_msgs`.

```bash
ssh yupi@192.168.1.5
cd ~/almondmatcha/ws_jetson

# Clean build
bash build_clean.sh
source install/setup.bash

# Incremental build
bash build_inc.sh
source install/setup.bash
```

Packages built: `vision_navigation`, `rover_monitoring`.

---

### Build order summary

| Order | Machine | Command |
|---|---|---|
| 1st | Base PC | `cd common_ifaces && colcon build --symlink-install --packages-select msgs_ifaces` |
| 2nd | Base PC | `cd ws_base && bash build_clean.sh` |
| 3rd | RPi | `cd ws_rpi && bash build_clean.sh` |
| 4th | Jetson | `cd ws_jetson && bash build_clean.sh` |

Steps 2–4 are independent of each other and can proceed in parallel once step 1 is done
on the base PC (the RPi and Jetson build their own `msgs_ifaces` from source and do not
depend on the base PC's `common_ifaces/install/`).

---

## Prerequisites

### 1. Flash STM32 firmware (both boards)

Both `platform/rtps/config.h` files have been updated with `MAX_NUM_PARTICIPANTS=20`.  
Both `app.cpp` files now call `memory_reporter_start()` after all threads launch.  
**You must rebuild and re-flash before running the POC.**

```bash
# On the base PC — build chassis firmware
cd mros2-mbed-chassis-dynamics
bash build.bash
# Flash via ST-Link

# Build sensors firmware
cd mros2-mbed-sensors-gnss
bash build.bash
# Flash via ST-Link
```

### 2. Pre-flight checks on each SBC (no installation needed)

> **Why no installation?** The measurement tool (`collect_latency.py`) is a plain `rclpy`
> subscriber node — no LTTng required.
>
> **Background:** The official apt `ros-humble-rclcpp` for arm64/Humble is compiled with
> `TRACETOOLS_DISABLED=1`, so `ros2:*` LTTng tracepoints are not available in the binary.
> Jitter and latency are measured at the application layer instead.

SSH into each SBC and run the check script:

```bash
# RPi
ssh curry@192.168.1.1 'bash ~/almondmatcha/ws_base/tools/tracing/setup_tracing.sh'

# Jetson
ssh yupi@192.168.1.5  'bash ~/almondmatcha/ws_base/tools/tracing/setup_tracing.sh'
```

All checks must say `[OK]`. If `ws_rpi` or `ws_jetson` is not built yet, build it now (see the **Building Each Workspace** section above for full details):

```bash
# RPi (SSH in)
cd ~/almondmatcha/ws_rpi
bash build_clean.sh
source install/setup.bash

# Jetson (SSH in)
cd ~/almondmatcha/ws_jetson
bash build_clean.sh
source install/setup.bash
```

> **Note:** On the RPi, `msgs_ifaces` is built directly inside `ws_rpi/install/` (no
> separate `common_ifaces` step needed on the RPi). On the base PC, see the build order
> in the **Building Each Workspace** section — `common_ifaces` must be built first.

### 3. Python dependencies on base PC

```bash
pip3 install pandas numpy matplotlib
```

### 4. Smoke-test: verify collect_latency.py works end-to-end

This test verifies the collector records data correctly **without needing STM32 boards or a full system launch**. It uses `ros2 topic pub` to inject synthetic traffic.

> **Note:** `rover_monitoring_node` is a *subscriber* — it waits for STM32 data and won't produce messages on its own. Use `ros2 topic pub` below instead.

Run all commands on the **RPi** (SSH in from base PC or directly):

**Terminal 1** — inject synthetic IMU messages at 10 Hz:
```bash
source /opt/ros/humble/setup.bash
source ~/almondmatcha/ws_rpi/install/setup.bash
export ROS_DOMAIN_ID=5

ros2 topic pub /tpc_chassis_imu msgs_ifaces/msg/ChassisIMU \
  "{accel_x: 100, accel_y: 0, accel_z: 980, gyro_x: 0, gyro_y: 0, gyro_z: 0}" \
  --rate 10
```

**Terminal 2** — run the collector:
```bash
source /opt/ros/humble/setup.bash
source ~/almondmatcha/ws_rpi/install/setup.bash
export ROS_DOMAIN_ID=5

python3 ~/almondmatcha/ws_base/tools/tracing/collect_latency.py \
    --topics /tpc_chassis_imu \
    --out ~/ros2_traces/smoke_test.csv
```

Expected output in Terminal 2:
```
Writing to /home/curry/ros2_traces/smoke_test.csv
Waiting for 1 topic(s) to appear on the DDS bus...
  Subscribed: /tpc_chassis_imu  [msgs_ifaces/msg/ChassisIMU]
All topics subscribed. Collecting...
```

Wait 30 seconds, then **Ctrl-C** Terminal 2 first, then Terminal 1.

**Verify the CSV:**
```bash
wc -l ~/ros2_traces/smoke_test.csv
# Expected: ~31 lines (1 header + ~30 messages at 10 Hz over 30 s)

head -4 ~/ros2_traces/smoke_test.csv
# Expected:
# topic,recv_time_s,header_stamp_s,latency_ms,interval_ms
# /tpc_chassis_imu,1775224309.123,...,,
# /tpc_chassis_imu,1775224309.223,...,,100.1
# /tpc_chassis_imu,1775224309.323,...,,99.8
```

> **Smoke test timestamps:** `recv_time_s` is a real NTP-aligned UNIX epoch float (identical
> format to the main experiment). `header_stamp_s` and `latency_ms` are empty because
> `ChassisIMU` has no `header.stamp` field — latency is only measurable for
> `/tpc_telemetry_relay`. The smoke test only runs the latency collector (no STM32 or
> net_stats collectors), so `--merge` does not apply here; use `--csv` for single-file
> analysis.

**Quick analysis on base PC:**
```bash
scp curry@192.168.1.1:~/ros2_traces/smoke_test.csv /tmp/

python3 - << 'EOF'
import csv, statistics, collections
intervals = collections.defaultdict(list)
with open("/tmp/smoke_test.csv") as f:
    for row in csv.DictReader(f):
        if row["interval_ms"]:
            intervals[row["topic"]].append(float(row["interval_ms"]))
for topic, vals in sorted(intervals.items()):
    print(f"{topic}")
    print(f"  n={len(vals)}  mean={statistics.mean(vals):.1f}ms  "
          f"std={statistics.stdev(vals):.2f}ms  max={max(vals):.1f}ms")
EOF
# Expected: /tpc_chassis_imu  n=~29  mean=~100ms  std=<5ms
```

If `interval_ms` column is empty for all rows: confirm `ROS_DOMAIN_ID=5` is set in both terminals.

---

## Running the POC Experiment

### Option A — Automated (recommended): `launch_poc_experiment.sh`

A single script handles the entire sequence from the base PC — starting collectors,
prompting for the power-cycle, launching all ROS2 nodes, waiting, and pulling CSVs.

**Before running:** verify which `/dev/ttyACM*` port is which:
```bash
minicom -b 115200 -D /dev/ttyACM0   # check "app name:" line on boot
minicom -b 115200 -D /dev/ttyACM1
# ttyACM0 = sensors/GNSS board  (IP 192.168.1.6, "STM32 Sensors Node")
# ttyACM1 = chassis board        (IP 192.168.1.2, "RoverWithIMU")
# USB enumeration order depends on plug-in sequence — verify before each run.
```

**Run the experiment:**
```bash
cd ~/almondmatcha

# Default: 5-minute run, /dev/ttyACM1=chassis, /dev/ttyACM0=sensors
bash ws_base/launch_poc_experiment.sh

# Override run duration (seconds)
bash ws_base/launch_poc_experiment.sh --duration 600

# If ttyACM order is reversed
bash ws_base/launch_poc_experiment.sh --chassis /dev/ttyACM0 --sensors /dev/ttyACM1

# Collectors only — skip ROS2 node launch (nodes already running)
bash ws_base/launch_poc_experiment.sh --skip-launch
```

**What the script does:**

| Step | Action |
|---|---|
| Pre-flight | Checks `tmux`, `ssh`, `pyserial`, both serial ports, SSH reachability |
| Start STM32 collector | Starts `collect_stm32_memory.py` in background — **before boards power on** |
| ⏸ Interactive pause | Waits for you to physically power-cycle both STM32 boards |
| Launch ROS2 nodes | SSHes into RPi → Jetson → launches base PC nodes during STM32 discovery wait |
| Start latency collectors | Runs `start_trace.sh` on RPi and Jetson via SSH |
| Start topic-BW collector | Runs `collect_topic_bw.py` locally on base PC (auto-discovers all D5 topics) |
| Start net-stats collectors | Runs `collect_net_stats.py` on both SBCs via SSH background `nohup` |
| Timed run | Live dashboard shows elapsed time, STM32 heap values, and collector health; Ctrl-C stops early and still collects CSVs |
| Stop and pull | Stops all collectors, pulls CSVs to base PC, runs `merge_run_csv.py`, prints analysis commands |

**Output files — all in one numbered run directory on the base PC:**
```
ws_base/tools/poc_run/run_NNN/
  latency_rpi.csv          # collect_latency.py — RPi inter-arrival + latency
  latency_jetson.csv       # collect_latency.py — Jetson inter-arrival + latency
  net_stats_rpi.csv        # collect_net_stats.py — RPi NIC counters
  net_stats_jetson.csv     # collect_net_stats.py — Jetson NIC counters
  topic_bw.csv             # collect_topic_bw.py — per-topic CDR bandwidth
  stm32_chassis.csv        # collect_stm32_memory.py — chassis board heap
  stm32_sensors.csv        # collect_stm32_memory.py — sensors board heap
  merged_all.csv           # built automatically: 1s-bucketed union of all above
  logs/                    # stdout/stderr from every sub-process
    launch_rpi.log
    launch_jetson.log
    launch_base.log
    stm32_collector.log
    topic_bw.log
```

`run_NNN` is auto-incremented — each experiment launch creates the next available
number. The `run_*/` directories are git-ignored; the tools are tracked.

> **STM32 CSV naming:** the collector initially writes timestamped filenames
> (`stm32_YYYYMMDD_HHMMSS.csv`). The launcher renames them to `stm32_chassis.csv` /
> `stm32_sensors.csv` after stopping the collector so the merge step has predictable paths.

---

### Option B — Manual (step-by-step)

Use this if you need to run only a subset of collectors, or if the automated script fails
partway through and you want to resume from a specific step.

#### Launch sequence overview

The hardest constraint is the **10-second STM32 discovery window**: the heap peak (the
primary measurement) happens in that window and is missed if collectors or Linux nodes
are late. Follow this order exactly.

| # | Elapsed | Machine | Action |
|---|---|---|---|
| 1 | t − 30 s | Base PC | Start STM32 collector (Step 1 below) |
| 2 | t = 0 | Base PC | Power-cycle **both** STM32 boards (unplug/replug USB-C power) |
| 3 | t + 2 s | RPi | `bash launch_rover_single_domain.sh` |
| 4 | t + 3 s | Jetson | `bash launch_jetson_single_domain.sh` |
| 5 | t + 4 s | Base PC | `bash launch_base_single_domain.sh` |
| 6 | t + 5 s | Base PC | `start_trace.sh` on RPi + Jetson (latency collectors) |
| 7 | t + 6 s | Base PC | `collect_net_stats.py` on RPi + Jetson (SSH terminals) |
| 8 | t + 10 s | — | STM32 discovery wait ends; boards begin publishing |
| 9 | t + 5 min | — | Stop all collectors; pull CSVs |

> **Why Linux nodes at t+2–5 s?** DDS participant matching is bidirectional — the STM32 only
> allocates proxy memory for participants it has already seen. Bringing Linux nodes up late
> means fewer RPi/Jetson participants are matched during the discovery window, producing an
> artificially low `heap_max`.

#### Step 1 — Start STM32 memory collection on base PC

```bash
python3 ws_base/tools/stm32_serial/collect_stm32_memory.py \
  --chassis /dev/ttyACM1 --sensors /dev/ttyACM0 \
  --out ~/ros2_traces/stm32_memory_poc
```

Leave running. **Now power-cycle both STM32 boards.**

#### Step 2 — Start all nodes

```bash
# RPi
ssh curry@192.168.1.1 "bash ~/almondmatcha/ws_rpi/launch_rover_single_domain.sh"

# Jetson
ssh yupi@192.168.1.5 "bash ~/almondmatcha/ws_jetson/launch_jetson_single_domain.sh"

# Base PC
bash ws_base/launch_base_single_domain.sh
```

The tmux session borders are **yellow/red** (vs blue/cyan in baseline).

#### Step 3 — Start latency/jitter collection on RPi and Jetson

```bash
TARGET_HOST=curry@192.168.1.1 TARGET_LABEL=rpi    bash ws_base/tools/tracing/start_trace.sh
TARGET_HOST=yupi@192.168.1.5  TARGET_LABEL=jetson bash ws_base/tools/tracing/start_trace.sh
```

#### Step 4 — Start network stats collection on each SBC

```bash
# Terminal A — RPi
ssh curry@192.168.1.1 \
  "python3 ~/almondmatcha/ws_base/tools/monitoring/collect_net_stats.py \
     --iface eth0 --out ~/ros2_traces/net_stats_rpi.csv"

# Terminal B — Jetson
ssh yupi@192.168.1.5 \
  "python3 ~/almondmatcha/ws_base/tools/monitoring/collect_net_stats.py \
     --iface eth0 --out ~/ros2_traces/net_stats_jetson.csv"
```

#### Step 5 — Let it run

Run for **at least 5 minutes** under representative load (send a mission command, drive the rover).

#### Step 6 — Stop collection

```bash
# Stop latency collectors and pull CSVs to base PC
TARGET_HOST=curry@192.168.1.1 TARGET_LABEL=rpi \
    LOCAL_DEST_CSV=/tmp/latency_rpi.csv    bash ws_base/tools/tracing/stop_and_collect_trace.sh
TARGET_HOST=yupi@192.168.1.5  TARGET_LABEL=jetson \
    LOCAL_DEST_CSV=/tmp/latency_jetson.csv bash ws_base/tools/tracing/stop_and_collect_trace.sh
# → /tmp/latency_rpi.csv   (or omit LOCAL_DEST_CSV to use legacy path)
# → /tmp/latency_jetson.csv

# Stop net-stats (Ctrl-C in each SSH terminal), then pull:
scp curry@192.168.1.1:~/ros2_traces/net_stats_rpi.csv    /tmp/net_stats_rpi.csv
scp yupi@192.168.1.5:~/ros2_traces/net_stats_jetson.csv  /tmp/net_stats_jetson.csv

# Stop STM32 collector (Ctrl-C in the Step 1 terminal)
# Stop topic-BW collector (Ctrl-C if running)
```

> **Tip:** the automated script (Option A) handles all of the above automatically,
> including routing all files into `ws_base/tools/poc_run/run_NNN/` and calling
> `merge_run_csv.py` at the end. The manual steps above write to `/tmp/` for brevity;
> adjust destination paths as needed.

---

## Analyzing Results

### Latency and jitter (from CSV)

The primary metric is **inter-arrival jitter** (std-dev of the time between consecutive
messages on each topic). This works for all topics and is the most relevant indicator
of DDS scheduling instability under domain consolidation.

**End-to-end latency** (publisher timestamp → subscriber receive time) is also available
for `/tpc_telemetry_relay` — the only project message type that includes `header.stamp`.

```bash
# Analyze POC run only (prints table to stdout + saves latency_summary.csv)
python3 ws_base/tools/tracing/analyze_latency.py \
  --csv ws_base/tools/poc_run/run_001/latency_rpi.csv

# Side-by-side comparison (requires baseline CSV from main branch)
python3 ws_base/tools/tracing/analyze_latency.py \
  --baseline ws_base/tools/tracing/data/baseline_latency_rpi.csv \
  --poc      ws_base/tools/poc_run/run_001/latency_rpi.csv \
  --out-dir  ws_base/tools/tracing/results/

# Filter to specific topics only
python3 ws_base/tools/tracing/analyze_latency.py \
  --poc ws_base/tools/poc_run/run_001/latency_rpi.csv \
  --topics /tpc_chassis_imu /tpc_chassis_sensors /tpc_rover_ctrl_cmd
```

Output:
- Per-topic table: `mean`, `p50`, `p95`, `p99`, `max`, `std` of inter-arrival interval (ms)
- Per-topic latency stats for `/tpc_telemetry_relay` if present
- `latency_summary.csv` — machine-readable comparison table
- `jitter_boxplot.png` — box-plot when both `--baseline` and `--poc` are given

### Unified timeline — all sources on one NTP-aligned axis

After option A finishes, a `unified_timeline.png` can be generated from the run directory
with a single command. All Linux clocks are NTP-synced (~1–10 ms accuracy). The STM32
has no RTC; its `wall_clock` column (base PC receive time) is used as the sync anchor.

**Recommended — using `--run-dir` (auto-discovers all CSVs):**
```bash
python3 ws_base/tools/tracing/analyze_latency.py --merge \
    --run-dir ws_base/tools/poc_run/run_001
# Produces: ws_base/tools/poc_run/run_001/unified_timeline.png
```

**Manual — specifying each file individually:**
```bash
python3 ws_base/tools/tracing/analyze_latency.py --merge \
    --latency-rpi    ws_base/tools/poc_run/run_001/latency_rpi.csv \
    --latency-jetson ws_base/tools/poc_run/run_001/latency_jetson.csv \
    --stm32          ws_base/tools/poc_run/run_001/stm32_chassis.csv \
    --stm32-sensors  ws_base/tools/poc_run/run_001/stm32_sensors.csv \
    --net-rpi        ws_base/tools/poc_run/run_001/net_stats_rpi.csv \
    --net-jetson     ws_base/tools/poc_run/run_001/net_stats_jetson.csv \
    --topic-bw       ws_base/tools/poc_run/run_001/topic_bw.csv \
    --out-dir        ws_base/tools/poc_run/run_001/
```

Outputs `unified_timeline.png` in the output directory — up to 5 stacked panels sharing
a common elapsed-seconds x-axis (x=0 = earliest event across all datasets):

| Panel | Source | What to look for |
|---|---|---|
| RPi jitter | `latency_rpi.csv` | Spikes in `interval_ms` during discovery |
| Jetson jitter | `latency_jetson.csv` | Same, Jetson-side |
| STM32 heap | `stm32_chassis/sensors.csv` | Rising ramp during 10 s boot window → flat steady-state |
| Network BW | `net_stats_*.csv` | Bandwidth spike when camera_stream@30fps joins D5 |
| Per-topic BW | `topic_bw.csv` | Which topics drive bandwidth; gaps = publisher outage |

All `--*` arguments (and `--run-dir`) are optional — omit any source not collected.

### Merged time-bucketed CSV — `merged_all.csv`

The launcher builds this automatically at the end of every run. It can also be regenerated
at any time:

```bash
python3 ws_base/tools/poc_run/merge_run_csv.py \
    --run-dir ws_base/tools/poc_run/run_001

# Finer time resolution (0.5-second buckets):
python3 ws_base/tools/poc_run/merge_run_csv.py \
    --run-dir ws_base/tools/poc_run/run_001 --bucket-s 0.5
```

`merged_all.csv` has one row per time bucket with all metrics as columns:

| Column group | Example column | Content |
|---|---|---|
| Time | `elapsed_s` | Seconds since earliest event (NTP-aligned) |
| Latency per topic per SBC | `interval_rpi__tpc_chassis_imu__mean_ms` | Mean inter-arrival interval in the bucket |
| Latency p95 | `interval_rpi__tpc_chassis_imu__p95_ms` | 95th-percentile interval |
| End-to-end latency | `latency_rpi__tpc_telemetry_relay__mean_ms` | Mean header.stamp → recv latency |
| Network BW | `net_rpi__rx_kbps` | Mean RX KB/s in the bucket |
| Topic BW | `bw__tpc_chassis_imu__kbps` | Mean CDR KB/s per topic |
| STM32 heap | `stm32_chassis__heap_used_kb` | Mean heap used in the bucket |

This is the most convenient file for loading into pandas or a spreadsheet for
custom analysis, since all sources are already aligned to the same time axis.

```bash
python3 - << 'EOF'
import pandas as pd
df = pd.read_csv("ws_base/tools/poc_run/run_001/merged_all.csv")
print(df.head())
print(df.describe())
EOF
```

### Socket buffer and bandwidth (from net_stats CSV)

Open in any spreadsheet or:

```bash
python3 - << 'EOF'
import pandas as pd
df = pd.read_csv("ws_base/tools/poc_run/run_001/net_stats_rpi.csv")
print(df[["elapsed_s","rx_bps","tx_bps","udp_sockets","max_rx_queue","rx_drop"]].describe())
EOF
```

Key columns:
| Column | Meaning |
|---|---|
| `rx_bps` / `tx_bps` | Interface bandwidth in bytes/s |
| `udp_sockets` | Number of open UDP sockets at each sample |
| `max_rx_queue` | Largest per-socket receive buffer fill (bytes) — non-zero = buffer pressure |
| `rx_drop` | Cumulative dropped receive packets — should stay at 0 |

### STM32 heap footprint (from serial CSV)

```bash
python3 - << 'EOF'
import pandas as pd
for f in ["ws_base/tools/poc_run/run_001/stm32_chassis.csv",
          "ws_base/tools/poc_run/run_001/stm32_sensors.csv"]:
    try:
        df = pd.read_csv(f)
        node = df["node"].iloc[0] if "node" in df.columns else f
        print(f"\n=== {node} ===")
        print(df[["ts_ms","heap_used","heap_max","heap_free","alloc_fail"]].describe())
    except FileNotFoundError:
        pass
EOF
```

Compare `heap_used` and `heap_max` between baseline and POC runs. An increase indicates the extra 5 participant slots (or extra RTPS endpoint tracking for 15 vs 11 remote participants) consuming more SRAM.

---

## Collecting Baseline Data for Comparison

To get a proper before/after comparison, repeat **Steps 2–6** on the `main` branch,
which runs the original multi-domain architecture with the same measurement tooling.
Baseline CSVs will be saved as `baseline_latency_rpi.csv` / `baseline_latency_jetson.csv`.

```bash
git checkout main
# Rebuild and re-flash both STM32 boards from main branch firmware
# Launch with main branch scripts, then:
TARGET_HOST=curry@192.168.1.1 TARGET_LABEL=rpi bash ws_base/tools/tracing/start_trace.sh
```

See the `README.md` on the `main` branch for the baseline launch guide.

---

## CSV Field Reference

Each collector writes a CSV with a fixed schema. This section is the authoritative
reference for every column in every output file.

---

### `latency_rpi.csv` / `latency_jetson.csv`

Produced by `collect_latency.py` running on the respective SBC.
One row per received message on any tracked topic.

| Column | Type | Description |
|---|---|---|
| `topic` | string | ROS2 topic name (e.g. `/tpc_chassis_imu`) |
| `recv_time_s` | float | Wall-clock time when the subscriber callback fired — `time.time()` (NTP-aligned UNIX epoch, seconds). This is the primary time anchor for cross-source alignment. |
| `header_stamp_s` | float \| empty | `msg.header.stamp` expressed as a UNIX epoch float (sec + nanosec×10⁻⁹). Populated only for message types that carry `header.stamp`: `sensor_msgs/Image` (camera topics) and `/tpc_telemetry_relay`. Empty for all other message types. |
| `latency_ms` | float \| empty | `(recv_time_s − header_stamp_s) × 1000`. End-to-end time from when the publisher stamped the message to when the subscriber received it. Populated whenever `header_stamp_s` is present. |
| `interval_ms` | float \| empty | Time elapsed since the previous message on the same topic, in ms. Empty for the very first message on each topic. Standard deviation of this column is the primary **jitter** metric. |

> **QoS note:** `collect_latency.py` reads the publisher's QoS reliability before
> subscribing. Topics published as `BEST_EFFORT` (camera nodes) get a matching
> `BEST_EFFORT` subscription; all others get `RELIABLE`. A mismatch would cause
> zero messages to be received with no error.

**Topics tracked (as of this branch):**

| Topic | Jitter | Latency | Source node |
|---|---|---|---|
| `/tpc_chassis_imu` | ✓ | — | chassis STM32 |
| `/tpc_chassis_sensors` | ✓ | — | chassis STM32 |
| `/tpc_chassis_cmd` | ✓ | — | chassis STM32 |
| `/tpc_gnss_spresense` | ✓ | — | sensors STM32 |
| `/tpc_gnss_ublox` | ✓ | — | sensors STM32 |
| `/tpc_rover_ctrl_cmd` | ✓ | — | Jetson kinematic control |
| `/tpc_telemetry_relay` | ✓ | ✓ | RPi relay node |
| `/tpc_rover_d415_rgb` | ✓ | ✓ | Jetson camera_stream_node (30 fps) |
| `/tpc_rover_d415_depth` | ✓ | ✓ | Jetson camera_stream_node (30 fps) |
| `/tpc_rover_nav_lane` | ✓ | — | Jetson lane_detection_node |

---

### `net_stats_rpi.csv` / `net_stats_jetson.csv`

Produced by `collect_net_stats.py` on each SBC. One row per sample interval (default 0.5 s).

| Column | Type | Description |
|---|---|---|
| `timestamp` | ISO 8601 UTC | Wall-clock at sample time (`datetime.utcnow().isoformat()`). NTP-aligned; used for cross-source time sync. |
| `elapsed_s` | float | Seconds since the collector started (monotonic). |
| `rx_bytes_delta` | int | Bytes received on `eth0` since the previous sample. |
| `tx_bytes_delta` | int | Bytes transmitted on `eth0` since the previous sample. |
| `rx_packets_delta` | int | Packets received since previous sample. |
| `tx_packets_delta` | int | Packets transmitted since previous sample. |
| `rx_errs` | int | Cumulative RX hardware errors (CRC, frame) at the NIC. Should stay 0. |
| `tx_errs` | int | Cumulative TX hardware errors. Should stay 0. |
| `rx_drop` | int | Cumulative RX packets dropped at the kernel NIC ring buffer. Non-zero = SBC falling behind at driver level. |
| `tx_drop` | int | Cumulative TX drops. |
| `rx_bps` | float | Receive bandwidth estimate: `rx_bytes_delta / interval_s` (bytes/s). |
| `tx_bps` | float | Transmit bandwidth estimate (bytes/s). |
| `udp_sockets` | int | Number of open UDP sockets (`/proc/net/udp`). DDS opens one socket per topic per participant; more participants → higher count. |
| `total_rx_queue` | int | Sum of unread bytes across all UDP socket receive buffers (`/proc/net/udp`). |
| `total_tx_queue` | int | Sum of pending bytes across all UDP socket send buffers. |
| `max_rx_queue` | int | Largest single UDP socket receive buffer fill (bytes). Non-zero means the application is not draining the kernel buffer fast enough — a DDS back-pressure indicator. |
| `max_tx_queue` | int | Largest single UDP socket send buffer fill (bytes). |
| `ss_udp_total` | int | Total UDP socket count from `ss -s` (cross-check against `udp_sockets`). |

---

### `topic_bw.csv`

Produced by `collect_topic_bw.py` on the base PC. One row per topic per 1-second interval.

| Column | Type | Description |
|---|---|---|
| `timestamp` | ISO 8601 UTC | Wall-clock at the end of the aggregation interval (`datetime.now(timezone.utc).isoformat()`). |
| `elapsed_s` | float | Seconds since the collector started. |
| `topic` | string | ROS2 topic name. |
| `msg_count` | int | Number of messages received in this interval. Zero indicates a gap (publisher stalled or dropped). |
| `bytes` | int | Total CDR-serialized bytes received in this interval. Measured via `serialize_message()` — same encoding as the DDS wire format, before RTPS/UDP framing. |
| `bps` | float | Bytes per second: `bytes / interval_s`. |
| `msg_per_s` | float | Messages per second: `msg_count / interval_s`. Cross-check against expected publish rate. |

> **Coverage:** auto-discovers all D5 topics every 3 s. Skips `/rosout`,
> `/parameter_events`, `/tf`, `/tf_static`. QoS is matched per-publisher
> (same mechanism as `collect_latency.py`).

---

### `stm32_chassis.csv` / `stm32_sensors.csv`

Produced by `collect_stm32_memory.py` on the base PC via USB serial. One row per JSON
telemetry line emitted by the STM32 firmware (~1 per second per board).

| Column | Type | Description |
|---|---|---|
| `wall_clock` | ISO 8601 UTC | Base PC wall-clock when the JSON line was received (`datetime.utcnow().isoformat()`). NTP-aligned; used as the time anchor (the STM32 has no RTC). |
| `type` | string | Always `STM32_MEM` — the firmware message type discriminator. |
| `node` | string | Board identity: `chassis` or `sensors`. |
| `ts_ms` | int | STM32 uptime in milliseconds since last reset. Useful for relative timing within a single board run; not suitable for cross-source alignment (use `wall_clock` for that). |
| `heap_used` | int | Current mbed heap allocated (bytes). Rises during RTPS discovery as participant proxy structs are allocated, then plateaus. |
| `heap_max` | int | Peak heap allocated since boot (bytes). Set during the 10-second discovery window and never decreases. **This is the primary POC comparison figure** — compare to baseline to see the cost of 20 vs 15 participants. |
| `heap_free` | int | `total_heap − heap_used` (bytes). Negative trend indicates a memory leak. |
| `alloc_fail` | int | Cumulative count of failed `malloc()` calls in the RTPS pool. Any non-zero value is a critical failure — the RTPS pool is exhausted. |
| `stack_free` | int | Minimum free stack observed across all RTOS threads (bytes). Must stay positive; too-small headroom risks stack overflow under domain-consolidation load. |

---

### `merged_all.csv`

Produced by `merge_run_csv.py`. One row per time bucket (default 1 second).
All metrics from all sources are aligned to a common `elapsed_s` x-axis.

Columns are grouped by type. `<topic>` in column names is the topic name with the
leading `/` stripped and any inner `/` replaced by `__`.

| Column group | Example column | Content |
|---|---|---|
| Time | `elapsed_s` | Seconds since the earliest event across all collectors (NTP-aligned). |
| Inter-arrival jitter — RPi | `interval_rpi__tpc_chassis_imu__mean_ms` | Mean `interval_ms` across all messages in the bucket for this topic on RPi. |
| Inter-arrival jitter p95 — RPi | `interval_rpi__tpc_chassis_imu__p95_ms` | 95th-percentile `interval_ms` in the bucket. |
| Inter-arrival jitter — Jetson | `interval_jetson__tpc_rover_d415_rgb__mean_ms` | Same for Jetson-side subscriber. |
| End-to-end latency — RPi | `latency_rpi__tpc_telemetry_relay__mean_ms` | Mean `latency_ms` in the bucket. Only populated for topics that carry `header.stamp`. |
| End-to-end latency — Jetson | `latency_jetson__tpc_rover_d415_rgb__mean_ms` | Camera publish-to-receive latency. |
| Network RX/TX — RPi | `net_rpi__rx_kbps` | Mean `rx_bps / 1024` across samples in the bucket. |
| Network RX/TX — Jetson | `net_jetson__tx_kbps` | Mean `tx_bps / 1024` across samples in the bucket. |
| Per-topic bandwidth | `bw__tpc_rover_d415_rgb__kbps` | Mean CDR KB/s for the topic in the bucket. |
| STM32 chassis heap | `stm32_chassis__heap_used_kb` | Mean `heap_used / 1024` across samples in the bucket. |
| STM32 chassis heap free | `stm32_chassis__heap_free_kb` | Mean `heap_free / 1024` across samples in the bucket. |
| STM32 sensors heap | `stm32_sensors__heap_used_kb` | Same for the sensors board. |

> Missing data (no samples in a bucket, or file not present) is represented as an
> empty string — not zero. Filter for non-empty values before computing statistics.

---

## Files Created in This Branch

```
ws_rpi/
  launch_rover_single_domain.sh       # all D5, session 'rover_poc'
ws_jetson/
  launch_jetson_single_domain.sh      # all D5 (was D4+D5+D6)
ws_base/
  launch_base_single_domain.sh        # all D5 (monitoring was D4)
  tools/
    poc_run/
      .gitignore                      # ignores generated run_*/ directories
      merge_run_csv.py                # 1s-bucketed union of all run CSVs → merged_all.csv
    tracing/
      setup_tracing.sh                # pre-flight environment check on SBC
      collect_latency.py              # rclpy subscriber → latency/jitter CSV
      start_trace.sh                  # SSH wrapper: launch collect_latency.py on SBC
      stop_and_collect_trace.sh       # SSH wrapper: stop collector + scp CSV to base PC
                                      #   (accepts LOCAL_DEST_CSV env var for output path)
      analyze_latency.py              # CSV → jitter/latency stats + boxplot + unified timeline
                                      #   (--run-dir shorthand auto-discovers all CSVs)
    monitoring/
      collect_net_stats.py            # /proc/net/dev + udp queue → CSV
      collect_topic_bw.py             # rclpy: auto-discovers topics, serialize_message() → bps CSV
    stm32_serial/
      collect_stm32_memory.py         # USB serial → STM32_MEM JSON → CSV
mros2-mbed-chassis-dynamics/
  platform/rtps/config.h             # MAX_NUM_PARTICIPANTS 15→20
  workspace/chassis_controller/
    app.cpp                           # +include memory_reporter.h, +reporter_start
    memory_reporter.h                 # new: heap/stack JSON reporter thread
mros2-mbed-sensors-gnss/
  platform/rtps/config.h             # MAX_NUM_PARTICIPANTS 15→20
  workspace/sensors_node/
    app.cpp                           # +include memory_reporter.h, +reporter_start
    memory_reporter.h                 # new: heap/stack JSON reporter thread
```
