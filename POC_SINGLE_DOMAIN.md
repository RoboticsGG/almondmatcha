# POC — Single ROS2 Domain ID=5: Measurement Guide

**Branch:** `single-domain`  
**Goal:** Measure how collapsing D4+D5+D6 → all D5 affects message latency, publish jitter, socket buffer pressure, interface bandwidth on the Linux SBCs (RPi + Jetson), and heap/stack footprint on both STM32 Nucleo boards.

---

## Quick Start

> **Prerequisites:** STM32 firmware flashed, all three machines built (see [Prerequisites](#prerequisites) below), `pip3 install pandas numpy matplotlib` on base PC.

### Step 1 — Verify clock sync and connectivity

```bash
# On base PC — check chrony is in sync on all machines
chronyc tracking | grep -E "Reference|System time|RMS offset"
ssh curry@192.168.1.1 'chronyc tracking | grep -E "System time|RMS offset"'
ssh yupi@192.168.1.5  'chronyc tracking | grep -E "System time|RMS offset"'
```

Acceptable thresholds and what they mean:

| Metric | Good | Borderline | Impact if over |
|---|---|---|---|
| System time | < 2 ms | 2–10 ms | Cross-host `latency_ms` has systematic offset — jitter (`interval_ms`) unaffected |
| RMS offset | < 5 ms | 5–15 ms | Time-axis alignment in `merged_all.csv` slightly fuzzy vs. 1 s bucket size |

> For this POC the primary metric is **relative change** between branches, not absolute values. A consistent 5–6 ms offset on one host is acceptable — it will be present in both single-domain and multi-domain runs equally.

If any host is over ~10 ms, force an immediate step correction:
```bash
ssh yupi@192.168.1.5 'sudo chronyc makestep'
# Re-check after ~5 s
ssh yupi@192.168.1.5 'chronyc tracking | grep -E "System time|RMS offset"'
```

#### One-time setup — use RPi as LAN NTP server (recommended)

This routes all three hosts through the same low-latency LAN source, bringing RMS offset below 1 ms. Only needs to be done once.

**On RPi** — allow LAN clients to use it as a time source:
```bash
ssh curry@192.168.1.1
sudo tee -a /etc/chrony/chrony.conf << 'EOF'
allow 192.168.1.0/24
local stratum 4
EOF
sudo systemctl restart chrony
chronyc clients   # no error = serving OK
```

**On Jetson** — prefer RPi as time source:
```bash
ssh yupi@192.168.1.5
sudo sed -i '1s/^/server 192.168.1.1 iburst prefer\n/' /etc/chrony/chrony.conf
sudo systemctl restart chrony
sleep 30
chronyc tracking | grep -E "Reference|System time|RMS offset"
# Reference ID should show 192.168.1.1
```

**On base PC** — prefer RPi as time source:
```bash
sudo sed -i '1s/^/server 192.168.1.1 iburst prefer\n/' /etc/chrony/chrony.conf
sudo systemctl restart chrony
sleep 30
chronyc tracking | grep -E "Reference|System time|RMS offset"
```

**Verify all three are using RPi:**
```bash
ssh curry@192.168.1.1 'chronyc clients'                         # should list 192.168.1.5 + base PC IP
ssh yupi@192.168.1.5  'chronyc tracking | grep Reference'       # should show 192.168.1.1
chronyc tracking | grep Reference                               # base PC — should show 192.168.1.1
```

> **Fallback:** If RPi loses internet, `local stratum 4` keeps it serving its own clock to LAN clients rather than stopping. Jetson and base PC retain the last known good offset in the meantime.

```bash
# Connectivity + ROS2 environment check
bash ws_base/tools/check_connectivity.sh
```

### Step 2 — Identify STM32 serial ports

```bash
minicom -b 115200 -D /dev/ttyACM0   # check "app name:" line
minicom -b 115200 -D /dev/ttyACM1
# Note which is chassis (192.168.1.2) and which is sensors (192.168.1.6)
```

### Step 3 — Run the experiment

```bash
cd ~/almondmatcha
bash ws_base/launch_poc_experiment.sh
# or override duration / ports:
bash ws_base/launch_poc_experiment.sh --duration 600 --chassis /dev/ttyACM1 --sensors /dev/ttyACM0
```

The script runs the full sequence automatically: starts all collectors, waits for STM32 topics, runs the timed measurement window, stops and pulls all CSVs, then calls post_run.sh.

### Step 4 — Review results (auto-generated at end of run)

All output files land in `ws_base/runs/single_domain/run_NNN/`:

| File | Description |
|---|---|
| `merged_all.csv` | 1-second NTP-aligned time-bucketed wide CSV (all sources) |
| `merged_flat.csv` | Un-bucketed raw event log (full resolution, all sources) |
| `latency_summary.csv` | Per-topic jitter/latency stats table (mean, p50, p95, p99) |
| `unified_timeline.png` | Multi-panel chart: jitter, STM32 heap, network BW, per-topic BW |
| `latency_rpi.csv` etc. | Individual raw collector outputs |

### Step 5 — Re-run post-processing or compare runs

```bash
# Re-generate all result files from a completed run at any time:
bash ws_base/tools/post_run.sh ws_base/runs/single_domain/run_001

# Side-by-side comparison with a baseline (multi-domain) run:
bash ws_base/tools/post_run.sh ws_base/runs/single_domain/run_001 \
    --compare ws_base/runs/multi_domain/run_001
# Produces: run_001/jitter_boxplot.png
```

---

## Topology Change Summary

| | Baseline (multi-domain) | POC (this branch) |
|---|---|---|
| Domain 4 | monitoring (Base + Jetson + RPi pub) — 3 nodes | ✗ removed (all on D5) |
| Domain 5 | ~12 participants (control + STM32) | **~16 participants** (all nodes) |
| Domain 6 | vision (Jetson localhost, shared memory) — 2 nodes | ✗ removed (all on D5) |
| Camera/lane images@30fps on network | **no** (D6 shared memory) | **yes** (D5 stress factor) |
| STM32 sees D5 participants | ~12 | ~16 |
| STM32 SPDP_MAX | 30 (18 margin) | 30 (14 margin) |

---

## Clock Synchronization

All Linux hosts (base PC, RPi, Jetson) must be time-synchronized before any cross-host latency measurement is valid.

### Current mechanism: chrony (NTP)

All three machines run `chronyd` synchronized to a public NTP pool via the local router.

**Expected accuracy:** 1–10 ms LAN-to-LAN peer offset. This is the **noise floor** of every end-to-end latency measurement in this POC. Any measured `latency_ms` value smaller than ~5 ms should be treated as inconclusive — it may be real latency or clock-sync error.

**Verify sync before every run (on each machine):**
```bash
chronyc tracking | grep -E "Reference|System time|RMS offset|Stratum"
# Target: System time < 2 ms, RMS offset < 5 ms, Stratum ≤ 3
```

If offset is > 10 ms, force an immediate step correction:
```bash
sudo chronyc makestep
```

**Is NTP accurate enough for this POC?**
Yes. The domain-consolidation effects being measured operate in the 5–50 ms range (DDS discovery overhead, extra multicast replication). The ~5 ms NTP noise floor does not obscure these deltas. PTP would only add value if comparing sub-millisecond DDS dispatch jitter between branches.

### Improving accuracy (optional — PTP)

If sub-millisecond accuracy is needed:

| Method | Typical LAN accuracy | Requirement |
|---|---|---|
| `chrony` + NTP pool (current) | 2–10 ms | Nothing extra |
| `chrony` + local Stratum-1 GPS server on base PC | 0.5–2 ms | `gpsd` + GPS receiver on base PC |
| **PTP (IEEE 1588)** via `linuxptp` (`ptp4l` + `phc2sys`) | < 100 µs | NICs with hardware timestamping; managed switch with PTP support |

```bash
# To set up PTP on base PC as master (grandmaster clock):
sudo apt install linuxptp
sudo ptp4l -i eth0 -m &          # PTP master on base PC
sudo phc2sys -s eth0 -c CLOCK_REALTIME -O 0 -m &   # sync system clock to PHC

# On RPi / Jetson (slave mode):
sudo ptp4l -i eth0 -m -s &       # -s = slave-only mode
sudo phc2sys -s eth0 -c CLOCK_REALTIME -O 0 -m &
```

### `ros2 topic delay` vs `latency_ms` in `collect_latency.py`

`ros2 topic delay <topic>` measures **message age** — `now − msg.header.stamp` — displayed live in a terminal. This is **identical** to the `latency_ms` column written by `collect_latency.py`.

```bash
# Live diagnostic during a run (works for any topic with header.stamp):
ros2 topic delay /tpc_telemetry_relay
ros2 topic delay /tpc_rover_d415_rgb
```

| | `ros2 topic delay` | `latency_ms` in `collect_latency.py` |
|---|---|---|
| Saves to CSV | No | Yes |
| Requires header.stamp | Yes | Yes (only for those topics) |
| Covers all topics (jitter) | No | Yes (`interval_ms` for all) |
| Best used for | Live diagnostic during setup | Archival comparison analysis |

**Recommendation:** use `ros2 topic delay` during setup to confirm the system is operating in the expected latency range, then rely on `collect_latency.py` for the recorded CSV used in baseline vs POC comparison.

Both require synchronized clocks — see above.

---

## What is Measured

Seven independent collectors run in parallel. Each targets a different layer of the system.

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

> **`ros2 topic delay` is equivalent** to `latency_ms` in this collector — see the
> **Clock Synchronization** section above for a direct comparison.

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

### 5. ROS2 message drop rate — `ros2 topic hz` (on SBC, during run)

Source: `ros2 topic hz` counts messages received per second and reports observed rate, min/max delta, and std-dev.

**Drop rate** = `(expected_Hz − observed_Hz) / expected_Hz × 100%`

Run on **both RPi and Jetson** during the experiment. Expected rates for reference:

| Topic | Expected rate | Node |
|---|---|---|
| `/tpc_chassis_imu` | 10 Hz | chassis STM32 |
| `/tpc_chassis_sensors` | 4 Hz | sensors STM32 |
| `/tpc_rover_ctrl_cmd` | 10 Hz | Jetson kinematic control |
| `/tpc_rover_d415_rgb` | 30 Hz | Jetson camera |
| `/tpc_rover_d415_depth` | 30 Hz | Jetson camera |

```bash
# SSH into RPi or Jetson
source /opt/ros/humble/setup.bash
source ~/almondmatcha/ws_rpi/install/setup.bash   # or ws_jetson
export ROS_DOMAIN_ID=5

# Live per-topic rate + jitter std-dev (Ctrl-C to stop):
ros2 topic hz /tpc_chassis_imu     --window 50
ros2 topic hz /tpc_chassis_sensors --window 50
ros2 topic hz /tpc_rover_ctrl_cmd  --window 50

# Batch report — capture 10-second window for every topic, save to file:
for topic in /tpc_chassis_imu /tpc_chassis_sensors /tpc_rover_ctrl_cmd \
             /tpc_rover_d415_rgb /tpc_rover_d415_depth; do
  echo "--- $topic ---"
  timeout 10 ros2 topic hz $topic --window 50 2>/dev/null || echo "(no messages)"
done | tee ~/almondmatcha_poc/hz_report_$(hostname)_$(date +%Y%m%d_%H%M%S).txt
```

`ros2 topic hz` output per topic:
```
average rate: 9.97
        min: 0.098s  max: 0.104s  std dev: 0.00212s  window: 50
```

> **`std dev` here = inter-arrival jitter**, the same as `interval_ms` std-dev from `collect_latency.py`. Use `ros2 topic hz` for a live spot-check during a run; use `collect_latency.py` for the full archival CSV record.

Pull the hz report to base PC after the run:
```bash
# The automated script (launch_poc_experiment.sh) pulls these automatically.
# If running manually:
scp curry@192.168.1.1:~/almondmatcha_poc/hz_report_*.txt \
    ws_base/runs/single_domain/run_NNN/
scp yupi@192.168.1.5:~/almondmatcha_poc/hz_report_*.txt \
    ws_base/runs/single_domain/run_NNN/
```

---

### 6. CPU load and memory — RPi and Jetson

#### RPi — one-liner CSV logger

Logs CPU %, memory %, and CPU temperature to CSV at 1-second intervals.

```bash
# SSH into RPi
ssh curry@192.168.1.1
mkdir -p ~/almondmatcha_poc

echo "timestamp,cpu_pct,mem_pct,temp_c" | tee ~/almondmatcha_poc/cpu_rpi.csv && \
while sleep 1; do
  echo "$(date +%s),$(top -bn1 | grep 'Cpu(s)' | awk '{print 100 - $8}'),$(free | awk '/Mem/ {printf "%.2f", $3/$2*100}'),$(cat /sys/class/thermal/thermal_zone0/temp | awk '{print $1/1000}')"
done | tee -a ~/almondmatcha_poc/cpu_rpi.csv
```

#### Jetson — tegrastats logger

`tegrastats` outputs a continuous one-line status with CPU cluster loads, GPU, EMC (memory bandwidth), and thermal zones. Two output formats:

**Full raw log with prepended timestamp** (recommended — parse later):
```bash
# SSH into Jetson
ssh yupi@192.168.1.5
mkdir -p ~/almondmatcha_poc

# Log tegrastats at 1 s intervals, prepend UNIX timestamp to each line
tegrastats --interval 1000 | \
  while IFS= read -r line; do
    echo "$(date +%s) $line"
  done | tee ~/almondmatcha_poc/cpu_jetson_tegrastats.txt
```

**Compact CSV** (CPU clusters, GPU, temperatures):
```bash
tegrastats --interval 1000 | \
  awk '
    BEGIN { print "timestamp,cpu1_pct,cpu2_pct,gpu_pct,temp_cpu_c,temp_gpu_c" }
    {
      ts = systime()
      match($0, /CPU \[([0-9]+)%@[0-9]+,[0-9]+%@[0-9]+,[0-9]+%@[0-9]+,[0-9]+%@[0-9]+,([0-9]+)%@[0-9]+,([0-9]+)%@[0-9]+\]/, c)
      match($0, /GR3D_FREQ ([0-9]+)%/, g)
      match($0, /CPU@([0-9.]+)C/, tc)
      match($0, /GPU@([0-9.]+)C/, tg)
      printf "%d,%s,%s,%s,%s,%s\n", ts, c[1], c[5], g[1], tc[1], tg[1]
    }
  ' | tee ~/almondmatcha_poc/cpu_jetson.csv
```

> **tegrastats key fields:** `CPU [%@MHz × 6 cores]` (across 2 clusters), `GR3D_FREQ %` (GPU), `EMC_FREQ %` (memory bus utilization), `VDD_IN` (total board power in mW), `CPU@`, `GPU@`, `SOC@` thermal zones. Parsing note: the 6-core format on Jetson Orin/Xavier uses two bracket groups — use the raw log file if the awk regex does not match your board's exact format.

Pull after the run:
```bash
# The automated script pulls these automatically.
# If running manually:
scp curry@192.168.1.1:~/almondmatcha_poc/cpu_rpi.csv \
    ws_base/runs/single_domain/run_NNN/cpu_rpi.csv
scp yupi@192.168.1.5:~/almondmatcha_poc/cpu_jetson_tegrastats.txt \
    ws_base/runs/single_domain/run_NNN/cpu_jetson_tegrastats.txt
scp yupi@192.168.1.5:~/almondmatcha_poc/cpu_jetson.csv \
    ws_base/runs/single_domain/run_NNN/cpu_jetson.csv
```

---

### 7. Linux network SoftIRQ busyness — RPi and Jetson

SoftIRQ counts reveal how busy the kernel's network receive path is. Under domain consolidation, more DDS multicast streams arrive on the same NIC, increasing `NET_RX` firings. High `NET_RX_delta` combined with a rising `max_rx_queue` (§2) is the clearest sign of receive-path saturation — the kernel is being interrupted more frequently to demultiplex multicast packets even if raw bandwidth (bytes/s) has not changed.

**Live terminal watch during a run:**
```bash
# SSH into RPi or Jetson — run in a dedicated pane
watch -n 1 "grep -E 'NET_RX|NET_TX|TIMER|SCHED' /proc/softirqs | \
  awk '{printf \"%s: \", \$1; sum=0; for (i=2; i<=NF; i++) sum+=\$i; printf \"%d\n\", sum}'"
```

**CSV logger — per-second delta counts:**
```bash
# Run on RPi (change filename to softirq_jetson.csv on Jetson)
mkdir -p ~/almondmatcha_poc
(
  echo "timestamp,NET_RX_delta,NET_TX_delta,SCHED_delta,TIMER_delta"
  prev_rx=0; prev_tx=0; prev_sched=0; prev_timer=0
  while sleep 1; do
    ts=$(date +%s)
    rx=$(grep 'NET_RX:' /proc/softirqs | awk '{s=0; for(i=2;i<=NF;i++) s+=$i; print s}')
    tx=$(grep 'NET_TX:' /proc/softirqs | awk '{s=0; for(i=2;i<=NF;i++) s+=$i; print s}')
    sched=$(grep 'SCHED:' /proc/softirqs | awk '{s=0; for(i=2;i<=NF;i++) s+=$i; print s}')
    timer=$(grep 'TIMER:' /proc/softirqs | awk '{s=0; for(i=2;i<=NF;i++) s+=$i; print s}')
    echo "$ts,$((rx-prev_rx)),$((tx-prev_tx)),$((sched-prev_sched)),$((timer-prev_timer))"
    prev_rx=$rx; prev_tx=$tx; prev_sched=$sched; prev_timer=$timer
  done
) | tee ~/almondmatcha_poc/softirq_rpi.csv
```

Pull after the run:
```bash
# The automated script pulls these automatically.
# If running manually:
scp curry@192.168.1.1:~/almondmatcha_poc/softirq_rpi.csv \
    ws_base/runs/single_domain/run_NNN/softirq_rpi.csv
scp yupi@192.168.1.5:~/almondmatcha_poc/softirq_jetson.csv \
    ws_base/runs/single_domain/run_NNN/softirq_jetson.csv
```

> **Interpreting `NET_RX_delta`:** each value is the number of receive softIRQ firings in the last second. In the baseline (multi-domain), topics are split across domains so multicast streams are partitioned. In the POC (single-domain), all topics share one multicast group on one NIC — if `NET_RX_delta` rises significantly vs baseline with no proportional increase in `rx_bps`, the extra overhead is from increased interrupt-to-socket-demux overhead, not bandwidth.

---

| Collector | Script / command | Runs on | Key question |
|---|---|---|---|
| ROS2 message timing | `collect_latency.py` | RPi, Jetson | Does single-domain increase jitter or latency? |
| Linux NIC / socket buffers | `collect_net_stats.py` | RPi, Jetson | Do extra participants fill UDP buffers or drop packets? |
| STM32 RTPS heap | `collect_stm32_memory.py` | Base PC (USB serial) | Does `MAX_NUM_PARTICIPANTS=20` leave enough heap headroom? |
| Per-topic bandwidth | `collect_topic_bw.py` | Base PC (ROS2 subscriber) | How many KB/s does each DDS topic contribute on the wire? |
| Message drop rate | `ros2 topic hz` (manual) | RPi, Jetson | Are messages being dropped at the ROS2 layer? |
| CPU load | one-liner CSV / `tegrastats` | RPi, Jetson | Does domain consolidation increase CPU utilization? |
| SoftIRQ busyness | `/proc/softirqs` delta CSV | RPi, Jetson | Is the kernel network receive path saturated? |

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

### 5. Switching to/from the multi-domain branch

**No rebuild is required when switching branches.** Both `single-domain` and `multi-domain` carry identical C++ source code — the only files that differ between them are launch scripts, docs, and `launch_poc_experiment.sh`. The compiled `build/` and `install/` trees are valid on both branches.

```bash
# On the base PC (and repeat on RPi / Jetson via SSH if their working trees are checked out)
git stash           # stash any local edits if needed
git checkout multi-domain
```

After switching:
- **No rebuild needed** on any machine
- **poc_run data is preserved** — `ws_base/runs/single_domain/` and `ws_base/runs/multi_domain/` are gitignored subdirectories and are unaffected by `git checkout`
- Use `ws_rpi/launch_rover_multi_domain.sh`, `ws_jetson/launch_jetson_multi_domain.sh`, `ws_base/launch_base_multi_domain.sh` on the multi-domain branch
- `launch_poc_experiment.sh` on the multi-domain branch already references those scripts and writes to `runs/multi_domain/`

To switch back:
```bash
git checkout single-domain
# No rebuild needed — launch scripts and poc_run path revert automatically
```

> **Important — keep the working trees in sync:** Each machine (RPi, Jetson) keeps its own
> git working tree. When you switch branches on the base PC, SSH into each SBC and run
> `git checkout <branch>` there as well. Otherwise the old launch scripts will be used.

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
| Launch ROS2 nodes | SSHes into RPi (8 nodes, 2 s stagger) → waits 20 s → Jetson (4 nodes) → waits 10 s → base PC (2 nodes). Staggered to avoid saturating STM32 discovery. |
| Start latency collectors | Runs `start_trace.sh` on RPi and Jetson via SSH |
| Start topic-BW collector | Runs `collect_topic_bw.py` locally on base PC (auto-discovers all D5 topics) |
| Start net-stats collectors | Runs `collect_net_stats.py` on both SBCs via SSH background `nohup` |
| Timed run | Live dashboard shows elapsed time, STM32 heap values, and collector health; Ctrl-C stops early and still collects CSVs |
| Stop and pull | Stops all collectors, pulls CSVs to base PC, runs `merge_run_csv.py`, prints analysis commands |

**Output files — all in one numbered run directory on the base PC:**
```
ws_base/runs/single_domain/run_NNN/
  latency_rpi.csv          # collect_latency.py — RPi inter-arrival + latency
  latency_jetson.csv       # collect_latency.py — Jetson inter-arrival + latency
  net_stats_rpi.csv        # collect_net_stats.py — RPi NIC counters
  net_stats_jetson.csv     # collect_net_stats.py — Jetson NIC counters
  topic_bw.csv             # collect_topic_bw.py — per-topic CDR bandwidth (D5)
  stm32_chassis.csv        # collect_stm32_memory.py — chassis board heap
  stm32_sensors.csv        # collect_stm32_memory.py — sensors board heap
  cpu_rpi.csv              # CPU/mem/temp on RPi (1s intervals)
  cpu_jetson_tegrastats.txt # full tegrastats log on Jetson (1s intervals)
  softirq_rpi.csv          # NET_RX/TX/SCHED/TIMER softIRQ deltas on RPi
  softirq_jetson.csv       # same on Jetson
  hz_report_rpi.txt        # mid-run ros2 topic hz from RPi
  hz_report_jetson.txt     # mid-run ros2 topic hz from Jetson
  # —— post_run.sh produces ——
  merged_all.csv           # 1s NTP-aligned time-bucketed wide CSV
  merged_flat.csv          # un-bucketed raw events (full resolution)
  latency_summary.csv      # per-topic stats: mean, p50, p95, p99, max
  unified_timeline.png     # multi-panel chart
  logs/                    # stdout/stderr from every sub-process
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
python3 ws_base/tools/collect_stm32_memory.py \
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
  "python3 ~/almondmatcha/ws_base/tools/collect_net_stats.py \
     --out ~/ros2_traces/net_stats_rpi.csv"

# Terminal B — Jetson
ssh yupi@192.168.1.5 \
  "python3 ~/almondmatcha/ws_base/tools/collect_net_stats.py \
     --out ~/ros2_traces/net_stats_jetson.csv"
```

> **Note:** `--iface` is optional — the script auto-detects the correct network interface based on the 192.168.1.0/24 subnet.

#### Step 4b — Start CPU load logging on each SBC

Open two additional SSH terminals (or tmux panes):

```bash
# Terminal C — RPi CPU logger
ssh curry@192.168.1.1 "mkdir -p ~/almondmatcha_poc && \
  bash -c 'echo timestamp,cpu_pct,mem_pct,temp_c | tee ~/almondmatcha_poc/cpu_rpi.csv && \
  while sleep 1; do echo \"\$(date +%s),\$(top -bn1 | grep Cpu | awk \"{print 100-\\\$8}\"),\$(free | awk \"/Mem/{printf \\\"%.2f\\\",\\\$3/\\\$2*100}\"),\$(cat /sys/class/thermal/thermal_zone0/temp | awk \"{print \\\$1/1000}\")\" ; done | tee -a ~/almondmatcha_poc/cpu_rpi.csv'"

# Terminal D — Jetson CPU logger (tegrastats raw)
ssh yupi@192.168.1.5 "mkdir -p ~/almondmatcha_poc && \
  tegrastats --interval 1000 | \
  while IFS= read -r line; do echo \"\$(date +%s) \$line\"; done \
  | tee ~/almondmatcha_poc/cpu_jetson_tegrastats.txt"
```

#### Step 4c — Start SoftIRQ logging on each SBC

```bash
# Terminal E — RPi SoftIRQ
ssh curry@192.168.1.1 "mkdir -p ~/almondmatcha_poc && bash -c '
  echo timestamp,NET_RX_delta,NET_TX_delta,SCHED_delta,TIMER_delta | tee ~/almondmatcha_poc/softirq_rpi.csv
  prev_rx=0; prev_tx=0; prev_sched=0; prev_timer=0
  while sleep 1; do
    ts=\$(date +%s)
    rx=\$(grep NET_RX: /proc/softirqs | awk \"{s=0;for(i=2;i<=NF;i++)s+=\\\$i;print s}\")
    tx=\$(grep NET_TX: /proc/softirqs | awk \"{s=0;for(i=2;i<=NF;i++)s+=\\\$i;print s}\")
    sc=\$(grep SCHED: /proc/softirqs | awk \"{s=0;for(i=2;i<=NF;i++)s+=\\\$i;print s}\")
    ti=\$(grep TIMER: /proc/softirqs | awk \"{s=0;for(i=2;i<=NF;i++)s+=\\\$i;print s}\")
    echo \"\$ts,\$((rx-prev_rx)),\$((tx-prev_tx)),\$((sc-prev_sched)),\$((ti-prev_timer))\"
    prev_rx=\$rx; prev_tx=\$tx; prev_sched=\$sc; prev_timer=\$ti
  done | tee -a ~/almondmatcha_poc/softirq_rpi.csv'"

# Terminal F — Jetson SoftIRQ (same command, different output file)
ssh yupi@192.168.1.5 "mkdir -p ~/almondmatcha_poc && bash -c '
  echo timestamp,NET_RX_delta,NET_TX_delta,SCHED_delta,TIMER_delta | tee ~/almondmatcha_poc/softirq_jetson.csv
  prev_rx=0; prev_tx=0; prev_sched=0; prev_timer=0
  while sleep 1; do
    ts=\$(date +%s)
    rx=\$(grep NET_RX: /proc/softirqs | awk \"{s=0;for(i=2;i<=NF;i++)s+=\\\$i;print s}\")
    tx=\$(grep NET_TX: /proc/softirqs | awk \"{s=0;for(i=2;i<=NF;i++)s+=\\\$i;print s}\")
    sc=\$(grep SCHED: /proc/softirqs | awk \"{s=0;for(i=2;i<=NF;i++)s+=\\\$i;print s}\")
    ti=\$(grep TIMER: /proc/softirqs | awk \"{s=0;for(i=2;i<=NF;i++)s+=\\\$i;print s}\")
    echo \"\$ts,\$((rx-prev_rx)),\$((tx-prev_tx)),\$((sc-prev_sched)),\$((ti-prev_timer))\"
    prev_rx=\$rx; prev_tx=\$tx; prev_sched=\$sc; prev_timer=\$ti
  done | tee -a ~/almondmatcha_poc/softirq_jetson.csv'"
```

#### Step 5 — Let it run

Run for **at least 5 minutes** under representative load (send a mission command, drive the rover). Optionally spot-check drop rate live from another terminal:
```bash
# On RPi or Jetson — quick drop rate check during the run
source /opt/ros/humble/setup.bash && source ~/almondmatcha/ws_rpi/install/setup.bash
export ROS_DOMAIN_ID=5
ros2 topic hz /tpc_chassis_imu --window 50
ros2 topic hz /tpc_chassis_sensors --window 50
```

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

# Stop CPU loggers (Ctrl-C in Terminal C/D), then pull:
scp curry@192.168.1.1:~/almondmatcha_poc/cpu_rpi.csv               /tmp/cpu_rpi.csv
scp yupi@192.168.1.5:~/almondmatcha_poc/cpu_jetson_tegrastats.txt   /tmp/cpu_jetson_tegrastats.txt
scp yupi@192.168.1.5:~/almondmatcha_poc/cpu_jetson.csv              /tmp/cpu_jetson.csv

# Stop SoftIRQ loggers (Ctrl-C in Terminal E/F), then pull:
scp curry@192.168.1.1:~/almondmatcha_poc/softirq_rpi.csv     /tmp/softirq_rpi.csv
scp yupi@192.168.1.5:~/almondmatcha_poc/softirq_jetson.csv   /tmp/softirq_jetson.csv

# Stop STM32 collector (Ctrl-C in the Step 1 terminal)
# Stop topic-BW collector (Ctrl-C if running)
```

> **Tip:** the automated script (Option A) handles all of the above automatically,
> including routing all files into `ws_base/runs/single_domain/run_NNN/` and calling
> `post_run.sh` at the end.

---

## Analyzing Results

> **Post-processing is automatic.** `launch_poc_experiment.sh` calls `post_run.sh` at the end of every run. The files below are already in your run directory. To regenerate or run the comparison:
> ```bash
> bash ws_base/tools/post_run.sh ws_base/runs/single_domain/run_001
> bash ws_base/tools/post_run.sh ws_base/runs/single_domain/run_001 --compare ws_base/runs/multi_domain/run_001
> ```

### Output files explained

| File | Format | Use case |
|---|---|---|
| `merged_all.csv` | Wide CSV, 1-second NTP-aligned buckets | Time-series comparison in pandas / spreadsheet; all sources on one x-axis |
| `merged_flat.csv` | Long CSV, one row per raw event | Full-resolution histogram, density plots, custom aggregation |
| `latency_summary.csv` | Wide CSV, one row per topic+metric | Quick stats table; compare mean/p95/p99 across runs |
| `unified_timeline.png` | PNG chart, 5 stacked panels | Visual sanity check; share in reports |

### Latency and jitter (from CSV)

The primary metric is **inter-arrival jitter** (std-dev of the time between consecutive
messages on each topic). This works for all topics and is the most relevant indicator
of DDS scheduling instability under domain consolidation.

**End-to-end latency** (publisher timestamp → subscriber receive time) is also available
for `/tpc_telemetry_relay` — the only project message type that includes `header.stamp`.

```bash
# Analyze POC run only (prints table to stdout + saves latency_summary.csv)
python3 ws_base/tools/tracing/analyze_latency.py \
  --csv ws_base/runs/single_domain/run_001/latency_rpi.csv

# Side-by-side comparison (requires baseline CSV from multi-domain branch)
python3 ws_base/tools/tracing/analyze_latency.py \
  --baseline ws_base/runs/multi_domain/run_001/latency_rpi.csv \
  --poc      ws_base/runs/single_domain/run_001/latency_rpi.csv \
  --out-dir  ws_base/runs/single_domain/run_001/

# Filter to specific topics only
python3 ws_base/tools/tracing/analyze_latency.py \
  --poc ws_base/runs/single_domain/run_001/latency_rpi.csv \
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
    --run-dir ws_base/runs/single_domain/run_001
# Produces: ws_base/runs/single_domain/run_001/unified_timeline.png
```

**Manual — specifying each file individually:**
```bash
python3 ws_base/tools/tracing/analyze_latency.py --merge \
    --latency-rpi    ws_base/runs/single_domain/run_001/latency_rpi.csv \
    --latency-jetson ws_base/runs/single_domain/run_001/latency_jetson.csv \
    --stm32          ws_base/runs/single_domain/run_001/stm32_chassis.csv \
    --stm32-sensors  ws_base/runs/single_domain/run_001/stm32_sensors.csv \
    --net-rpi        ws_base/runs/single_domain/run_001/net_stats_rpi.csv \
    --net-jetson     ws_base/runs/single_domain/run_001/net_stats_jetson.csv \
    --topic-bw       ws_base/runs/single_domain/run_001/topic_bw.csv \
    --out-dir        ws_base/runs/single_domain/run_001/
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
python3 ws_base/tools/merge_run_csv.py \
    --run-dir ws_base/runs/single_domain/run_001

# Finer time resolution (0.5-second buckets):
python3 ws_base/tools/merge_run_csv.py \
    --run-dir ws_base/runs/single_domain/run_001 --bucket-s 0.5
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
df = pd.read_csv("ws_base/runs/single_domain/run_001/merged_all.csv")
print(df.head())
print(df.describe())
EOF
```

### Socket buffer and bandwidth (from net_stats CSV)

Open in any spreadsheet or:

```bash
python3 - << 'EOF'
import pandas as pd
df = pd.read_csv("ws_base/runs/single_domain/run_001/net_stats_rpi.csv")
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
for f in ["ws_base/runs/single_domain/run_001/stm32_chassis.csv",
          "ws_base/runs/single_domain/run_001/stm32_sensors.csv"]:
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

## Replacing the Base PC

The base PC IP (`192.168.1.4`) is hardcoded in launch scripts and FastDDS XML. As long as the new machine uses the **same IP**, no scripts or SBC configuration need to change.

### Before decommissioning the old PC

Back up experiment run data — `ws_base/runs/` is gitignored and will not transfer via `git clone`:
```bash
cp -r ~/almondmatcha/ws_base/runs/ /path/to/backup/
```

### On the new base PC

**1. Clone and build:**
```bash
git clone <repo-url> ~/almondmatcha
cd ~/almondmatcha/ws_base && colcon build --symlink-install
pip3 install pandas numpy matplotlib pyserial
```

**2. Set up passwordless SSH to both SBCs:**
```bash
ssh-keygen -t ed25519        # skip if you already have a key
ssh-copy-id curry@192.168.1.1
ssh-copy-id yupi@192.168.1.5
```

**3. Re-add RPi as preferred NTP source:**
```bash
sudo sed -i '1s/^/server 192.168.1.1 iburst prefer\n/' /etc/chrony/chrony.conf
sudo systemctl restart chrony
sleep 30
chronyc tracking | grep Reference   # should show 192.168.1.1
```

**4. Restore run data (optional):**
```bash
mkdir -p ~/almondmatcha/ws_base/runs
cp -r /path/to/backup/runs/* ~/almondmatcha/ws_base/runs/
```

### On RPi and Jetson — clear the old host key (if applicable)

Only needed if either SBC ever SSH'd back to the base PC (e.g. reverse tunnels). Normally not required:
```bash
ssh curry@192.168.1.1 'ssh-keygen -R 192.168.1.4'
ssh yupi@192.168.1.5  'ssh-keygen -R 192.168.1.4'
```

> **STM32 boards, RPi, and Jetson need zero changes** — they communicate by IP only and do not track the base PC's identity.

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

### `cpu_rpi.csv`

Produced by the RPi one-liner CPU logger (§6). One row per second.

| Column | Type | Description |
|---|---|---|
| `timestamp` | int | UNIX epoch seconds (`date +%s`). |
| `cpu_pct` | float | Total CPU utilization % (100 − idle%, from `top -bn1`). Sum across all cores; divide by core count for per-core average. |
| `mem_pct` | float | Used memory as a percentage of total physical RAM (`free`). |
| `temp_c` | float | CPU die temperature in °C (`/sys/class/thermal/thermal_zone0/temp ÷ 1000`). |

---

### `cpu_jetson_tegrastats.txt`

Produced by `tegrastats --interval 1000` with prepended UNIX timestamp. One line per second.
Each line has the format: `<unix_ts> 12:34:56 RAM 1234/7782MB (lfb 512x4MB) CPU [5%@1190,3%@1190,...] ...`

Key fields to extract (with `awk` or `grep`):
- `CPU [...]` — per-core utilization % and frequency MHz
- `GR3D_FREQ %` — GPU utilization
- `EMC_FREQ %` — external memory controller (memory bandwidth) utilization
- `VDD_IN` — total board power draw in mW
- `CPU@`, `GPU@`, `SOC@` — thermal zone temperatures in °C

---

### `softirq_rpi.csv` / `softirq_jetson.csv`

Produced by the SoftIRQ delta logger (§7). One row per second.

| Column | Type | Description |
|---|---|---|
| `timestamp` | int | UNIX epoch seconds. |
| `NET_RX_delta` | int | Receive softIRQ firings in the last second across all CPUs. Each multicast DDS packet triggers at least one. Rising value vs baseline indicates more multicast demux overhead. |
| `NET_TX_delta` | int | Transmit softIRQ firings in the last second. |
| `SCHED_delta` | int | Scheduler softIRQ firings. Rises with thread count and context-switch rate. |
| `TIMER_delta` | int | Timer softIRQ firings. Steady background value; large spikes indicate timer storm. |

> **Cross-reference:** compare `NET_RX_delta` with `rx_packets_delta` from `net_stats_*.csv`. If `NET_RX_delta / rx_packets_delta > 1`, the kernel is firing multiple softIRQs per incoming packet (coalescing disabled or NIC GRO off) — indicates the NIC interrupt path is under pressure.

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
  runs/
    .gitignore                        # ignores generated run_*/ directories
  tools/
    post_run.sh                       # post-run automation: merge + stats + chart
    merge_run_csv.py                  # 1s-bucketed union of all run CSVs → merged_all.csv
    collect_net_stats.py              # /proc/net/dev + udp queue → CSV
    collect_topic_bw.py               # rclpy: auto-discovers topics, serialize_message() → bps CSV
    collect_stm32_memory.py           # USB serial → STM32_MEM JSON → CSV
    tracing/
      setup_tracing.sh                # pre-flight environment check on SBC
      collect_latency.py              # rclpy subscriber → latency/jitter CSV
      start_trace.sh                  # SSH wrapper: launch collect_latency.py on SBC
      stop_and_collect_trace.sh       # SSH wrapper: stop collector + scp CSV to base PC
      analyze_latency.py              # CSV → jitter/latency stats + boxplot + unified timeline
                                      #   (--run-dir shorthand auto-discovers all CSVs)
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
