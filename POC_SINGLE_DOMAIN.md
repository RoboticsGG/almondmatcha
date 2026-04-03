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

All checks must say `[OK]`. If `ws_rpi` or `ws_jetson` is not built yet, build it now:

```bash
# RPi (SSH in)
source /opt/ros/humble/setup.bash
source ~/almondmatcha/common_ifaces/install/setup.bash
cd ~/almondmatcha/ws_rpi && colcon build --symlink-install

# Jetson (SSH in)
source /opt/ros/humble/setup.bash
source ~/almondmatcha/common_ifaces/install/setup.bash
cd ~/almondmatcha/ws_jetson && colcon build --symlink-install
```

### 3. Python dependencies on base PC

```bash
pip3 install pandas numpy matplotlib
```

---

## Running the POC Experiment

### Step 1 — Start all nodes

Launch on each machine in this order (to give STM32 boards time to finish DDS discovery):

```bash
# STM32 boards: power-cycle both boards (they boot automatically)
# They now wait 10 s for participant discovery before publishing.

# RPi
ssh curry@192.168.1.1 "bash ~/almondmatcha/ws_rpi/launch_rover_single_domain.sh"

# Jetson
ssh yupi@192.168.1.5 "bash ~/almondmatcha/ws_jetson/launch_jetson_single_domain.sh"

# Base PC
bash ws_base/launch_base_single_domain.sh
```

The tmux session borders are **yellow/red** (vs blue/cyan in baseline) so you can tell at a glance which mode you are in.

### Step 2 — Start latency/jitter collection on RPi and Jetson

```bash
# From base PC — start collector on RPi (runs in background on SBC)
TARGET_HOST=curry@192.168.1.1 TARGET_LABEL=rpi bash ws_base/tools/tracing/start_trace.sh

# Start collector on Jetson
TARGET_HOST=yupi@192.168.1.5 TARGET_LABEL=jetson bash ws_base/tools/tracing/start_trace.sh
```

The collector (`collect_latency.py`) subscribes to all D5 topics and records per-message
timestamps to `~/ros2_traces/latency_<label>.csv` on the SBC. Topics appear automatically
as nodes come up — no manual topic registration needed.

### Step 3 — Start network stats collection on each SBC

Open two extra terminals on the base PC and SSH in:

```bash
# Terminal A — RPi network stats
ssh curry@192.168.1.1 \
  "python3 ~/almondmatcha/ws_base/tools/monitoring/collect_net_stats.py \
     --iface eth0 --out ~/ros2_traces/net_stats_rpi.csv"

# Terminal B — Jetson network stats
ssh yupi@192.168.1.5 \
  "python3 ~/almondmatcha/ws_base/tools/monitoring/collect_net_stats.py \
     --iface eth0 --out ~/ros2_traces/net_stats_jetson.csv"
```

### Step 4 — Start STM32 memory collection on base PC

Both STM32 boards are connected to the base PC via USB serial (`/dev/ttyACM0` chassis, `/dev/ttyACM1` sensors — verify with `ls /dev/ttyACM*`).

```bash
python3 ws_base/tools/stm32_serial/collect_stm32_memory.py \
  --chassis /dev/ttyACM0 --sensors /dev/ttyACM1 \
  --out ~/ros2_traces/stm32_memory_poc.csv
```

The collector filters only `{"type":"STM32_MEM",...}` JSON lines and ignores all other serial output. No suppression is needed in firmware.

### Step 5 — Let it run

Run for **at least 5 minutes** under representative load (send a mission command, drive the rover).

### Step 6 — Stop collection

```bash
# Stop latency collector and pull CSV to base PC
TARGET_HOST=curry@192.168.1.1 TARGET_LABEL=rpi    bash ws_base/tools/tracing/stop_and_collect_trace.sh
TARGET_HOST=yupi@192.168.1.5  TARGET_LABEL=jetson bash ws_base/tools/tracing/stop_and_collect_trace.sh
# Pulls to: ws_base/tools/tracing/data/poc_latency_rpi.csv
#           ws_base/tools/tracing/data/poc_latency_jetson.csv

# Stop net stats (Ctrl-C in each SSH terminal)
# Stop STM32 collector (Ctrl-C)

# Pull net stats CSVs back
scp curry@192.168.1.1:~/ros2_traces/net_stats_rpi.csv        ws_base/tools/monitoring/data/poc_net_stats_rpi.csv
scp yupi@192.168.1.5:~/ros2_traces/net_stats_jetson.csv      ws_base/tools/monitoring/data/poc_net_stats_jetson.csv
```

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
  --csv ws_base/tools/tracing/data/poc_latency_rpi.csv

# Side-by-side comparison (requires baseline CSV from multi-domain branch)
python3 ws_base/tools/tracing/analyze_latency.py \
  --baseline ws_base/tools/tracing/data/baseline_latency_rpi.csv \
  --poc      ws_base/tools/tracing/data/poc_latency_rpi.csv \
  --out-dir  ws_base/tools/tracing/results/

# Filter to specific topics only
python3 ws_base/tools/tracing/analyze_latency.py \
  --poc ws_base/tools/tracing/data/poc_latency_rpi.csv \
  --topics /tpc_chassis_imu /tpc_chassis_sensors /tpc_rover_ctrl_cmd
```

Output:
- Per-topic table: `mean`, `p50`, `p95`, `p99`, `max`, `std` of inter-arrival interval (ms)
- Per-topic latency stats for `/tpc_telemetry_relay` if present
- `latency_summary.csv` — machine-readable comparison table
- `jitter_boxplot.png` — box-plot when both `--baseline` and `--poc` are given

### Socket buffer and bandwidth (from net_stats CSV)

Open in any spreadsheet or:

```bash
python3 - << 'EOF'
import pandas as pd
df = pd.read_csv("ws_base/tools/monitoring/data/net_stats_rpi.csv")
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
import os
df = pd.read_csv(os.path.expanduser("~/ros2_traces/stm32_memory_poc.csv"))
for node, g in df.groupby("node"):
    print(f"\n=== {node} ===")
    print(g[["ts_ms","heap_used","heap_max","heap_free","alloc_fail"]].describe())
EOF
```

Compare `heap_used` and `heap_max` between baseline and POC runs. An increase indicates the extra 5 participant slots (or extra RTPS endpoint tracking for 15 vs 11 remote participants) consuming more SRAM.

---

## Collecting Baseline Data for Comparison

To get a proper before/after comparison, repeat **Steps 2–6** on the `multi-domain` branch,
which runs the original multi-domain architecture with the same measurement tooling.
Baseline CSVs will be saved as `baseline_latency_rpi.csv` / `baseline_latency_jetson.csv`.

```bash
git checkout multi-domain
# Rebuild and re-flash both STM32 boards from multi-domain firmware
# Launch with multi-domain scripts, then:
TARGET_HOST=curry@192.168.1.1 TARGET_LABEL=rpi bash ws_base/tools/tracing/start_trace.sh
```

See `MULTI_DOMAIN_BASELINE.md` on the `multi-domain` branch for the full guide.

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
    tracing/
      setup_tracing.sh                # pre-flight environment check on SBC
      collect_latency.py              # rclpy subscriber → latency/jitter CSV
      start_trace.sh                  # SSH wrapper: launch collect_latency.py on SBC
      stop_and_collect_trace.sh       # SSH wrapper: stop collector + scp CSV to base PC
      analyze_latency.py              # CSV → jitter/latency stats + boxplot
    monitoring/
      collect_net_stats.py            # /proc/net/dev + udp queue → CSV
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
