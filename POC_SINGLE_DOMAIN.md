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

### 2. Install LTTng + ros2_tracing on each SBC

Do this once on **both the RPi (192.168.1.1 / `curry`)** and the **Jetson (192.168.1.5 / `yupi`)**.  
SSH in and run each block in order, or use the helper script `ws_base/tools/tracing/setup_tracing.sh`.

#### 2a. LTTng kernel and userspace packages

```bash
sudo apt-get update
sudo apt-get install -y \
    lttng-tools \
    lttng-modules-dkms \
    liblttng-ust-dev \
    python3-lttng \
    babeltrace2
```

- `lttng-tools` — the `lttng` CLI used to create/start/stop trace sessions  
- `lttng-modules-dkms` — kernel probe modules (sched, irq, net); DKMS rebuilds them automatically on kernel upgrades  
- `liblttng-ust-dev` — userspace tracing library; required so that ROS2 nodes compiled with tracepoints can register them at runtime  
- `python3-lttng` — Python bindings for scripting  
- `babeltrace2` — CTF trace reader used by `analyze_latency.py`

#### 2b. ROS2 tracetools packages

```bash
sudo apt-get install -y \
    ros-humble-tracetools \
    ros-humble-tracetools-launch \
    ros-humble-ros2trace \
    ros-humble-tracetools-read \
    ros-humble-tracetools-analysis
```

> **Note:** `ros-humble-tracetools-analysis` may not be available in the standard apt mirror for arm64.  
> If the install fails for that package only, omit it — `analyze_latency.py` uses `babeltrace2` directly and does not depend on it.

#### 2c. Add your user to the `tracing` group

Without this, starting an LTTng session requires `sudo`.

```bash
sudo usermod -aG tracing $USER
```

**Log out and back in** (or run `newgrp tracing` in the current shell) for the group change to take effect.

#### 2d. Load kernel probe modules

The kernel modules are needed to capture scheduler and network events alongside the ROS2 userspace tracepoints.

```bash
sudo modprobe lttng-probe-sched
sudo modprobe lttng-probe-irq
# Persist across reboots:
echo -e "lttng-probe-sched\nlttng-probe-irq" | sudo tee /etc/modules-load.d/lttng.conf
```

#### 2e. Verify tracepoints are visible

Start a short-lived ROS2 node, then list registered userspace tracepoints:

```bash
source /opt/ros/humble/setup.bash
# In one terminal — run any ros2 node briefly so rclcpp registers its tracepoints
ros2 run demo_nodes_cpp talker &
# In another terminal
lttng list --userspace | grep ros2
# Expected output lines like:
#   ros2:rclcpp_publish (loglevel: TRACE_DEBUG_FUNCTION (12)) (type: tracepoint)
#   ros2:dispatch_subscription_callback (...)
#   ...
kill %1
```

If no `ros2:` lines appear, verify that `ros-humble-tracetools` was installed and that the node was built with `-DTRACETOOLS_TRACEPOINTS_ENABLED=ON` (the Humble apt packages have this enabled by default).

#### 2f. Quick smoke-test

```bash
lttng create test_session --output=/tmp/test_trace
lttng enable-event --userspace 'ros2:*'
lttng start test_session
# run a node for a few seconds
source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker &
sleep 5 && kill %1
lttng stop test_session && lttng destroy test_session
babeltrace2 /tmp/test_trace | head -20
# Should print CTF events including ros2:rclcpp_publish lines
```

After setup, **log out and back in** on each SBC so the `tracing` group is active.

### 3. Python dependencies on base PC

```bash
pip install babeltrace2 pandas numpy matplotlib
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

### Step 2 — Start LTTng tracing on RPi and Jetson

```bash
# From base PC — start tracing on RPi
TARGET_HOST=curry@192.168.1.1 TARGET_LABEL=rpi bash ws_base/tools/tracing/start_trace.sh

# Start tracing on Jetson
TARGET_HOST=yupi@192.168.1.5 TARGET_LABEL=jetson bash ws_base/tools/tracing/start_trace.sh
```

### Step 3 — Start network stats collection on each SBC

Open two extra terminals on the base PC and SSH in:

```bash
# Terminal A — RPi network stats
ssh curry@192.168.1.1 \
  "python3 ~/almondmatcha/ws_base/tools/monitoring/collect_net_stats.py \
     --iface eth0 --out /tmp/net_stats_rpi.csv"

# Terminal B — Jetson network stats
ssh yupi@192.168.1.5 \
  "python3 ~/almondmatcha/ws_base/tools/monitoring/collect_net_stats.py \
     --iface eth0 --out /tmp/net_stats_jetson.csv"
```

### Step 4 — Start STM32 memory collection on base PC

Both STM32 boards are connected to the base PC via USB serial (`/dev/ttyACM0` chassis, `/dev/ttyACM1` sensors — verify with `ls /dev/ttyACM*`).

```bash
python3 ws_base/tools/stm32_serial/collect_stm32_memory.py \
  --ports /dev/ttyACM0 /dev/ttyACM1 \
  --out /tmp/stm32_memory.csv
```

The collector filters only `{"type":"STM32_MEM",...}` JSON lines and ignores all other serial output. No suppression is needed in firmware.

### Step 5 — Let it run

Run for **at least 5 minutes** under representative load (send a mission command, drive the rover).

### Step 6 — Stop collection

```bash
# Stop LTTng and pull CTF data to base PC
TARGET_HOST=curry@192.168.1.1  TARGET_LABEL=rpi    bash ws_base/tools/tracing/stop_and_collect_trace.sh
TARGET_HOST=yupi@192.168.1.5   TARGET_LABEL=jetson bash ws_base/tools/tracing/stop_and_collect_trace.sh

# Stop net stats (Ctrl-C in each SSH terminal)
# Stop STM32 collector (Ctrl-C)

# Pull net stats CSVs back
scp curry@192.168.1.1:/tmp/net_stats_rpi.csv        ws_base/tools/monitoring/data/
scp yupi@192.168.1.5:/tmp/net_stats_jetson.csv      ws_base/tools/monitoring/data/
```

---

## Analyzing Results

### Latency and jitter (from LTTng traces)

```bash
# Analyze POC trace only
python3 ws_base/tools/tracing/analyze_latency.py \
  --trace-dir ws_base/tools/tracing/traces/rpi_<timestamp>

# Side-by-side comparison (requires a baseline trace collected on main branch)
python3 ws_base/tools/tracing/analyze_latency.py \
  --baseline ws_base/tools/tracing/traces/baseline_rpi_<ts> \
  --poc      ws_base/tools/tracing/traces/poc_rpi_<ts>

# Filter to specific topics only
python3 ws_base/tools/tracing/analyze_latency.py \
  --poc ... \
  --topics /tpc_chassis_imu /tpc_chassis_sensors /tpc_rover_ctrl_cmd
```

Output:
- Per-topic table: `mean`, `p50`, `p95`, `p99`, `max`, `std_dev` latency (ms)
- Per-topic jitter: `std_dev` of inter-publish interval (ms)
- `latency_poc.csv` (or `latency_baseline.csv`)
- `latency_boxplot.png` (when both `--baseline` and `--poc` are given)

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
df = pd.read_csv("/tmp/stm32_memory.csv")
for node, g in df.groupby("node"):
    print(f"\n=== {node} ===")
    print(g[["ts_ms","heap_used","heap_max","heap_free","alloc_fail"]].describe())
EOF
```

Compare `heap_used` and `heap_max` between baseline and POC runs. An increase indicates the extra 5 participant slots (or extra RTPS endpoint tracking for 15 vs 11 remote participants) consuming more SRAM.

---

## Collecting Baseline Traces for Comparison

To get a proper before/after comparison, repeat **Steps 2–6** on the `main` branch (with the original `launch_*_tmux.sh` scripts and unmodified firmware).

```bash
git checkout main
# flash original firmware
# launch with original scripts
TARGET_LABEL=baseline_rpi bash ws_base/tools/tracing/start_trace.sh ...
```

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
      setup_tracing.sh                # install LTTng + ros-humble-tracetools
      start_trace.sh                  # create LTTng session (ros2:* events)
      stop_and_collect_trace.sh       # stop + scp CTF data to base PC
      analyze_latency.py              # CTF → latency/jitter CSV + plot
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
