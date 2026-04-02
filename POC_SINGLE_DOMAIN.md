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

> **Jetson Orin Nano:** `lttng-modules-dkms` will **fail** to build on the Tegra BSP kernel
> (`5.15.148-tegra`) because `linux-headers-*-tegra` are not available as standard apt packages.
> Use the helper script `ws_base/tools/tracing/setup_tracing.sh` which detects the Tegra kernel
> and skips `lttng-modules-dkms` automatically. Or run the block below — it is safe on both RPi and Jetson.

```bash
# Jetson-safe: use setup_tracing.sh (auto-detects Tegra kernel)
bash ~/almondmatcha/ws_base/tools/tracing/setup_tracing.sh
```

Or manually (RPi only — includes lttng-modules-dkms):

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
- `lttng-modules-dkms` — kernel probe modules (sched, irq, net); **not available on Jetson Tegra BSP kernel** (skip on Jetson)  
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

#### 2d. Load kernel probe modules (RPi only — skip on Jetson)

> **Jetson Orin Nano:** The Tegra BSP kernel (`5.15.148-tegra`) does **not** ship
> `lttng-probe-sched` or `lttng-probe-irq`. Running `modprobe` will fail with
> _"Module not found in directory /lib/modules/5.15.148-tegra"_.  
> **Skip this step on the Jetson.** Userspace tracepoints (`ros2:*`, `ros2_rmw:*`)
> are fully available and sufficient for latency/jitter measurement.

On the **RPi** (standard Ubuntu kernel):

```bash
sudo modprobe lttng-probe-sched
sudo modprobe lttng-probe-irq
# Persist across reboots:
echo -e "lttng-probe-sched\nlttng-probe-irq" | sudo tee /etc/modules-load.d/lttng.conf
```

#### 2e. Verify tracepoints are visible

> **Run on: RPi (`curry`) and Jetson (`orion`) — SSH in from base PC, or run directly on each SBC.**

Start a short-lived ROS2 node, then list registered userspace tracepoints.

> **Note:** `demo_nodes_cpp` is not installed on the SBCs. Use a node from the rover workspace instead.

```bash
cd ~/almondmatcha/ws_rpi        # on RPi
# cd ~/almondmatcha/ws_jetson   # on Jetson
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5

# Run any rover node briefly so rclcpp registers its tracepoints
ros2 run mission_control mission_command_node &
sleep 3

# In the same terminal — check registered tracepoints
lttng list --userspace | grep ros2
# Expected output lines like:
#   ros2:rclcpp_publish (loglevel: TRACE_DEBUG_FUNCTION (12)) (type: tracepoint)
#   ros2:dispatch_subscription_callback (...)
#   ...

kill %1
```

If no `ros2:` lines appear, verify that `ros-humble-tracetools` was installed and that the workspace was built with tracepoints enabled (the default for Humble apt packages).

#### 2f. Quick smoke-test

> **Run on: RPi (`curry`) and Jetson (`orion`) — SSH in from base PC, or run directly on each SBC.**

```bash
cd ~/almondmatcha/ws_rpi        # on RPi
# cd ~/almondmatcha/ws_jetson   # on Jetson
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=5

lttng create test_session --output=~/ros2_traces/test_trace
lttng enable-event --userspace 'ros2:*'
lttng start test_session

# Run a rover node for a few seconds
ros2 run mission_control mission_command_node &
sleep 5 && kill %1

lttng stop test_session && lttng destroy test_session
babeltrace2 ~/ros2_traces/test_trace | head -20
# Should print CTF events including ros2:rclcpp_publish lines
```

After setup, **log out and back in** on each SBC so the `tracing` group is active.

### 3. Python dependencies on base PC

> **Note:** `babeltrace2` is **not** a pip package. Install it via apt, then install the rest with pip3.

```bash
sudo apt-get install -y babeltrace2 python3-bt2
pip3 install pandas numpy matplotlib
```

Verify:
```bash
python3 -c "import bt2; print(bt2.__version__)"
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
# Stop LTTng and pull CTF data to base PC
TARGET_HOST=curry@192.168.1.1  TARGET_LABEL=rpi    bash ws_base/tools/tracing/stop_and_collect_trace.sh
TARGET_HOST=yupi@192.168.1.5   TARGET_LABEL=jetson bash ws_base/tools/tracing/stop_and_collect_trace.sh

# Stop net stats (Ctrl-C in each SSH terminal)
# Stop STM32 collector (Ctrl-C)

# Pull net stats CSVs back
scp curry@192.168.1.1:~/ros2_traces/net_stats_rpi.csv        ws_base/tools/monitoring/data/poc_net_stats_rpi.csv
scp yupi@192.168.1.5:~/ros2_traces/net_stats_jetson.csv      ws_base/tools/monitoring/data/poc_net_stats_jetson.csv
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
import os
df = pd.read_csv(os.path.expanduser("~/ros2_traces/stm32_memory_poc.csv"))
for node, g in df.groupby("node"):
    print(f"\n=== {node} ===")
    print(g[["ts_ms","heap_used","heap_max","heap_free","alloc_fail"]].describe())
EOF
```

Compare `heap_used` and `heap_max` between baseline and POC runs. An increase indicates the extra 5 participant slots (or extra RTPS endpoint tracking for 15 vs 11 remote participants) consuming more SRAM.

---

## Collecting Baseline Traces for Comparison

To get a proper before/after comparison, repeat **Steps 2–6** on the `multi-domain` branch, which runs the original multi-domain architecture with the same measurement tooling.

```bash
git checkout multi-domain
# Rebuild and re-flash both STM32 boards from multi-domain firmware
# Launch with multi-domain scripts
TARGET_LABEL=rpi bash ws_base/tools/tracing/start_trace.sh
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
