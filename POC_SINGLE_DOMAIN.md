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

> **Run every step directly on the SBC** (SSH in from base PC or run locally).  
> Do this once on **both the RPi (`curry`)** and the **Jetson (`orion`)**.

#### 2a. Install all packages

The helper script auto-detects the Tegra BSP kernel on Jetson and skips `lttng-modules-dkms` (which cannot build against it). It installs everything in one shot:

```bash
cd ~/almondmatcha && git pull
bash ws_base/tools/tracing/setup_tracing.sh
```

This installs: `lttng-tools`, `liblttng-ust-dev`, `python3-lttng`, `babeltrace2`, `lttng-modules-dkms` (**RPi only**), and all `ros-humble-tracetools*` packages.

#### 2b. Add user to the `tracing` group

```bash
sudo usermod -aG tracing $USER
newgrp tracing   # apply immediately in current shell
```

> **Also log out and back in** so the group change is permanent across all shells.

#### 2c. Load kernel probe modules (RPi only — skip on Jetson)

> **Jetson Orin Nano:** The Tegra BSP kernel (`5.15.148-tegra`) does **not** ship
> `lttng-probe-sched` or `lttng-probe-irq`. **Skip this entire step on the Jetson.**  
> Userspace tracepoints (`ros2:*`) are sufficient for latency/jitter measurement.

On the **RPi** only:

```bash
sudo modprobe lttng-probe-sched
sudo modprobe lttng-probe-irq
# Persist across reboots:
echo -e "lttng-probe-sched\nlttng-probe-irq" | sudo tee /etc/modules-load.d/lttng.conf
```

#### 2d. Verify installation (no running nodes needed)

```bash
# Check lttng is installed
lttng --version

# Check you are in the tracing group (must show 'tracing')
groups

# Check ros2 tracetools packages are installed
source /opt/ros/humble/setup.bash
dpkg -l | grep ros-humble-tracetools | awk '{print $2, $3}'
# Expected: at least ros-humble-tracetools and ros-humble-ros2trace

# Check liblttng-ust is present
ldconfig -p | grep lttng-ust
# Expected: liblttng-ust.so.1 ...
```

#### 2e. Start session daemon and verify it accepts commands

```bash
lttng-sessiond --daemonize; sleep 1
lttng list
# Expected: "Currently no available recording session"
# This is CORRECT — it means the daemon is running and responding.
# "No session" just means no trace session has been created yet.
```

#### 2f. Smoke-test with a C++ rover node (RPi only — requires workspace built)

> **Why C++ only?**  
> The `ros2:*` LTTng tracepoints are instrumented in `rclcpp` (the C++ client library).  
> Python nodes (`rclpy`) do **not** emit `ros2:*` events. Since all Jetson nodes in this  
> project are Python, **the smoke-test is RPi only**. The tracing infrastructure on Jetson  
> is still valid — it will capture `ros2_rmw:*` events from CycloneDDS at the middleware layer.

> **If the RPi workspace is not yet built**, skip this step. The tracepoints will be  
> confirmed during Step 2 of the experiment when the rover nodes are running.

On **RPi only** (workspace must be built):

```bash
cd ~/almondmatcha/ws_rpi
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5
mkdir -p ~/ros2_traces

# Create and start a trace session
lttng create test_session --output=~/ros2_traces/test_trace
lttng enable-event --userspace 'ros2:*'
lttng start test_session

# Run a C++ rover node briefly
ros2 run rover_monitoring rover_monitoring_node &
NODE_PID=$!
sleep 5 && kill $NODE_PID

# Stop and verify
lttng stop test_session && lttng destroy test_session
babeltrace2 ~/ros2_traces/test_trace | head -30
# Should print CTF event lines including: ros2:rclcpp_publish, ros2:rcl_timer_init, etc.
```

**On Jetson — skip 2f and just verify the daemon responds:**

```bash
# All ws_jetson nodes are Python (rclpy) — no ros2:rclcpp_* tracepoints.
# Just confirm the infrastructure is ready:
lttng-sessiond --daemonize 2>/dev/null; sleep 1
lttng list
# Expected: "Currently no available recording session" — daemon is running
```

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
