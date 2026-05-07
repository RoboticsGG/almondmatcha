# POC — Multi-Domain (D4/D5/D6): Measurement Guide

**Branch:** `multi-domain`  
**Goal:** Measure the baseline tri-domain topology (D4 telemetry + D5 control + D6 vision) using the same experiment infrastructure, STM32 patches, and analysis tools developed on `single-domain`. Results are directly comparable with `single-domain` runs to evaluate the impact of domain consolidation.

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

> For this POC the primary metric is **relative change** between branches, not absolute values. A consistent offset on one host is acceptable — it will be present in both single-domain and multi-domain runs equally.

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

> **Fallback:** If RPi loses internet, `local stratum 4` keeps it serving its own clock to LAN clients rather than stopping.

```bash
# Connectivity + ROS2 environment check
bash ws_base/tools/check_connectivity.sh

# Verify domain separation (optional — run after node launch)
export ROS_DOMAIN_ID=5 && ros2 node list  # ~12 D5 nodes (control + STM32)
export ROS_DOMAIN_ID=4 && ros2 node list  # 3 D4 nodes (monitoring)
export ROS_DOMAIN_ID=6 && ros2 node list  # 2 D6 nodes (Jetson vision — shared memory)
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

The script starts all collectors across D4/D5/D6, waits, stops and pulls all CSVs, then calls `post_run.sh` automatically.

### Step 4 — Review results (auto-generated at end of run)

All output files land in `ws_base/runs/multi_domain/run_NNN/`:

| File | Description |
|---|---|
| `merged_all.csv` | 1-second NTP-aligned time-bucketed wide CSV (all sources) |
| `merged_flat.csv` | Un-bucketed raw event log (full resolution, all sources) |
| `latency_summary.csv` | Per-topic jitter/latency stats table (mean, p50, p95, p99) |
| `unified_timeline.png` | Multi-panel chart: jitter per domain, STM32 heap, network BW |
| `latency_rpi.csv`, `latency_jetson.csv`, `latency_jetson_d6.csv` | Raw per-SBC / per-domain collector outputs |
| `topic_bw.csv`, `topic_bw_d4.csv`, `topic_bw_d6.csv` | Per-domain bandwidth (D5/D4/D6) |

### Step 5 — Re-run post-processing or compare with single-domain

```bash
# Re-generate all result files from a completed run:
bash ws_base/tools/post_run.sh ws_base/runs/multi_domain/run_001

# Side-by-side comparison with single-domain (both run dirs must exist):
bash ws_base/tools/post_run.sh ws_base/runs/multi_domain/run_001 \
    --compare ws_base/runs/single_domain/run_001
# Produces: run_001/jitter_boxplot.png
```

---

## Topology Comparison

| | This branch (multi-domain) | Single-domain POC |
|---|---|---|
| Domain 4 | monitoring (Base + Jetson + RPi pub) — 3 nodes | ✗ removed (all on D5) |
| Domain 5 | ~12 participants (control + STM32) | **16 participants** (all nodes) |
| Domain 6 | vision (Jetson localhost, shared memory) — 2 nodes | ✗ removed (all on D5) |
| Camera/lane Images@30fps on network | **no** (D6 shared memory) | yes (D5 stress factor) |
| STM32 sees D5 participants | ~12 | ~16 |
| STM32 SPDP_MAX | 30 (18 margin) | 30 (14 margin) |

---

## Domain Architecture

| Domain | Purpose | Machines | DDS Transport | Nodes |
|--------|---------|----------|--------------|-------|
| **D4** | Telemetry relay | RPi (pub) → Base + Jetson (sub) | UDP multicast | 3 (RPi internal D4 pub, Base monitoring, Jetson local monitoring) |
| **D5** | Control + STM32 | RPi + Base + STM32×2 + Jetson (kinematic pub) | UDP multicast + FastDDS XML | ~12 |
| **D6** | Vision processing | Jetson only | Shared memory (localhost) | 2 (camera, lane detection) |

### Cross-Domain Bridges (no separate processes)

| Bridge | Node | Mechanism |
|--------|------|-----------|
| D5 → D4 | `mission_monitoring_node_rpi` | Dual rclcpp context — subscribes all D5 topics, publishes `tpc_telemetry_relay` on D4 at 5 Hz |
| D6 → D5 | `rover_kinematic_control` | Dual rclpy context — subscribes `tpc_rover_nav_lane` on D6, publishes `tpc_rover_ctrl_cmd` on D5 |

### Node Inventory

| # | Node | Machine | Domain | FastDDS XML |
|---|------|---------|--------|-------------|
| 1 | `gnss_spresense_node` | RPi | D5 | `fastdds_rover.xml` |
| 2 | `gnss_ublox_node` | RPi | D5 | `fastdds_rover.xml` |
| 3 | `gnss_mission_monitor_node` | RPi | D5 | `fastdds_rover.xml` |
| 4 | `chassis_controller_node` | RPi | D5 | `fastdds_rover.xml` |
| 5 | `chassis_imu_node` | RPi | D5 | `fastdds_rover.xml` |
| 6 | `chassis_sensors_node` | RPi | D5 | `fastdds_rover.xml` |
| 7 | `mission_monitoring_node_rpi` | RPi | D5→D4 | `fastdds_rover.xml` |
| 8 | `rover_monitoring_node` | RPi | D5 | `fastdds_rover.xml` |
| 9 | `camera_stream_node` | Jetson | D6 | none (shared memory) |

> **Camera configuration:** `camera_stream_node` uses the `device_serial` parameter (default `806312060441`, D415) and supports `fallback_video` for automatic fallback to a local video file when the camera is unavailable.

| 10 | `lane_detection_node` | Jetson | D6 | none (shared memory) |
| 11 | `rover_kinematic_control` | Jetson | D6→D5 | none (default DDS) |
| 12 | `rover_local_monitoring_node` | Jetson | D4 | none |
| 13 | `mission_command_node` | Base | D5 | `fastdds_base.xml` |
| 14 | `mission_monitoring_node_pc` | Base | D4 | none |
| 15 | STM32 chassis | 192.168.1.2 | D5 | embeddedRTPS |
| 16 | STM32 sensors | 192.168.1.6 | D5 | embeddedRTPS |

---

## STM32 Firmware

**Identical to `single-domain` branch** — same patches (001–005), same `config.h`, same `mbed_app.json` (`PBUF_POOL_SIZE=32`), same build process. The STM32 boards run on D5 in both configurations. Domain separation is handled entirely by Linux-side launch scripts.

---

## What is Measured

Same five collectors as `single-domain` — see `POC_SINGLE_DOMAIN.md` section "What is Measured" for full details:

1. **ROS2 message jitter/latency** — `collect_latency.py` (on RPi + Jetson)
2. **Network interface bandwidth** — `collect_net_stats.py` (on RPi + Jetson)
3. **Per-topic bandwidth** — `collect_topic_bw.py` (on Base PC, D5 scope)
4. **STM32 heap/stack** — `collect_stm32_memory.py` (serial, on Base PC)
5. **Merged timeline** — `merge_run_csv.py` (time-bucketed union)

### Key Measurement Differences vs Single-Domain

| Metric | Multi-Domain (this) | Single-Domain |
|--------|-------------------|----------------|
| D5 network bandwidth | Lower — camera images stay on D6 shared memory | Higher — camera images on D5 network |
| STM32 SPDP/SEDP load | Lower — fewer D5 participants (~12 vs ~16) | Higher — all participants on D5 |
| `collect_topic_bw.py` D5 scope | Does NOT see D6 vision topics | Sees all topics including camera |
| RPi `max_rx_queue` | Lower — no camera UDP traffic | Higher — camera UDP traffic on D5 |

---

## Prerequisites

### Why a rebuild is required

The `multi-domain` branch was reset from `single-domain` and carries the same C++ source code — no application logic changed. However, each machine must have compiled workspace artifacts in its local `build/` and `install/` directories before the launch scripts will work. If you are running this branch on a machine for the first time (fresh clone, or previously only built `single-domain`), the `install/` tree may be missing or stale.

> **Note:** Because no C++ source was modified in this branch, an **incremental build** (`build_inc.sh`) is sufficient on machines where the workspace was already built on `single-domain`. A clean build is only needed on a fresh checkout.

### Build order

The workspaces have a dependency chain. Always follow this order:

```
common_ifaces  →  ws_rpi (RPi)
                   ws_jetson (Jetson)
                   ws_base (Base)
```

`ws_rpi` builds its own interface packages internally and does not depend on `common_ifaces`. `ws_jetson` and `ws_base` do depend on it.

### Per-machine build commands

**Base PC** (run once, or after any source change):
```bash
cd ~/almondmatcha

# 1. Build shared interface packages (msgs, actions, services)
cd common_ifaces
source /opt/ros/humble/setup.bash
colcon build --symlink-install
cd ..

# 2. Incremental build of ws_base
bash ws_base/build_inc.sh
```

**RPi** (run on the rover):
```bash
cd ~/almondmatcha

# Builds interfaces + application packages in one step
bash ws_rpi/build_inc.sh
```

**Jetson**:
```bash
cd ~/almondmatcha

# 1. Build common_ifaces (same steps as Base PC above)
cd common_ifaces
source /opt/ros/humble/setup.bash
colcon build --symlink-install
cd ..

# 2. Incremental build of ws_jetson
bash ws_jetson/build_inc.sh
```

### Verifying the build

After building, confirm the key executables are discoverable before launching:

```bash
# On Base PC
source ws_base/install/setup.bash
ros2 pkg executables mission_control  # must list mission_command_node, mission_monitoring_node_pc

# On RPi
source ws_rpi/install/setup.bash
ros2 pkg executables rover_bringup    # must list mission_monitoring_node_rpi, rover_monitoring_node

# On Jetson
source ws_jetson/install/setup.bash
ros2 pkg executables rover_vision     # must list camera_stream_node, lane_detection_node
ros2 pkg executables rover_navigation # must list rover_kinematic_control
```

### Switching to/from the single-domain branch

**No rebuild is required when switching branches.** Both `multi-domain` and `single-domain` carry identical C++ source code — the only files that differ are launch scripts, docs, and `launch_poc_experiment.sh`. The compiled `build/` and `install/` trees are valid on both branches.

```bash
# On the base PC (and repeat on RPi / Jetson via SSH)
git stash           # stash any local edits if needed
git checkout single-domain
```

After switching:
- **No rebuild needed** on any machine
- **poc_run data is preserved** — `ws_base/runs/single_domain/` and `ws_base/runs/multi_domain/` are gitignored subdirectories and are unaffected by `git checkout`
- `launch_poc_experiment.sh` on the single-domain branch references `*_single_domain.sh` scripts and writes to `runs/single_domain/`

To switch back:
```bash
git checkout multi-domain
# No rebuild needed — launch scripts and poc_run path revert automatically
```

> **Important — keep the working trees in sync:** Each machine (RPi, Jetson) keeps its own
> git working tree. When you switch branches on the base PC, SSH into each SBC and run
> `git checkout <branch>` there as well. Otherwise the old launch scripts will be used.

---

## Running the Experiment

### Option A — Automated (recommended): `launch_poc_experiment.sh`

```bash
cd ~/almondmatcha

# Default: 5-minute run
bash ws_base/launch_poc_experiment.sh

# Override duration
bash ws_base/launch_poc_experiment.sh --duration 600

# If ttyACM ports are reversed
bash ws_base/launch_poc_experiment.sh --chassis /dev/ttyACM0 --sensors /dev/ttyACM1

# Collectors only — skip node launch
bash ws_base/launch_poc_experiment.sh --skip-launch
```

**Output directory:** `ws_base/runs/multi_domain/run_NNN/`

> Data is separated by branch: `runs/single_domain/` (single-domain branch) and `runs/multi_domain/` (this branch) coexist on disk. Switching branches does not overwrite experiment data.

### Option B — Manual

1. Start STM32 memory collector
2. Launch RPi: `bash ws_rpi/launch_rover_multi_domain.sh`
3. Launch Jetson: `bash ws_jetson/launch_jetson_multi_domain.sh`
4. Launch Base: `bash ws_base/launch_base_multi_domain.sh`
5. Start latency/net-stats/topic-BW collectors
6. Wait for run duration
7. Stop and collect CSVs

---

## Analysis

> **Post-processing is automatic.** `launch_poc_experiment.sh` calls `post_run.sh` at the end of every run. To regenerate or compare:
> ```bash
> bash ws_base/tools/post_run.sh ws_base/runs/multi_domain/run_001
> bash ws_base/tools/post_run.sh ws_base/runs/multi_domain/run_001 --compare ws_base/runs/single_domain/run_001
> ```

```bash
# Per-machine latency analysis (prints per-topic stats to stdout)
python3 ws_base/tools/tracing/analyze_latency.py \
    --poc ws_base/runs/multi_domain/run_001/latency_rpi.csv

# Merge all CSVs into 1s-bucketed wide CSV (merged_all.csv)
python3 ws_base/tools/merge_run_csv.py \
    --run-dir ws_base/runs/multi_domain/run_001

# Unified timeline chart (auto-discovers D4/D5/D6 files from run dir)
python3 ws_base/tools/tracing/analyze_latency.py --merge \
    --run-dir ws_base/runs/multi_domain/run_001 \
    --out-dir ws_base/runs/multi_domain/run_001/
```

### Comparing Multi-Domain vs Single-Domain

After running experiments on both branches, compare side-by-side:

```bash
# Automated comparison (jitter boxplot)
bash ws_base/tools/post_run.sh ws_base/runs/multi_domain/run_001 \
    --compare ws_base/runs/single_domain/run_001

# Quick CSV spot-check
head -5 ws_base/runs/multi_domain/run_001/stm32_chassis.csv
head -5 ws_base/runs/single_domain/run_001/stm32_chassis.csv
```

---

## Pre-flight Checklist

```bash
# 1. Verify STM32 firmware is flashed (same firmware as single-domain)
minicom -b 115200 -D /dev/ttyACM0  # Check build banner

# 2. Connectivity check (D5 STM32 topics)
bash ws_base/tools/check_connectivity.sh

# 3. Verify domain separation (after nodes are running)
export ROS_DOMAIN_ID=5 && ros2 node list  # Should see ~12 D5 nodes
export ROS_DOMAIN_ID=4 && ros2 node list  # Should see 3 D4 nodes
export ROS_DOMAIN_ID=6 && ros2 node list  # Should see 2-3 D6 nodes (Jetson only)
```
