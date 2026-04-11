# POC — Multi-Domain (D4/D5/D6): Measurement Guide

**Branch:** `multi-domain`  
**Goal:** Measure the baseline tri-domain topology (D4 telemetry + D5 control + D6 vision) using the same experiment infrastructure, STM32 patches, and analysis tools developed on `single-domain`. Results are directly comparable with `single-domain` runs to evaluate the impact of domain consolidation.

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

**Output directory:** `ws_base/tools/poc_run/multi_domain/run_NNN/`

> Data is separated by branch: `poc_run/single_domain/` (from single-domain branch) and `poc_run/multi_domain/` (this branch) coexist on disk. Switching branches doesn't overwrite experiment data.

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

Same analysis tools as `single-domain`:

```bash
# Per-machine latency analysis
python3 ws_base/tools/tracing/analyze_latency.py \
    --poc ws_base/tools/poc_run/multi_domain/run_001/latency_rpi.csv

# Merge all CSVs into unified timeline
python3 ws_base/tools/poc_run/merge_run_csv.py \
    --run-dir ws_base/tools/poc_run/multi_domain/run_001

# Cross-configuration comparison (requires both directories populated)
python3 ws_base/tools/tracing/analyze_latency.py --merge \
    --run-dir ws_base/tools/poc_run/multi_domain/run_001 \
    --out-dir ws_base/tools/poc_run/multi_domain/run_001/
```

### Comparing Multi-Domain vs Single-Domain

After running experiments on both branches, compare side-by-side:

```bash
# Jitter comparison: multi-domain vs single-domain
python3 ws_base/tools/tracing/analyze_latency.py \
    --poc ws_base/tools/poc_run/multi_domain/run_001/latency_rpi.csv
python3 ws_base/tools/tracing/analyze_latency.py \
    --poc ws_base/tools/poc_run/single_domain/run_023/latency_rpi.csv

# STM32 memory comparison
head -5 ws_base/tools/poc_run/multi_domain/run_001/stm32_chassis.csv
head -5 ws_base/tools/poc_run/single_domain/run_023/stm32_chassis.csv
```

---

## Pre-flight Checklist

```bash
# 1. Verify STM32 firmware is flashed (same firmware as single-domain)
minicom -b 115200 -D /dev/ttyACM0  # Check build banner

# 2. Connectivity check (D5 STM32 topics — same for both branches)
bash ws_base/tools/check_connectivity.sh

# 3. Verify domain separation
export ROS_DOMAIN_ID=5 && ros2 node list  # Should see ~12 D5 nodes
export ROS_DOMAIN_ID=4 && ros2 node list  # Should see 3 D4 nodes
export ROS_DOMAIN_ID=6 && ros2 node list  # Should see 2-3 D6 nodes (Jetson only)
```
