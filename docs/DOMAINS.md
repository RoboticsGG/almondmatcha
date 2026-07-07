# ROS2 Domain Architecture

> **Branch: `single-domain`** — all workspaces on `ROS_DOMAIN_ID=5` (D4+D5+D6 collapsed). POC to measure the impact of domain consolidation. See the `multi-domain` branch for the baseline tri-domain architecture.

## Domain Assignment

| Domain | Purpose | Scope | Participants |
|--------|---------|-------|-------------|
| **5** | All communication | All systems via Ethernet | 16 nodes: RPi (8), Jetson (4), Base (2), STM32 chassis + sensors |

## Architecture

```mermaid
flowchart TB
    subgraph D5["Domain 5 — All nodes (all systems via Ethernet)"]
        CAM["camera_stream_node (Jetson)"]
        LANE["lane_detection_node (Jetson)"]
        CTRL["rover_kinematic_control (Jetson)\n→ tpc_rover_ctrl_cmd @ 50 Hz"]
        CC["chassis_controller_node (RPi)"]
        STM32C["chassis_controller (STM32 .2)"]
        STM32S["sensors_node (STM32 .6)"]
        GNSS["GNSS nodes (RPi): spresense + ublox + mission"]
        MON["mission_monitoring_node_rpi (RPi)"]
        LOG["rover_monitoring_node (RPi)"]
        CMD["mission_command_node (Base)"]
        MONPC["mission_monitoring_node_pc (Base)"]
        MONJ["rover_local_monitoring_node (Jetson)"]
    end

    CAM -->|tpc_rover_d415_rgb| LANE -->|tpc_rover_nav_lane| CTRL
    CTRL --> CC --> STM32C
    GNSS --> MON
    STM32C --> MON
    STM32S --> MON
    CTRL --> MON
    MON -->|tpc_telemetry_relay| MONPC
    MON -->|tpc_telemetry_relay| MONJ
```

## Design Rationale

**Measurement goal:** Collapse D4+D5+D6 → D5 to quantify:
- STM32 SRAM impact (16 visible participants vs 11 in multi-domain)
- Message latency/jitter with camera images sharing the control network
- Socket buffer pressure on RPi and Jetson from high-bandwidth vision topics

**Key differences from `multi-domain`:**
- `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS=30` (16 nodes + daemons + CLI + margin)
- Camera images (`tpc_rover_d415_rgb`) traverse the network — no shared-memory isolation
- All monitoring nodes on D5 (no D4 relay path)
- `rover_kinematic_control` — single D5 context (no dual-context bridge)

---

**See Also:** [ARCHITECTURE.md](ARCHITECTURE.md) · [TOPICS.md](TOPICS.md) · [CSV_LOGGING.md](CSV_LOGGING.md)
