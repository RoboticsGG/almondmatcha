# Architecture

## Overview

Base station for rover mission control — dual-domain, not unified.

```
BASE STATION (D5) ─ command/action/service ─→ ROVER (D5)
BASE STATION (D4) ←─ telemetry relay ────────── ROVER (D5, via RPi relay)
```

`mission_command_node` participates directly in Domain 5 alongside the rest
of the rover's control nodes. `mission_monitoring_node_pc` participates only
in Domain 4 and never joins Domain 5 — it has no STM32 memory cost and no
direct visibility into rover-side D5 topics; it reads a single aggregated
relay topic instead.

## Nodes

### 1. mission_command_node (Domain 5)

Generates mission goals and speed limits.

**Config** (`config/params.yaml` / node parameter defaults):
```yaml
rover_spd: 15                        # Speed (0-100%)
des_lat: 8.007286                    # Latitude
des_long: 101.90203                  # Longitude
action_watchdog_timeout_sec: 20.0    # Cancel goal if no feedback this long
mission_retry_sec: 5.0               # Retry interval until the mission is established (code default)
```

**Calls:**
- `/des_data` — `DesData` action (navigation goal, to `gnss_mission_monitor_node` on the RPi)
- `/srv_spd_limit` — `SpdLimit` service (speed cap, to `chassis_controller_node` on the RPi)

**Flow:** load params → set speed limit → send navigation goal → retry on a
timer (`mission_retry_sec`) until accepted → monitor action feedback, cancel
if it stalls past `action_watchdog_timeout_sec`.

### 2. mission_monitoring_node_pc (Domain 4)

Displays rover telemetry as it arrives (event-driven, not polled).

**Subscribes:**
- `tpc_telemetry_relay` (`TelemetryRelay.msg`) — the only topic this node
  reads; aggregates GNSS, chassis command/sensor/IMU state, lane and
  destination fields, published by `mission_monitoring_node_rpi` on the RPi
  at 5 Hz.

## Stack

```
App:  mission_command_node (D5) | mission_monitoring_node_pc (D4)
ROS2: rclcpp | actions | services | topics
DDS:  FastDDS (RTPS)
Net:  Gigabit Ethernet (switch, 192.168.1.0/24)
```

## Domain Architecture

**Dual-domain, not unified:**
- **Domain 5** (control): ws_rpi (8 nodes), ws_base (`mission_command_node`),
  ws_jetson (`rover_kinematic_control`, D5-side context), STM32 chassis +
  sensors boards — 12 participants total.
- **Domain 4** (telemetry): `mission_monitoring_node_pc` (base) and
  `rover_local_monitoring_node` (Jetson) — both read-only subscribers to
  `tpc_telemetry_relay`, no STM32 participation.

**Communication:**
- **Actions:** `/des_data` (base → RPi navigation goals) — Domain 5
- **Services:** `/srv_spd_limit` (base → RPi speed limits) — Domain 5
- **Topics:** rover-side D5 telemetry topics are visible to other D5
  participants directly; the base station's monitoring node deliberately
  does not join D5 and instead reads the aggregated D4 relay
- **Discovery:** native DDS/RTPS within each domain; domains do not discover
  each other — `mission_monitoring_node_rpi` on the RPi is the only node
  that bridges D5 data onto D4, by explicitly re-publishing it

See [../../docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md) and
[../../docs/DOMAINS.md](../../docs/DOMAINS.md) for the full system-level
picture.

## Structure

```
ws_base/
├── src/
│   ├── common_ifaces/      # Interfaces (symlinked)
│   └── mission_control/    # mission_command_node, mission_monitoring_node_pc
└── docs/                   # Documentation
```

## Message Flow

```
mission_command_node ─/srv_spd_limit→ chassis_controller_node (RPi, D5)
                      ─/des_data────→ gnss_mission_monitor_node (RPi, D5)
                                              │
                                     RPi D5 topics aggregated by
                                     mission_monitoring_node_rpi
                                              │
                                    tpc_telemetry_relay (D4, 5 Hz)
                                              │
                                  mission_monitoring_node_pc (base, D4)
```
