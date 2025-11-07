# Architecture

## Overview

Base station for rover mission control (Domain 2).

```
ROVER (D5) ←─ pkg_base_bridge ─→ BASE STATION (D2)
```

**Bridge:** Requires `pkg_base_bridge` running on ws_rpi to relay topics between Domain 5 (rover internal) and Domain 2 (base station).

## Nodes

### 1. mission_command_node
Generates mission goals and speed limits.

**Config** (`config/params.yaml`):
```yaml
rover_spd: 15      # Speed (0-100%)
des_lat: 500.0     # Latitude
des_long: 500.0    # Longitude
```

**Publishes:**
- `/des_data` - Navigation action
- `/spd_limit` - Speed service

**Flow:** Load params → Send speed → Send goal → Monitor feedback

### 2. mission_monitoring_node
Displays rover telemetry @ 1 Hz.

**Subscribes:**
- `tpc_gnss_spresense` - Position
- `tpc_gnss_mission_active` - Status
- `tpc_gnss_mission_remain_dist` - Distance
- `tpc_chassis_cmd` - Commands
- `tpc_rover_dest_coordinate` - Target

## Stack

```
App:  mission_command | mission_monitoring
ROS2: rclcpp | actions | services | topics
DDS:  Cyclone DDS (RTPS)
Net:  Ethernet | WiFi
```

## Domains

- **D5**: ws_rpi (Rover internal - all sensors & control)
- **D2**: ws_base (Base station - this workspace)
- **D7**: ws_jetson (Vision system)

**Bridge Communication:**
- `pkg_base_bridge` on ws_rpi relays topics between D5 ↔ D2
- **D5→D2 (Telemetry):** tpc_chassis_imu, tpc_chassis_sensors, tpc_gnss_spresense, tpc_chassis_cmd, tpc_gnss_mission_active, tpc_gnss_mission_remain_dist
- **D2→D5 (Commands):** tpc_rover_dest_coordinate

## Structure

```
ws_base/
├── src/
│   ├── common_ifaces/      # Interfaces
│   └── mission_control/    # Nodes
└── docs/                   # Documentation
```

## Message Flow

```
command_node → /spd_limit → ROVER
             → /des_data  →   ↓
                         tpc_* topics
                              ↓
                     monitoring_node
```
