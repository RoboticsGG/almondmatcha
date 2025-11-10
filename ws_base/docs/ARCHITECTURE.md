# Architecture

## Overview

Base station for rover mission control (Domain 5 - Unified Architecture).

```
BASE STATION (D5) ←─ Direct DDS ─→ ROVER (D5)
```

**No Bridge Needed:** All systems communicate directly on Domain 5. Actions and services work natively without relay nodes.

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

## Domain Architecture

**Unified Domain 5:**
- **ws_rpi**: GNSS, Chassis, Sensors (5 nodes)
- **ws_base**: Mission Command, Monitoring (2 nodes)
- **ws_jetson**: Vision Navigation (3 nodes)
- **STM32 Boards**: Chassis + GNSS (mROS2)

**Direct Communication:**
- **Actions:** `/des_data` (BASE→ROVER navigation goals)
- **Services:** `/srv_spd_limit` (BASE→ROVER speed limits)
- **Topics:** All telemetry topics visible to all nodes
- **Discovery:** Native DDS/RTPS - no relay needed

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
