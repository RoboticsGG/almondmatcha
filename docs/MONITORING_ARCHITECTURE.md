# Rover Monitoring Architecture

## Overview
Centralized CSV logging via single node (`node_rover_monitoring`) aggregating all sensor data. Other nodes publish ROS2 topics only.

## Logging Configuration

**Location**: `ws_rpi/runs/`  
**Format**: `run_NNN_YYYYMMDD_HHMMSS.csv`  
**Single Logger**: `node_rover_monitoring` only

## Data Flow

```
Sensors → ROS2 Topics → node_rover_monitoring → CSV + RoverStatus topic
```

## Node Responsibilities

| Node | Logging | Function |
|------|---------|----------|
| `node_chassis_sensors` | ❌ | Publish `/tpc_chassis_sensors` |
| `node_chassis_imu` | ❌ | Publish `/tpc_chassis_imu` |
| `node_gnss_spresense` | ❌ | Publish `/tpc_gnss_spresense` |
| `node_gnss_ublox` | ❌ | Publish `/tpc_gnss_ublox` |
| **`node_rover_monitoring`** | ✅ | Subscribe all topics, log CSV |
| `node_base_monitoring` | ❌ | Display only |

## CSV Content

**Columns**: Timestamp, RTK GNSS (10 fields), Spresense GNSS (7 fields), Chassis Sensors (4 fields), IMU (6 fields), Mission/Control (8 fields)

**Update Rate**: 1 Hz

## Build & Launch

### Build
```bash
cd ~/almondmatcha/ws_rpi && ./build_clean.sh
```

### Verify
```bash
ls -lh ~/almondmatcha/ws_rpi/runs/
tail -f ~/almondmatcha/ws_rpi/runs/run_*.csv
```

## Benefits
- Single source of truth
- Synchronized timestamps
- No duplicate logging
- Auto-incrementing run numbers
- Portable relative paths
