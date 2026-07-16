# Dual CSV Logging Architecture

## Overview

The rover implements a **dual-tier CSV logging system** to ensure data redundancy, leverage hardware strengths, and prepare for future database migration:

1. **Primary Logging (RPi)**: High-fidelity per-topic logging at native sensor rates
2. **Secondary Logging (Jetson)**: Aggregated telemetry logging on high-capacity storage

---

## Architecture

### Tier 1: RPi High-Fidelity Logging

**Node**: `mission_monitoring_node_rpi` (rover_monitoring)  
**Domain**: 5 (rover control domain)  
**Location**: ws_rpi/runs/  
**Language**: C++

**Characteristics**:
- Subscribes to 10 Domain 5 topics directly
- Logs each topic at native sensor rate (event-driven)
- 6 separate CSV files for different data categories
- Full-resolution data capture (4-50 Hz depending on sensor)
- Minimal overhead (direct subscription → CSV write)

**CSV Files**:
- `rtk_gnss.csv` (~10 Hz): RTK position data from u-blox ZED-F9P
- `spresense_gnss.csv` (~10 Hz): GPS data from Sony Spresense
- `chassis_imu.csv` (~10 Hz): Accelerometer and gyroscope from STM32
- `chassis_sensors.csv` (~4 Hz): Encoders, voltage, current, power
- `chassis_cmd.csv` (~50 Hz): Motor commands (speed, steering direction, drive direction)
- `mission_state.csv` (event-driven): Mission status, destination, steering, lane detection

**Directory Structure**:
```
ws_rpi/runs/
├── run_001_20250104_143052/
│   ├── rtk_gnss.csv
│   ├── spresense_gnss.csv
│   ├── chassis_imu.csv
│   ├── chassis_sensors.csv
│   ├── chassis_cmd.csv
│   └── mission_state.csv
└── run_002_20250104_151823/
    └── ...
```

---

### Tier 2: Jetson Aggregated Logging

**Node**: `rover_local_monitoring_node` (rover_monitoring)  
**Domain**: 4 (base telemetry domain)  
**Location**: ws_jetson/runs/  
**Language**: Python

**Characteristics**:
- Subscribes to `/tpc_telemetry_relay` on Domain 4 (aggregated message)
- Logs at 5 Hz (telemetry relay rate)
- Single subscription (lower overhead than 10+ subscriptions)
- High-capacity Jetson storage (vs limited RPi SD card)
- Python-based for easy database migration

**CSV Files**:
- `telemetry_unified.csv`: All telemetry data in one file (5 Hz)
- `rtk_gnss.csv`: RTK position data (5 Hz, valid only)
- `spresense_gnss.csv`: Spresense GPS data (5 Hz, valid only)
- `chassis_data.csv`: Combined sensors, IMU, commands (5 Hz)
- `mission_state.csv`: Mission status, destination, steering, lane (5 Hz)

**Directory Structure**:
```
ws_jetson/runs/
├── run_001_20250104_143052/
│   ├── telemetry_unified.csv
│   ├── rtk_gnss.csv
│   ├── spresense_gnss.csv
│   ├── chassis_data.csv
│   └── mission_state.csv
└── run_002_20250104_151823/
    └── ...
```

---

## Comparison

| Aspect | RPi Logging | Jetson Logging |
|--------|-------------|----------------|
| **Data Rate** | 4-50 Hz (per-topic) | 5 Hz (aggregated) |
| **Resolution** | Full-fidelity | Down-sampled |
| **Subscriptions** | 10 topics (D5) | 1 topic (D4) |
| **CSV Files** | 6 per-topic files | 5 files (1 unified + 4 categorical) |
| **Storage** | Limited (SD card) | High-capacity (SSD/eMMC) |
| **Language** | C++ | Python |
| **DB Migration** | Difficult | Easy (SQLite/PostgreSQL) |
| **Purpose** | Primary high-res logs | Redundancy + future DB backend |
| **Domain Impact** | +0 (already in D5) | +0 (D4, not D5) |

---

## CSV Format Details

### RPi: chassis_sensors.csv
```
Timestamp(us), Encoder_Left, Encoder_Right, Voltage(V), Current(A), Power(W)
1704373852123456, 1234.5, 1230.2, 24.1, 2.3, 55.43
```

### RPi: chassis_imu.csv
```
Timestamp(us), Accel_X, Accel_Y, Accel_Z, Gyro_X, Gyro_Y, Gyro_Z
1704373852123456, 0.12, 0.05, 9.81, 0.01, -0.02, 0.00
```

### RPi: chassis_cmd.csv
```
Timestamp(us), Left_Speed, Right_Speed, Steer_Dir, Drive_Dir
1704373852123456, 50, 50, 2, 1
```

`Steer_Dir` = `fdr_msg`: 1=right, 2=straight, 3=left  
`Drive_Dir` = `bdr_msg`: 0=stop, 1=forward, 2=backward

### RPi: rtk_gnss.csv
```
Timestamp(us), Latitude, Longitude, Altitude, Fix_Quality, Centimeter_Error, Satellites
1704373852123456, 37.7749, -122.4194, 15.2, 4, 2, 12
```

### RPi: spresense_gnss.csv
```
Timestamp(us), Latitude, Longitude, Altitude, Satellites
1704373852123456, 37.7749, -122.4194, 15.3, 8
```

### RPi: mission_state.csv
```
Timestamp(us), Mission_Active, Distance_km, Dest_Lat, Dest_Lon, Steering_Cmd, Lane_Valid, Lane_Theta, Lane_B, Lane_Detected
1704373852123456, 1, 2.345, 37.7800, -122.4000, 10, 1, 0.12, 50.3, 1
```

### Jetson: telemetry_unified.csv
```
Timestamp(ISO), Mission_Active, Distance_km, Spresense_Valid, Spresense_Lat, ..., Lane_Detected
2025-01-04T14:30:52.123, 1, 2.345, 1, 37.7749, ..., 1
```

---

## Run Directory Numbering

Both RPi and Jetson use synchronized run numbering:

**Pattern**: `run_NNN_YYYYMMDD_HHMMSS/`

- `NNN`: Auto-incremented run number (001, 002, ...)
- `YYYYMMDD`: Date (20250104 = Jan 4, 2025)
- `HHMMSS`: Time (143052 = 2:30:52 PM)

**Detection**: `glob.glob("run_*")` finds existing runs, increments max number

**Example**:
- Run 1: `run_001_20250104_143052/`
- Run 2: `run_002_20250104_151823/`
- Run 3: `run_003_20250105_090015/`

---

## Launch Integration

### RPi (ws_rpi)

CSV logging is **automatic** when `mission_monitoring_node_rpi` launches. No additional configuration needed.

```bash
cd ~/almondmatcha/ws_rpi
./launch_rover_tmux.sh
```

### Jetson (ws_jetson)

Add to launch script (Domain 4 context):

```bash
# Terminal: Rover Monitoring (Domain 4)
tmux new-window -t jetson:4 -n "RoveMon"
tmux send-keys -t jetson:4 "source install/setup.bash" C-m
tmux send-keys -t jetson:4 "export ROS_DOMAIN_ID=4" C-m
tmux send-keys -t jetson:4 "ros2 run rover_monitoring rover_local_monitoring_node" C-m
```

### Base Station (ws_base)

No CSV logging on base station (display-only). Monitoring node subscribes to Domain 4 telemetry relay for real-time display.


