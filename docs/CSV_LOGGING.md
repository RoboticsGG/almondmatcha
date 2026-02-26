# Dual CSV Logging Architecture

## Overview

The rover implements a **dual-tier CSV logging system** to ensure data redundancy, leverage hardware strengths, and prepare for future database migration:

1. **Primary Logging (RPi)**: High-fidelity per-topic logging at native sensor rates
2. **Secondary Logging (Jetson)**: Aggregated telemetry logging on high-capacity storage

---

## Architecture

### Tier 1: RPi High-Fidelity Logging

**Node**: `node_mission_monitoring_rpi` (pkg_rover_monitoring)  
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
- `chassis_cmd.csv` (~50 Hz): Motor commands (left/right speed & direction)
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

**Node**: `node_rover_local_monitoring` (rover_monitor_pkg)  
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

## Rationale

### Why Dual Logging?

1. **Data Redundancy**: If RPi fails or SD card corrupts, Jetson has backup
2. **Storage Management**: RPi limited SD space, Jetson high capacity
3. **Future-Proofing**: Python on Jetson enables database migration without STM32/RPi changes
4. **Resolution Trade-offs**: RPi captures high-rate data, Jetson stores long-term aggregated data
5. **Domain Isolation**: Jetson on D4 doesn't add STM32 memory load (D5 only)

### Why Not Merge?

- **Different rates**: RPi needs high-rate (50 Hz chassis commands), Jetson needs long-term (5 Hz aggregated)
- **Different hardware**: RPi limited compute/storage, Jetson high capacity
- **Different purposes**: RPi for debugging/analysis, Jetson for mission logs/database
- **Language mismatch**: C++ on RPi (ROS2 native), Python on Jetson (DB-friendly)

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
Timestamp(us), Left_Speed, Right_Speed, Left_Dir, Right_Dir
1704373852123456, 50, 50, 1, 1
```

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

## Future Database Migration

### Jetson → PostgreSQL Schema (Example)

```sql
CREATE TABLE telemetry_log (
    id SERIAL PRIMARY KEY,
    timestamp TIMESTAMPTZ NOT NULL,
    mission_active BOOLEAN,
    distance_remaining_km REAL,
    rtk_latitude DOUBLE PRECISION,
    rtk_longitude DOUBLE PRECISION,
    rtk_altitude REAL,
    ...
);

CREATE INDEX idx_telemetry_timestamp ON telemetry_log(timestamp);
```

### Migration Path

1. Replace `csv.writer` with database connection in `node_rover_local_monitoring.py`
2. Write to PostgreSQL/SQLite instead of CSV files
3. Keep RPi CSV logging unchanged (high-rate debugging data)
4. Query Jetson database for mission analysis, long-term trends

**Benefit**: No changes to RPi/STM32 firmware, minimal changes to Jetson Python node

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

CSV logging is **automatic** when `node_mission_monitoring_rpi` launches. No additional configuration needed.

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
tmux send-keys -t jetson:4 "ros2 run rover_monitor_pkg node_rover_local_monitoring" C-m
```

### Base Station (ws_base)

No CSV logging on base station (display-only). Monitoring node subscribes to Domain 4 telemetry relay for real-time display.

---

## Data Analysis

### RPi Logs (High-Rate Analysis)

Use for:
- Motor command debugging (~50 Hz chassis_cmd.csv)
- IMU vibration analysis (~10 Hz chassis_imu.csv)
- RTK position accuracy (~10 Hz rtk_gnss.csv)
- Power consumption patterns (~4 Hz chassis_sensors.csv)

**Tools**: Python pandas, MATLAB, Excel

```python
import pandas as pd
df = pd.read_csv('run_001_20250104_143052/chassis_cmd.csv')
df['Timestamp(us)'] = pd.to_datetime(df['Timestamp(us)'], unit='us')
df.plot(x='Timestamp(us)', y=['Left_Speed', 'Right_Speed'])
```

### Jetson Logs (Mission Analysis)

Use for:
- Mission trajectory replay (5 Hz telemetry_unified.csv)
- Long-term trends (multi-mission database queries)
- Automated mission reports (SQL aggregation)

**Tools**: PostgreSQL, SQLite, Grafana, Python matplotlib

```python
df = pd.read_csv('run_001_20250104_143052/telemetry_unified.csv')
df['Timestamp(ISO)'] = pd.to_datetime(df['Timestamp(ISO)'])
df.plot(x='Timestamp(ISO)', y='Distance_Remaining_km')
```

---

## Disk Space Management

### RPi (Limited SD Card)

**Est. Size**:
- chassis_cmd.csv: ~50 Hz × 60 bytes = ~3 KB/s = ~11 MB/hour
- chassis_imu.csv: ~10 Hz × 80 bytes = ~800 B/s = ~2.8 MB/hour
- rtk_gnss.csv: ~10 Hz × 100 bytes = ~1 KB/s = ~3.6 MB/hour
- **Total**: ~20-30 MB/hour

**Recommendation**: Archive runs after each mission, keep last 3 runs on RPi

### Jetson (High Capacity)

**Est. Size**:
- telemetry_unified.csv: ~5 Hz × 300 bytes = ~1.5 KB/s = ~5.4 MB/hour

**Recommendation**: Keep all runs, rotate to PostgreSQL after 1 week

---

## Troubleshooting

### RPi CSV Files Not Created

1. Check directory permissions: `ls -la ~/almondmatcha/ws_rpi/runs/`
2. Check node logs: `ros2 node info /node_mission_monitoring_rpi`
3. Verify filesystem not full: `df -h`

### Jetson CSV Files Missing Data

1. Verify Domain 4 telemetry: `ROS_DOMAIN_ID=4 ros2 topic echo /tpc_telemetry_relay`
2. Check RPi is publishing to D4: RPi logs should show "Publishing to Domain 4"
3. Verify Jetson has sourced D4: `echo $ROS_DOMAIN_ID` should show `4`

### Different Run Numbers on RPi vs Jetson

**Expected**: Run numbers are independent (RPi boots separately from Jetson). To synchronize, manually rename directories or use network time sync for timestamp alignment.

---

## Summary

The dual CSV logging architecture provides:

✅ **Redundancy**: Data logged on both RPi and Jetson  
✅ **Resolution**: High-rate RPi logs for debugging, aggregated Jetson logs for missions  
✅ **Scalability**: Jetson Python backend ready for database migration  
✅ **Domain Isolation**: Jetson on D4 doesn't increase D5 participant count  
✅ **Storage Balance**: Limited RPi space for high-rate, unlimited Jetson for long-term  

**Key Design Principle**: Log locally at high rate (RPi), aggregate remotely for long-term storage (Jetson), display centrally without logging (Base).
