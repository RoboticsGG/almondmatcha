# Rover Monitoring Architecture

## Overview
Event-driven per-topic CSV logging for full-rate data capture. Each sensor topic writes immediately to its own CSV file with microsecond timestamps.

## Logging Configuration

**Location**: `ws_rpi/runs/run_NNN_YYYYMMDD_HHMMSS/`  
**Format**: Separate CSV per topic in timestamped subdirectory  
**Single Logger**: `node_rover_monitoring` only

## Data Flow

```
Each Topic → Immediate CSV Write (event-driven, full rate)
├── rtk_gnss.csv (10 Hz)
├── spresense_gnss.csv (10 Hz)
├── chassis_imu.csv (10 Hz)
├── chassis_sensors.csv (4 Hz)
├── chassis_cmd.csv (50 Hz)
└── mission_state.csv (event-driven)
```

## CSV Strategy (Event-Driven Per-Topic)

### Why This Approach
- **Research-optimized**: No data loss at any rate
- **Full fidelity**: Every message logged at arrival time
- **Easy analysis**: One topic per file, simple to load
- **Perfect sync**: Microsecond timestamps across all files

## Node Responsibilities

| Node | Logging | Function |
|------|---------|----------|
| `node_chassis_sensors` | ❌ | Publish `/tpc_chassis_sensors` |
| `node_chassis_imu` | ❌ | Publish `/tpc_chassis_imu` |
| `node_gnss_spresense` | ❌ | Publish `/tpc_gnss_spresense` |
| `node_gnss_ublox` | ❌ | Publish `/tpc_gnss_ublox` |
| **`node_rover_monitoring`** | ✅ | Subscribe all topics, log CSV |
| `node_base_monitoring` | ❌ | Display only |

## CSV Files

### 1. rtk_gnss.csv (~10 Hz, ~36K rows/hour)
**Columns**: Timestamp_us, Date, Time, Latitude, Longitude, Altitude, Fix_Quality, Centimeter_Error, Satellites, SNR, Speed_ms

### 2. spresense_gnss.csv (~10 Hz, ~36K rows/hour)
**Columns**: Timestamp_us, Date, Time, Num_Satellites, Fix, Latitude, Longitude, Altitude

### 3. chassis_imu.csv (~10 Hz, ~36K rows/hour)
**Columns**: Timestamp_us, Accel_X, Accel_Y, Accel_Z, Gyro_X, Gyro_Y, Gyro_Z

### 4. chassis_sensors.csv (~4 Hz, ~14K rows/hour)
**Columns**: Timestamp_us, Motor_Left_Encoder, Motor_Right_Encoder, System_Current_A, System_Voltage_V

### 5. chassis_cmd.csv (~50 Hz, ~180K rows/hour)
**Columns**: Timestamp_us, FDR_Msg, RO_Ctrl_Deg, SPD_Msg, BDR_Msg

### 6. mission_state.csv (event-driven)
**Columns**: Timestamp_us, Mission_Active, Distance_Remaining_m, Dest_Latitude, Dest_Longitude

### Timestamp Synchronization
**Format**: Microseconds since epoch (uint64)  
**Example**: 1735224622500000 = 2025-12-26 14:30:22.500000  
**Python**: `pd.to_datetime(df['Timestamp_us'], unit='us')`  
**Merge**: `pd.merge_asof(df1, df2, on='time', tolerance=pd.Timedelta('100ms'))`

## Build & Launch

### Build
```bash
cd ~/almondmatcha/ws_rpi && ./build.sh
```

### Verify
```bash
# Check run directory structure
ls -lh ~/almondmatcha/ws_rpi/runs/run_*/

# Monitor live logging
tail -f ~/almondmatcha/ws_rpi/runs/run_*/rtk_gnss.csv
tail -f ~/almondmatcha/ws_rpi/runs/run_*/chassis_cmd.csv
```

### Post-Processing (Python)
```python
import pandas as pd
from pathlib import Path

# Load run directory
run_dir = Path('ws_rpi/runs/run_001_20251226_143022')

# Load all CSVs
rtk = pd.read_csv(run_dir / 'rtk_gnss.csv')
sensors = pd.read_csv(run_dir / 'chassis_sensors.csv')
commands = pd.read_csv(run_dir / 'chassis_cmd.csv')

# Convert timestamps
rtk['time'] = pd.to_datetime(rtk['Timestamp_us'], unit='us')
sensors['time'] = pd.to_datetime(sensors['Timestamp_us'], unit='us')
commands['time'] = pd.to_datetime(commands['Timestamp_us'], unit='us')

# Time-align (merge on nearest timestamp within tolerance)
merged = pd.merge_asof(
    rtk.sort_values('time'),
    sensors.sort_values('time'),
    on='time',
    tolerance=pd.Timedelta('100ms')  # 100ms max time difference
)
```

## Storage Estimates

**Per Hour**:
- RTK GNSS: ~36K rows × ~80 bytes = ~2.9 MB
- Spresense GNSS: ~36K rows × ~60 bytes = ~2.2 MB
- Chassis IMU: ~36K rows × ~50 bytes = ~1.8 MB
- Chassis Sensors: ~14K rows × ~40 bytes = ~560 KB
- Chassis Commands: ~180K rows × ~30 bytes = ~5.4 MB
- Mission State: Variable, ~1 KB

**Total**: ~13 MB/hour  
**1-hour mission**: 13 MB  
**8GB SD card**: ~600 hours of data

## Benefits
- **No data loss**: Full-rate capture at 50Hz for commands, 10Hz for sensors
- **Research-ready**: All raw data available for post-processing
- **Easy analysis**: One CSV per topic, load only what you need
- **Standard practice**: Similar to ROS bag file-per-topic approach
- **Perfect sync**: Microsecond timestamps enable sub-millisecond alignment
