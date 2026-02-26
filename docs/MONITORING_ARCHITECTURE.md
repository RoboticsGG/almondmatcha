# Rover Monitoring Architecture

## Overview

Dual-tier CSV logging: high-fidelity per-topic capture on RPi at native sensor rates, plus aggregated secondary logging on Jetson from the Domain 4 telemetry relay. Both tiers use the same `run_NNN_YYYYMMDD_HHMMSS/` directory convention.

## Data Flow

```
Domain 5 (RPi)
├── tpc_chassis_cmd (50 Hz)     ─┐
├── tpc_chassis_imu (10 Hz)     ─┤
├── tpc_chassis_sensors (4 Hz)  ─┤
├── tpc_gnss_spresense (10 Hz)  ─┤──> mission_monitoring_node_rpi
├── tpc_gnss_ublox (10 Hz)      ─┤       │
├── tpc_gnss_mission_active     ─┤       ├── CSV: 6 per-topic files (native rates)
├── tpc_gnss_mission_remain_dist─┤       └── Pub: /tpc_telemetry_relay (D4, 5 Hz)
├── tpc_rover_dest_coordinate   ─┤
├── tpc_rover_fmctl (50 Hz)     ─┤
└── tpc_rover_nav_lane (30 Hz)  ─┘

Domain 4 (Jetson)
└── /tpc_telemetry_relay (5 Hz) ──> node_rover_local_monitoring
                                        └── CSV: 5 aggregated files (5 Hz)
```

## Tier 1: RPi Primary Logger

**Node**: `mission_monitoring_node_rpi` (`pkg_rover_monitoring`, C++)  
**Domain**: 5 | **Location**: `ws_rpi/runs/run_NNN_YYYYMMDD_HHMMSS/`

| File | Rate | Columns |
|------|------|---------|
| `rtk_gnss.csv` | ~10 Hz | Timestamp_us, Latitude, Longitude, Altitude, Fix_Quality, Centimeter_Error, Satellites |
| `spresense_gnss.csv` | ~10 Hz | Timestamp_us, Latitude, Longitude, Altitude, Satellites |
| `chassis_imu.csv` | ~10 Hz | Timestamp_us, Accel_X, Accel_Y, Accel_Z, Gyro_X, Gyro_Y, Gyro_Z |
| `chassis_sensors.csv` | ~4 Hz | Timestamp_us, Encoder_Left, Encoder_Right, Voltage_V, Current_A, Power_W |
| `chassis_cmd.csv` | ~50 Hz | Timestamp_us, Left_Speed, Right_Speed, Left_Dir, Right_Dir |
| `mission_state.csv` | event | Timestamp_us, Mission_Active, Distance_km, Dest_Lat, Dest_Lon, Steering_Cmd, Lane_Valid, Lane_Theta, Lane_B, Lane_Detected |

Timestamps are microseconds since epoch (uint64). Writes are event-driven and flushed immediately.

## Tier 2: Jetson Secondary Logger

**Node**: `node_rover_local_monitoring` (`rover_monitor_pkg`, Python)  
**Domain**: 4 | **Location**: `ws_jetson/runs/run_NNN_YYYYMMDD_HHMMSS/`

| File | Rate | Content |
|------|------|---------|
| `telemetry_unified.csv` | 5 Hz | All TelemetryRelay fields in one row |
| `rtk_gnss.csv` | 5 Hz | RTK fields only (written when ublox_valid=True) |
| `spresense_gnss.csv` | 5 Hz | Spresense fields only (written when spresense_valid=True) |
| `chassis_data.csv` | 5 Hz | Sensors + IMU + command fields |
| `mission_state.csv` | 5 Hz | Mission, destination, steering, lane fields |

Timestamps are ISO-8601 strings. Python implementation allows straightforward database migration.

## Comparison

| Aspect | Tier 1 (RPi) | Tier 2 (Jetson) |
|--------|-------------|-----------------|
| Domain | D5 | D4 |
| Rate | 4\u201350 Hz per topic | 5 Hz (aggregated) |
| Files | 6 per-topic | 5 (1 unified + 4 categorical) |
| Timestamp | Microseconds (uint64) | ISO-8601 |
| Language | C++ | Python |
| DB migration | Difficult | Easy (SQLite/PostgreSQL) |
| Storage estimate | ~13 MB/hour | ~5 MB/hour |
| D5 participant cost | 0 (already in D5) | 0 (D4 only) |

## Build & Verify

```bash
# Build RPi (Tier 1 — merged into mission_monitoring_node_rpi)
cd ~/almondmatcha/ws_rpi && ./build.sh

# Build Jetson (Tier 2 — rover_monitor_pkg Python)
cd ~/almondmatcha/ws_jetson && colcon build --packages-select rover_monitor_pkg

# Verify RPi run directory
ls -lh ~/almondmatcha/ws_rpi/runs/run_*/

# Verify Jetson run directory
ls -lh ~/almondmatcha/ws_jetson/runs/run_*/

# Monitor live (RPi, high-rate)
tail -f ~/almondmatcha/ws_rpi/runs/run_*/chassis_cmd.csv
tail -f ~/almondmatcha/ws_rpi/runs/run_*/rtk_gnss.csv
```

## Post-Processing (Tier 1 RPi Logs)

```python
import pandas as pd
from pathlib import Path

run_dir = Path('ws_rpi/runs/run_001_20260226_143022')

rtk = pd.read_csv(run_dir / 'rtk_gnss.csv')
sensors = pd.read_csv(run_dir / 'chassis_sensors.csv')
cmds = pd.read_csv(run_dir / 'chassis_cmd.csv')

# Convert microsecond timestamps
for df in [rtk, sensors, cmds]:
    df['time'] = pd.to_datetime(df['Timestamp_us'], unit='us')

# Time-align within 100 ms tolerance
merged = pd.merge_asof(
    rtk.sort_values('time'),
    sensors.sort_values('time'),
    on='time',
    tolerance=pd.Timedelta('100ms')
)
```

## Storage Estimates

**Per hour (Tier 1, RPi):**
- chassis_cmd.csv: ~50 Hz × ~30 B = ~5.4 MB
- rtk_gnss.csv, spresense_gnss.csv, chassis_imu.csv: ~10 Hz × ~60 B each = ~6.5 MB combined
- chassis_sensors.csv: ~4 Hz × ~40 B = ~0.6 MB
- **Total Tier 1:** ~13 MB/hour (8 GB SD card: ~600 hours)

**Per hour (Tier 2, Jetson):**
- telemetry_unified.csv: ~5 Hz × ~300 B = ~5.4 MB
- **Total Tier 2:** ~8 MB/hour (128 GB eMMC: ~16,000 hours)

See [CSV_LOGGING.md](CSV_LOGGING.md) for full schema, rationale, and database migration path.
