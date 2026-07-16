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

## CSV Field Reference

### RPi: rtk_gnss.csv
Source: `tpc_gnss_ublox` topic (~10 Hz) — u-blox ZED-F9P RTK receiver

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) — base PC wall clock |
| `Latitude` | float64 | °  | WGS84 latitude, positive = North |
| `Longitude` | float64 | °  | WGS84 longitude, positive = East |
| `Altitude` | float64 | m  | Height above ellipsoid |
| `Fix_Quality` | string | — | `"No Fix"`, `"GPS"`, `"DGPS"`, `"RTK Float"`, `"RTK Fixed"` — use `RTK Fixed` for cm-level accuracy |
| `Centimeter_Error` | float32 | cm | Estimated horizontal position error reported by u-blox |
| `Satellites` | int32 | count | Number of satellites used in solution |

---

### RPi: spresense_gnss.csv
Source: `tpc_gnss_spresense` topic (~10 Hz) — Sony Spresense standard GPS

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `Latitude` | float32 | °  | WGS84 latitude |
| `Longitude` | float32 | °  | WGS84 longitude |
| `Altitude` | float32 | m  | Height above ellipsoid |
| `Satellites` | int32 | count | Number of satellites in use |

---

### RPi: chassis_imu.csv
Source: `tpc_chassis_imu` topic (~10 Hz) — LSM6DSV16X on STM32 chassis board

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `Accel_X` | int32 | m/s² × 1000 | X-axis acceleration (divide by 1000 for m/s²). Rover longitudinal axis. |
| `Accel_Y` | int32 | m/s² × 1000 | Y-axis acceleration. Rover lateral axis. |
| `Accel_Z` | int32 | m/s² × 1000 | Z-axis acceleration. Vertical. At rest ≈ 9810 (1 g). |
| `Gyro_X` | int32 | rad/s × 1000 | Angular velocity around X (roll rate) |
| `Gyro_Y` | int32 | rad/s × 1000 | Angular velocity around Y (pitch rate) |
| `Gyro_Z` | int32 | rad/s × 1000 | Angular velocity around Z (yaw rate) |

> Values are stored as raw integer × 1000. Divide by 1000.0 to get physical units (m/s², rad/s).

---

### RPi: chassis_sensors.csv
Source: `tpc_chassis_sensors` topic (~4 Hz) — INA226 + encoders on STM32 sensors board

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `Encoder_Left` | int32 | counts | Left motor encoder cumulative count |
| `Encoder_Right` | int32 | counts | Right motor encoder cumulative count |
| `Voltage_V` | float32 | V | Battery bus voltage measured by INA226 |
| `Current_A` | float32 | A | Battery bus current measured by INA226 |
| `Power_W` | float32 | W | Computed: `Voltage_V × Current_A` |

---

### RPi: chassis_cmd.csv
Source: `tpc_chassis_cmd` topic (~50 Hz) — commands sent by RPi chassis_controller_node to STM32

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `Left_Speed` | float32 | 0–255 | Speed command (`spd_msg`). Both wheels share the same value. |
| `Right_Speed` | float32 | 0–255 | Same as `Left_Speed` (logged separately for symmetry) |
| `Steer_Dir` | int | enum | **Steering direction** (`fdr_msg`): `1`=right, `2`=straight, `3`=left |
| `Drive_Dir` | int | enum | **Drive direction** (`bdr_msg`): `0`=stop, `1`=forward, `2`=backward |

---

### RPi: mission_state.csv
Source: multiple D5 topics (event-driven on any topic change)

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `Mission_Active` | bool | 0/1 | `1` = GNSS waypoint mission running; `0` = idle |
| `Distance_Remaining_km` | float64 | km | Remaining straight-line distance to destination waypoint |
| `Dest_Latitude` | float64 | °  | Current target waypoint latitude (from base station action) |
| `Dest_Longitude` | float64 | °  | Current target waypoint longitude |
| `Steering_Cmd` | float32 | — | Kinematic control output from Jetson (`tpc_rover_ctrl_cmd[0]`). Normalized steering demand fed to chassis_controller_node. |
| `Lane_Theta` | float64 | — | Lane center-line angle from `tpc_rover_nav_lane[0]`. Output of Hough / lane detection model on Jetson. |
| `Lane_B` | float64 | px | Lane center-line y-intercept (pixel offset) from `tpc_rover_nav_lane[1]`. Used by the bicycle-model PID. |
| `Lane_Detected` | bool | 0/1 | `1` = lane markers found in current frame; `0` = no lane (from `tpc_rover_nav_lane[2]`) |

---

### Jetson: telemetry_unified.csv
Source: `tpc_telemetry_relay` topic (5 Hz) — aggregated relay from RPi. One row = one relay message.

| Column | Type | Description |
|--------|------|-------------|
| `Timestamp` | ISO8601 string | Jetson wall-clock time when message was received |
| `Mission_Active` | bool | See mission_state above |
| `Distance_Remaining_km` | float64 | km to destination waypoint |
| `Spresense_Valid` | bool | `1` = Spresense GNSS data is fresh and valid |
| `Spresense_Lat/Lon/Alt` | float32 | Degrees / metres — Spresense GPS position |
| `Spresense_Sats` | int32 | Satellite count |
| `Ublox_Valid` | bool | `1` = RTK data is fresh and valid |
| `Ublox_Lat/Lon/Alt` | float64 | Degrees / metres — u-blox RTK position |
| `Ublox_Fix` | string | RTK fix quality string (e.g. `"RTK Fixed"`) |
| `Ublox_Err_cm` | float32 | cm — u-blox estimated horizontal error |
| `Ublox_Sats` | int32 | Satellite count |
| `Chassis_Cmd_Valid` | bool | `1` = chassis command data is fresh |
| `Cmd_Left_Speed` / `Cmd_Right_Speed` | float32 | Speed command 0–255 (both equal to `spd_msg`) |
| `Cmd_Steer_Dir` | int | Steering enum: 1=right, 2=straight, 3=left |
| `Cmd_Drive_Dir` | int | Drive enum: 0=stop, 1=forward, 2=backward |
| `Chassis_Sensors_Valid` | bool | `1` = sensor data is fresh |
| `Encoder_Left` / `Encoder_Right` | int32 | Motor encoder counts |
| `Voltage_V` / `Current_A` / `Power_W` | float32 | Battery measurements |
| `Chassis_IMU_Valid` | bool | `1` = IMU data is fresh |
| `Accel_X/Y/Z` | int32 | Raw × 1000 (÷1000 → m/s²) |
| `Gyro_X/Y/Z` | int32 | Raw × 1000 (÷1000 → rad/s) |
| `Steering_Valid` | bool | `1` = steering command is fresh |
| `Steering_Cmd` | float32 | Kinematic control output from Jetson |
| `Lane_Valid` | bool | `1` = lane detection data is fresh |
| `Lane_Theta` | float64 | Lane angle from Jetson detector |
| `Lane_B` | float64 | Lane y-intercept (px) |
| `Lane_Detected` | bool | `1` = lane visible in frame |
| `Dest_Valid` | bool | `1` = destination coordinate has been set |
| `Dest_Lat` / `Dest_Lon` | float32 | Target waypoint coordinates |

> **Down-sampling note:** Jetson logs at 5 Hz (relay rate). RPi logs at native topic rate (4–50 Hz). Use RPi logs for high-frequency analysis (motor commands, IMU). Use Jetson logs for aggregated mission analysis.

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


