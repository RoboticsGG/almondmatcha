# Topics Reference

## Base Station Topics (Domain 2)

### Action Clients
| Name | Type | Direction | Purpose |
|------|------|-----------|---------|
| `/des_data` | DesData | BASE→ROVER | Navigation goal (lat/long, feedback: distance) |

### Service Clients
| Name | Type | Direction | Purpose |
|------|------|-----------|---------|
| `/spd_limit` | SpdLimit | BASE→ROVER | Speed limit command (0-100%) |

### Subscribed Topics
| Name | Type | Rate | Source | Purpose |
|------|------|------|--------|---------|
| `tpc_gnss_spresense` | SpresenseGNSS | 1Hz | ws_rpi | Current GNSS position |
| `tpc_gnss_mission_active` | Bool | 1Hz | ws_rpi | Mission active status |
| `tpc_gnss_mission_remain_dist` | Float64 | 1Hz | ws_rpi | Distance to target (km) |
| `tpc_chassis_cmd` | ChassisCtrl | 10Hz | ws_rpi | Current chassis commands |
| `tpc_rover_dest_coordinate` | Float64MultiArray | event | ws_rpi | Target coordinates |

## Message Types

### SpresenseGNSS
```yaml
# Position and fix data
uint16 date, hour, minute, sec      # Timestamp
uint8 sat_num, fix_ind              # Satellite count, fix quality
float64 latitude, longitude         # Position (decimal degrees)
float64 altitude                    # Altitude (meters)
```

### ChassisCtrl
```yaml
uint8 steer_dir        # 0=maintain, 1=left, 2=right
uint8 ctrl_val         # Steering value
uint8 rover_spd        # Speed (0-100%)
uint8 move_inst        # 0=stop, 1=forward, 2=reverse
```

### DesData (Action)
```yaml
# Goal
float64 des_lat, des_long

# Feedback
float64 dis_remain     # Distance remaining (km)

# Result
uint8 result_fser      # Result code
```

### SpdLimit (Service)
```yaml
# Request
uint8 rover_spd        # Speed limit (0-100%)

# Response
uint8 ack              # Acknowledgment
```

## Topic Naming Convention

Pattern: `tpc_<subsystem>_<description>`

Examples:
- `tpc_gnss_spresense` - GNSS subsystem, Spresense data
- `tpc_chassis_cmd` - Chassis subsystem, command data
- `tpc_rover_dest_coordinate` - Rover subsystem, destination

## Domain Mapping

| Domain | Workspace | Topics Visible |
|--------|-----------|----------------|
| 5 | ws_rpi | All rover control topics |
| 6 | ws_base | Base station + rover (bridged) |
| 7 | ws_jetson | Vision + rover (bridged) |

**Note:** Cross-domain visibility requires DDS multicast/discovery configuration.

## Debug Commands

```bash
# List all topics
ros2 topic list

# Topic info
ros2 topic info /des_data
ros2 topic info tpc_gnss_spresense

# Echo topic
ros2 topic echo tpc_chassis_cmd

# Check action status
ros2 action list
ros2 action info /des_data

# Service list
ros2 service list
ros2 service type /spd_limit
```
