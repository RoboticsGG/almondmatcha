# STM32 Memory Configuration

## Current Configuration

**Boards**: NUCLEO-F767ZI (chassis dynamics & sensors controller)  
**Memory**: 114688 bytes heap, 60% free after initialization

### Participant Limits
```c
#define MAX_NUM_PARTICIPANTS 15
```

### Domain 5 Participant Count
- **STM32 nodes**: 2 (chassis dynamics, sensors)
- **Raspberry Pi nodes**: 9 (GNSS, navigation, control, monitoring)
- **Total**: 11 participants
- **Headroom**: 4 slots (26%)

## Memory Safety

✅ **Safe configuration** - 26% headroom provides buffer for:
- Transient participant creation during startup
- Future node additions
- Network topology changes

## Verification

Actual memory usage confirmed via mbed serial output during system startup. Centralized logging and Domain 4 separation ensure no additional Domain 5 participants from monitoring.

## Configuration Files

Both boards use identical settings in `platform/rtps/config.h`:
```c
// Domain 5 participant limits
#define MAX_NUM_PARTICIPANTS 15  // 11 actual + 4 headroom
```

### Chassis Dynamics Board (192.168.1.2)
- Node: `rover_node`
- Publishers: `/tpc_chassis_imu`, `/tpc_chassis_sensors`
- Subscribers: `/tpc_chassis_cmd`

### Sensors Controller Board (192.168.1.6)
- Node: `sensors_node`
- Publishers: `/tpc_chassis_sensors`
- Subscribers: None

## Future Capacity

Remaining capacity: 4 participants before reaching limit.  

**Recommendation**: Monitor participant count if adding new Domain 5 nodes. Consider increasing MAX_NUM_PARTICIPANTS if planning to add 3+ new nodes.

## Impact Assessment

### Centralized Logging (Current)
- CSV logging: Single node (`node_rover_monitoring`)
- Domain 5 participants: Unchanged
- STM32 impact: None ✅

### Domain 4 Monitoring
- Base station monitoring: Domain 4 only
- Domain 5 participants: Unchanged
- STM32 impact: None ✅

Both architectural changes maintain STM32 memory safety.
