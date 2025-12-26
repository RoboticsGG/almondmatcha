# ROS2 Multi-Domain Architecture

## Domain Strategy
Multi-domain ROS2 DDS configuration isolating monitoring, control, and vision traffic while protecting STM32 memory.

## Domain Allocation

### Domain 4: Base Station Monitoring
- **Purpose**: Display-only monitoring without adding Domain 5 participants
- **Nodes**: `node_base_monitoring` (ws_base)
- **Data**: Aggregated `/tpc_rover_status` from Domain 5 via relay
- **Benefit**: Zero STM32 memory impact from base station

### Domain 5: Rover Control & Operations
- **Purpose**: Primary rover operations, sensor data, mission control
- **Participants**: 11 total (2 STM32 + 9 Raspberry Pi)
- **STM32 Memory**: MAX_NUM_PARTICIPANTS=15, actual=11, headroom=4
- **Topics**: All sensor data, chassis control, mission commands, GNSS

### Domain 6: Vision Processing (Jetson)
- **Purpose**: Camera data and lane detection (localhost only)
- **Scope**: Jetson Orin Nano internal only
- **Benefit**: Reduces Ethernet multicast, keeps video local

## Domain Bridge

**Implementation**: `domain_relay.py` (Python, ws_rpi)  
- Subscribes `/tpc_rover_status` in Domain 5
- Republishes to Domain 4
- Dual ROS2 context with queue-based threading

## Memory Protection

STM32 boards (Domain 5 only):
```
MAX_NUM_PARTICIPANTS: 15
Actual participants:  11
Free slots:           4 (26% headroom)
Heap size:            114688 bytes
Free RAM after init:  60%
```

Domain 4 monitoring does not create Domain 5 participants, protecting STM32 memory.

## Launch Configuration

### ws_rpi (Raspberry Pi)
- All nodes launch in Domain 5 (default)
- `domain_relay.py` bridges to Domain 4
- `node_rover_monitoring` logs CSV, publishes status

### ws_jetson (Jetson Orin Nano)
- Vision nodes: Domain 6 localhost (export ROS_DOMAIN_ID=6)
- Steering control: Domain 5 (export ROS_DOMAIN_ID=5)

### ws_base (Base Station)
- Monitoring: Domain 4 (export ROS_DOMAIN_ID=4)
- Commands: Domain 5 (export ROS_DOMAIN_ID=5)

## Benefits
1. STM32 memory protection (Domain 4 isolated)
2. Network efficiency (Domain 6 localhost only)
3. Monitoring scalability (multiple base stations)
4. Clean separation of concerns
