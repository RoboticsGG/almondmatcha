# Domain Architecture

## Domain Separation

| Domain | Purpose | Participants | Network |
|--------|---------|--------------|---------|
| **Domain 4** | Monitoring | Base station display, domain relay | Ethernet |
| **Domain 5** | Rover Control | All rover nodes, STM32 boards, commands | Ethernet |
| **Domain 6** | Vision | Jetson camera/lane detection | Localhost only |

## Domain 5 (Rover Control)

**Participants**: 11 total
- 2x STM32 mbed nodes (chassis dynamics, sensors)
- 9x Raspberry Pi nodes (GNSS, navigation, control, monitoring)

**Topics**: Sensor data, mission commands, chassis control

## Domain 4 (Monitoring)

**Purpose**: Base station monitoring without adding Domain 5 participants

**Bridge**: `domain_relay.py` relays `/tpc_rover_status` from Domain 5 → Domain 4

## Domain 6 (Vision)

**Purpose**: Keep camera data local to Jetson (reduce network load)

**Scope**: Localhost only, no external DDS traffic

## Memory Considerations

**STM32 Configuration**:
- MAX_NUM_PARTICIPANTS: 15
- Domain 5 participants: 11
- Headroom: 4 (26%)

Centralized logging and Domain 4 separation maintain STM32 memory safety.
