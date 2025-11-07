# ws_base - Mission Control Base Station

ROS 2 workspace for autonomous rover base station (Domain 2).

## Quick Start

```bash
cd ~/almondmatcha/ws_base
./launch_base_screen.sh
```

## Overview

**Purpose:** Command and monitor autonomous rover  
**Domain:** 6 (Base Station)  
**Platform:** Ubuntu 20.04/22.04, ROS 2 Humble/Iron

**Nodes:**
- `mission_command_node` - Generate navigation goals and speed limits
- `mission_monitoring_node` - Display real-time telemetry

## Installation

```bash
# Clone and build
cd ~/almondmatcha/ws_base
colcon build
source install/setup.bash
export ROS_DOMAIN_ID=2
```

## Usage

### Launch with Session Manager

```bash
# GNU Screen (recommended)
./launch_base_screen.sh

# Tmux (requires install)
./launch_base_tmux.sh
```

### Manual Launch

```bash
source install/setup.bash
export ROS_DOMAIN_ID=2

# Terminal 1: Command node
ros2 run mission_control mission_command_node

# Terminal 2: Monitoring node
ros2 run mission_control mission_monitoring_node
```

## Configuration

Edit `src/mission_control/config/params.yaml`:

```yaml
node_commands:
  ros__parameters:
    rover_spd: 15        # Speed limit (0-100%)
    des_lat: 7.007286    # Target latitude
    des_long: 100.50203  # Target longitude
```

## Communication

**Sends to Rover (Domain 5):**
- Action: `/des_data` (navigation goal)
- Service: `/spd_limit` (speed limit)

**Receives from Rover:**
- `tpc_gnss_spresense` - GNSS position
- `tpc_gnss_mission_active` - Mission status
- `tpc_gnss_mission_remain_dist` - Distance remaining
- `tpc_chassis_cmd` - Chassis commands

## Structure

```
ws_base/
├── launch_base_screen.sh     # GNU Screen launcher
├── launch_base_tmux.sh        # Tmux launcher
├── docs/                      # Documentation
│   ├── QUICK_START.md         # Quick reference
│   ├── ARCHITECTURE.md        # System design
│   ├── LAUNCH.md              # Launch scripts guide
│   ├── TOPICS.md              # Topic reference
│   └── SETUP.md               # Setup & troubleshooting
└── src/
    ├── common_ifaces/         # Shared interfaces
    │   ├── action_ifaces/     # Action definitions
    │   ├── msgs_ifaces/       # Message definitions
    │   └── services_ifaces/   # Service definitions
    └── mission_control/       # Main package
        ├── config/            # params.yaml
        ├── launch/            # Launch files
        └── src/               # Node source code
```

## Documentation

- [Quick Start Guide](docs/QUICK_START.md) - Fast reference
- [Architecture](docs/ARCHITECTURE.md) - System design
- [Launch Scripts](docs/LAUNCH.md) - Session managers
- [Topics Reference](docs/TOPICS.md) - Communication interfaces
- [Setup & Troubleshooting](docs/SETUP.md) - Installation and fixes

## Requirements

- ROS 2 Humble or Iron
- GNU screen or tmux
- Network access to rover (Domain 5)

## Troubleshooting

**Build fails:**
```bash
rm -rf build install log
colcon build
```

**Nodes won't start:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2
```

**No topics visible:**
```bash
ros2 topic list
ros2 doctor    # Check DDS configuration
```

See [SETUP.md](docs/SETUP.md) for detailed troubleshooting.

## Domain Configuration

| Domain | System | Description |
|--------|--------|-------------|
| 5 | ws_rpi | Rover control and navigation |
| 6 | ws_base | Base station (this workspace) |
| 7 | ws_jetson | Vision and autonomous navigation |

## License

See LICENSE file.

## Related Workspaces

- **ws_rpi** - Rover mission control (Raspberry Pi)
- **ws_jetson** - Vision processing (Jetson Nano)
- **mros2-mbed-chassis-dynamics** - STM32 chassis control
- **mros2-mbed-sensors-gnss** - STM32 GNSS/sensors
