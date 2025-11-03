# Almondmatcha - Multi-Board Rover Control System

Modular ROS2-enabled firmware for multi-board STM32 NUCLEO rovers with real-time task scheduling, motor control, and sensor integration.

## Project Structure

```
almondmatcha/
├── mros2-mbed-sensors-gnss/                Sensors node (Domain 6)
│   ├── workspace/sensors_node/
│   │   └── 3-task design (encoder, power, GNSS)
│   └── ...
├── mros2-mbed-chassis-dynamics/            Chassis controller (Domain 5)
│   ├── workspace/chassis_controller/
│   │   ├── app.cpp                  (269 lines)
│   │   ├── motor_control.h/cpp      (243 lines combined)
│   │   ├── led_status.h/cpp         (83 lines combined)
│   │   └── README.md                (modular structure documentation)
│   ├── platform/                    (mROS2 platform layer)
│   └── CMakeLists.txt
├── ws_base/                                 ROS2 base interfaces
├── ws_jetson/                               Jetson compute module
├── ws_rpi/                                  Raspberry Pi compute module
├── WORK_SESSION_2025-11-04.md               Session documentation
└── TOMORROW_ACTION_ITEMS.md                 Action checklist

```

## Key Boards

### Chassis Controller (Domain 5)
- **MCU:** NUCLEO-F767ZI (STM32F767ZI, Cortex-M7)
- **IP:** 192.168.1.2
- **Tasks:** 2 (Motor Control + IMU Reader)
- **Output:** 385.7 KB flash, 102 KB RAM
- **Source:** Modular (motor_control, led_status, app)
- **Documentation:** mros2-mbed-chassis-dynamics/workspace/chassis_controller/README.md

### Sensors Node (Domain 6)
- **MCU:** NUCLEO-F767ZI (STM32F767ZI, Cortex-M7)
- **IP:** 192.168.1.6
- **Tasks:** 3 (Encoder Reader + Power Monitor + GNSS Reader)
- **Output:** 384.5 KB flash
- **Documentation:** mros2-mbed-sensors-gnss/workspace/sensors_node/README.md

## Quick Start

### Build Chassis Controller
```bash
cd /home/yupi/almondmatcha/mros2-mbed-chassis-dynamics
./build.bash all NUCLEO_F767ZI chassis_controller
```

### Build Sensors Node
```bash
cd /home/yupi/almondmatcha/mros2-mbed-sensors-gnss
./build.bash all NUCLEO_F767ZI sensors_node
```

### Network Setup (Host Machine)
```bash
# Configure static IP on Ethernet interface
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip link set eth0 up

# Verify connectivity
ping 192.168.1.2  # Chassis controller
ping 192.168.1.6  # Sensors node
```

### Monitor & Test
```bash
# Set Domain ID for target board
export ROS_DOMAIN_ID=5  # For chassis controller

# List topics
ros2 topic list

# Publish motor commands to chassis controller
ros2 topic pub /pub_rovercontrol msgs_ifaces/msg/MainRocon \
  '{mainrocon_msg: {fdr_msg: 0, ro_ctrl_msg: 0.0, bdr_msg: 0, spd_msg: 50}}'

# Monitor IMU data
ros2 topic echo /tp_imu_data_d5
```

## Architecture

### Multi-Board Design
- **Centralized Orchestration:** Host machine (ROS2 publisher/subscriber)
- **Independent Domains:** Domain 5 (chassis) and Domain 6 (sensors) communicate via DDS
- **Task Model:** Each board uses independent task + centralized main loop pattern
- **Thread Safety:** Mutex-protected shared data structures

### Chassis Controller (Domain 5)
**Task 1: Motor Control (High Priority, 50 ms)**
- Subscribes to rover commands
- Calculates steering servo PWM and motor H-bridge signals
- Responsive control loop

**Task 2: IMU Reader (Normal Priority, 10 ms)**
- Polls LSM6DSV16X sensor at 100 Hz
- Publishes aggregated data at 10 Hz
- Non-blocking I2C operations

### Sensors Node (Domain 6)
**Task 1: Encoder Reader (Normal Priority, 20 ms)**
- Polls quadrature encoders for wheel speed feedback

**Task 2: Power Monitor (Normal Priority, 100 ms)**
- Monitors INA226 power sensors for battery voltage and current

**Task 3: GNSS Reader (Normal Priority, 100 ms)**
- Reads SimpleRTK2b GNSS module for positioning data

## Recent Updates (November 4, 2025)

### Chassis Controller Refactoring
- Refactored from 421-line monolithic app.cpp to modular 4-file architecture
- Motor control logic isolated in motor_control.h/cpp (reusable, testable)
- LED status control isolated in led_status.h/cpp (mockable for tests)
- app.cpp focuses on ROS2 integration and task orchestration
- Added LED1 status indication: OFF during init, ON solid when ready
- Improved API design: unified set_status_led(bool) vs separate on/off functions
- All changes committed to main branch

### Code Quality Improvements
- Professional naming conventions (snake_case, descriptive function names)
- Clear separation of concerns across modules
- Comprehensive documentation in workspace README files
- Thread-safe design with Mutex protection

## Documentation Files

- **WORK_SESSION_2025-11-04.md** - Complete session summary with technical details
- **TOMORROW_ACTION_ITEMS.md** - Quick checklist for next build/test steps
- **mros2-mbed-chassis-dynamics/README.md** - Chassis controller architecture overview
- **mros2-mbed-chassis-dynamics/workspace/chassis_controller/README.md** - Modular code organization
- **mros2-mbed-sensors-gnss/workspace/sensors_node/README.md** - Sensors node documentation

## Network Configuration

All boards use static IP addresses on 192.168.1.0/24 network:

| Device | Domain ID | IP Address |
|--------|-----------|------------|
| Chassis Controller | 5 | 192.168.1.2 |
| Sensors Node | 6 | 192.168.1.6 |
| Host Machine | 5 or 6 | 192.168.1.100+ |

Each board must use the same Domain ID as its intended subscribers for ROS2 DDS communication.

## Building & Flashing

### Prerequisites
- Docker (or native Mbed tools)
- STM32 NUCLEO-F767ZI board with USB/SWD debugger
- Mbed OS 6.x

### Build Steps
```bash
cd mros2-mbed-chassis-dynamics
./build.bash all
# Output: mros2-mbed.bin in build/ directory
```

### Flash to Board
1. Connect board via USB
2. Copy .bin to board mount point or use STM32 STLink Utility
3. Monitor serial output at 115200 baud

## Testing Checklist

- [ ] Build completes without errors
- [ ] LED1 OFF during startup (first ~1 second)
- [ ] LED1 ON solid after initialization
- [ ] Serial console shows all tasks starting
- [ ] Motor responds to ROS2 commands
- [ ] IMU data publishes at 10 Hz
- [ ] Network connectivity verified

## Git Workflow

All work is committed to the main branch. Latest commits:
- refactor(chassis): modularize chassis controller into motor_control and led_status modules
- docs: Consolidate refactoring documentation into README.md and remove summary file

## Future Work

- Non-blocking LED error indication via Ticker
- Motor controller C++ class for encapsulation
- Unit tests for PWM calculations
- Configuration file for hardware parameters
- Encoder feedback for closed-loop motor control
- Command timeout safety mechanism
- GNSS-based autonomous navigation

---

**Last Updated:** November 4, 2025
**Repository:** github.com/RoboticsGG/almondmatcha
**Branch:** main

