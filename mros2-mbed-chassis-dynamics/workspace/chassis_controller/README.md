# Chassis Controller - Modular Source Organization

This directory contains the STM32 NUCLEO-F767ZI firmware for the rover chassis controller, organized into focused, reusable modules.

## Directory Structure

```
chassis_controller/
├── app.cpp                      (269 lines)  Main orchestration & ROS2 integration
├── motor_control.h              (102 lines)  Motor control API declarations
├── motor_control.cpp            (141 lines)  Motor PWM & steering servo implementations
├── led_status.h                 (44 lines)   Status LED API declarations
├── led_status.cpp               (39 lines)   Status LED implementations
└── README.md                    
```

## Module Overview

### 1. Motor Control Module (motor_control.h / motor_control.cpp)

**Responsibility:** Steering servo PWM calculation and H-bridge motor control

**Exports:**
- `struct RoverCommandData` — rover command structure (direction, angle, speed)
- `extern RoverCommandData rover_cmd` + `Mutex rover_cmd_mutex` — shared command data
- `float calculate_steering_pwm_duty(direction, angle_degrees)` — servo PWM from steering angle
- `std::tuple<duty, fwd, bwd> calculate_motor_direction(direction, speed%)` — motor direction determination
- `void apply_motor_control(servo_duty, fwd, bwd, pwm_period, speed%)` — apply signals to hardware
- `const uint32_t MOTOR_RESPONSE_PERIOD_MS` — motor polling rate (50 ms)

**Hardware Managed:**
- Steering servo: PA_3 (PwmOut, standard RC servo)
- Motor PWM: PA_6 (right), PE_11 (left)
- Motor direction: PF_12, PD_15, PF_13, PE_9 (H-bridge enable pins)

**Key Features:**
- Independent from ROS2 and task scheduling
- Fully testable motor control algorithms
- Reusable in other rover variants with similar motor architecture

---

### 2. LED Status Module (led_status.h / led_status.cpp)

**Responsibility:** Board status LED control for initialization indication

**Exports:**
- `void set_status_led(bool on)` — thread-safe LED on/off control
- `void status_led_blink_blocking(count, delay)` — blocking blink for error indication (startup only)

**Hardware Managed:**
- LED1 (DigitalOut on STM32 NUCLEO-F767ZI)

**Key Features:**
- Simple, composable API
- Thread-safe for basic operations
- Clear documentation of blocking behavior and limitations
- Easy to mock for unit testing

---

### 3. Main Application (app.cpp)

**Responsibility:** ROS2 integration, task orchestration, and IMU sensor handling

**Contains:**
- Task 1: `motor_control_task()` — subscribes to rover commands, drives motors
- Task 2: `imu_reader_task()` — polls IMU sensor, publishes aggregated data
- Callback: `rover_control_callback()` — ROS2 message handler
- `main()` — network setup, ROS2 initialization, task launch

**Dependencies:**
- `#include "motor_control.h"` — motor control API
- `#include "led_status.h"` — LED status API
- `mbed.h`, `mros2.h`, `plt_iks4a1.h` — platform libraries

**Key Features:**
- Clean separation of concerns
- Focused on architecture and coordination
- Easy to understand overall program flow
- Includes comprehensive logging and error handling

---

## Build Instructions

### Prerequisites

System Requirements:
- Linux (Ubuntu 18.04 LTS or newer)
- ARM GCC cross-compiler 10.3 or later
- Mbed OS toolchain
- CMake 3.19 or later
- Python 3.8 or later
- Git with submodule support

Verify Installation:
```bash
arm-none-eabi-gcc --version
cmake --version
python3 --version
```

### Quick Build

Standard compilation:
```bash
cd /home/yupi/almondmatcha/mros2-mbed-chassis-dynamics
sudo ./build.bash all NUCLEO_F767ZI chassis_controller
```

Output Artifacts:
- Flash Binary: `build/NUCLEO_F767ZI/mros2-mbed.bin` (385.8 KB)
- Intel HEX: `build/NUCLEO_F767ZI/mros2-mbed.hex`
- Executable ELF: `build/NUCLEO_F767ZI/mros2-mbed.elf`

Build Duration:
- Incremental build: 2-3 seconds (only changed files recompiled)
- First build: 5-10 minutes (all framework libraries compiled)

### Build System Configuration

The CMake build system automatically includes:
- Application source files: `app.cpp`, `motor_control.cpp`, `led_status.cpp`
- X-Nucleo-IKS4A1 IMU driver library
- Mbed OS 6.x core and drivers
- mROS2 v0.5.4 framework
- All required header paths for modular compilation

Module Source Files in CMakeLists.txt:
```cmake
set(APP_SRCS
  workspace/${APP_NAME}/app.cpp
  workspace/${APP_NAME}/motor_control.cpp
  workspace/${APP_NAME}/led_status.cpp
)
```

### Build Output Analysis

Memory Summary:
```
Total Static RAM memory (data + bss): 104344 bytes
Total Flash memory (text + data): 385788 bytes

Module Breakdown:
workspace/chassis_controller      2030 bytes flash, 389 bytes RAM
libs/X-Nucleo-IKS4A1_mbedOS      1328 bytes flash
mros2/embeddedRTPS              21156 bytes flash
[Remaining: Mbed OS, lwIP, mbedTLS, etc.]
```

### Clean Build

To rebuild from scratch:
```bash
cd /home/yupi/almondmatcha/mros2-mbed-chassis-dynamics
rm -rf build/
sudo ./build.bash all NUCLEO_F767ZI chassis_controller
```

### Build Troubleshooting

Problem: `error: cannot access 'mros2': No such file or directory`
Solution: Initialize Git submodules.
```bash
git submodule update --init --recursive
```

Problem: `arm-none-eabi-g++: command not found`
Solution: Install the ARM GCC cross-compiler.
```bash
sudo apt-get install gcc-arm-none-eabi
```

Problem: `CMake Error: cmake version 3.18`
Solution: Update CMake to version 3.19 or later.
```bash
sudo apt-get install --upgrade cmake
```

Problem: `undefined reference to 'motor_control_cpp'`
Solution: Module source files not in CMakeLists.txt. Verify:
```bash
grep "motor_control.cpp" CMakeLists.txt
grep "led_status.cpp" CMakeLists.txt
```

Problem: Build takes longer than expected on first run
Solution: This is normal. First build compiles all framework libraries (Mbed OS, mROS2). Subsequent builds only recompile modified files and are much faster.

### Customizing the Build

Adding a New Module:

1. Create header and implementation files:
```bash
touch workspace/chassis_controller/gripper_control.h
touch workspace/chassis_controller/gripper_control.cpp
```

2. Add to CMakeLists.txt in the project root:
```cmake
set(APP_SRCS
  workspace/${APP_NAME}/app.cpp
  workspace/${APP_NAME}/motor_control.cpp
  workspace/${APP_NAME}/led_status.cpp
  workspace/${APP_NAME}/gripper_control.cpp
)
```

3. Rebuild:
```bash
sudo ./build.bash all NUCLEO_F767ZI chassis_controller
```

### Deployment and Flashing

Connect Board via USB:
```bash
lsblk  # Identify DAPLINK mount point (typically /media/DAPLINK)
```

Copy Binary to Board:
```bash
cp build/NUCLEO_F767ZI/mros2-mbed.bin /media/DAPLINK/
```

Alternative: Using STLINK Programmer:
```bash
st-flash write build/NUCLEO_F767ZI/mros2-mbed.bin 0x8000000
```

Verify Flash Success:
- LED on board blinks
- Serial console at 115200 baud shows startup messages
- Board resets automatically

Monitor Serial Output:
```bash
minicom -D /dev/ttyACM0 -b 115200
# Or:
screen /dev/ttyACM0 115200
```

### Incremental Development Workflow

For faster iteration during development:

```bash
# 1. Edit a module (e.g., motor control)
nano workspace/chassis_controller/motor_control.cpp

# 2. Rebuild (only modified files recompiled, ~2-3 seconds)
sudo ./build.bash all NUCLEO_F767ZI chassis_controller

# 3. Flash to board
cp build/NUCLEO_F767ZI/mros2-mbed.bin /media/DAPLINK/

# 4. Monitor output
minicom -D /dev/ttyACM0 -b 115200
```

This cycle typically takes less than 10 seconds per iteration.

---

## Code Organization Benefits

| Benefit | Realization |
|---------|------------|
| Modularity | Motor and LED logic are independent; changes don't cascade |
| Reusability | `motor_control.*` can be used in other rover projects as-is |
| Testability | Motor PWM calculations can be unit-tested without mocking tasks or ROS2 |
| Maintainability | Each file is focused (44–141 lines) vs. 421-line monolith |
| Onboarding | New developers can understand one module without reading everything |
| Extensibility | Adding a 3rd subsystem (e.g., gripper) is straightforward |

---

## API Usage Examples

### Motor Control

```cpp
#include "motor_control.h"

// In motor_control_task():
float servo_duty = calculate_steering_pwm_duty(0, 15.0f);  // 0° steering, 15° angle
auto [duty, fwd, bwd] = calculate_motor_direction(0, 75);   // forward, 75% speed
apply_motor_control(servo_duty, fwd, bwd, 20, 75);         // apply all signals
```

### LED Status

```cpp
#include "led_status.h"

// In main():
set_status_led(false);  // OFF during init
// ... initialization ...
set_status_led(true);   // ON when ready

// In error path (startup only):
status_led_blink_blocking(5, std::chrono::milliseconds(200));
```

---

## Architecture Notes

### Task Design Pattern

Both the motors and IMU use the **independent task + centralized main loop** pattern:

1. Motor Control Task (High priority, 50 ms polling)
   - Continuously checks for new commands
   - Responds immediately when updates arrive
   - Minimal critical section (Mutex hold time)

2. IMU Reader Task (Normal priority, 10 ms sampling)
   - Polls sensor at 100 Hz capability
   - Publishes aggregated data at 10 Hz
   - Non-blocking I2C operations

3. Main Loop (ROS2 spin)
   - Handles network communication
   - Dispatches callbacks (rover control commands)
   - Non-blocking DDS middleware

### Thread Safety

- Mutex Protection: `rover_cmd_mutex` protects `rover_cmd` struct
- Minimal Lock Duration: Commands extracted with local copies
- Callback Pattern: Callback stores data with lock, task processes without lock
- Single-Threaded Operations: DigitalOut and PwmOut writes are atomic for single bits

---

## Related Documentation

- Parent README — Overall architecture, network configuration, testing procedures

---

## Migration & Design Notes

This modular structure was refactored from a single 421-line app.cpp on November 4, 2025. No functional changes were made to motor or LED control—only file organization changed. All Mutex protection, threading behavior, ROS2 integration, and hardware pin mappings remain identical.

**Key design decisions:**

- Motor control module is completely independent from ROS2 and task scheduling, enabling reuse in other rover variants
- LED status module provides a simple, thread-safe API suitable for mocking in unit tests
- app.cpp focuses solely on ROS2 integration and task orchestration
- All tasks use the independent task + centralized main loop pattern for responsiveness and safety

**Future enhancements:**

- Non-blocking LED patterns via Ticker for error indication during operation
- Motor controller class for further encapsulation
- Unit tests for PWM calculations
- Configuration file for magic numbers (servo ranges, motor mappings)

---

Architecture Version: 2-Task Multi-Threaded (Modular)
Last Updated: November 4, 2025
Mbed OS Version: 6.x
Board: NUCLEO-F767ZI (STM32F767ZI)
