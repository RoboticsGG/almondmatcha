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
└── REFACTORING_SUMMARY.md                    Detailed module responsibilities & API contracts
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

## Build

```bash
cd /home/yupi/almondmatcha/mros2-mbed-chassis-dynamics
./build.bash all NUCLEO_F767ZI chassis_controller
```

The build system automatically discovers all `.cpp` and `.h` files in this directory via CMake.

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
