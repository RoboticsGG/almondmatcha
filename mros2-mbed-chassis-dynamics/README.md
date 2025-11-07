# Rover Chassis Controller - Multi-Task Architecture

**Board:** NUCLEO-F767ZI (STM32F767ZI microcontroller)  
**ROS Domain ID:** 5  
**Build Output:** `mros2-mbed.bin` (385.7 KB flash, 102 KB RAM)

---

## Architecture Overview

This firmware implements a 2-task multi-threaded design using Mbed OS for 4-wheel rover motor control and IMU sensor acquisition. The architecture separates motor command processing from sensor data collection, enabling responsive steering and drive control while maintaining continuous vehicle orientation monitoring.

**Source Organization:** Modular (motor_control, led_status, and app modules) — see `workspace/chassis_controller/README.md` for details.

### Task Breakdown

| Task | Priority | Polling Rate | Function | Stack |
|------|----------|--------------|----------|-------|
| **Motor Control Task** | High | 50 ms (~20 Hz) | Subscribes to rover commands, drives motors & steering servo | 2 KB |
| **IMU Reader Task** | Normal | 10 ms (100 Hz raw) | Polls LSM6DSV16X 6-axis sensor, publishes at 10 Hz | 2 KB |
| **Main Loop** | Normal | - | ROS2 DDS communication spin (non-blocking) | - |

---

## Task Descriptions

### Task 1: Motor Control Subscriber (`motor_control_task`)

**Purpose:** Process incoming rover control commands and drive motors with minimal latency.

**Operation:**
1. Polls `rover_cmd` shared struct every `MOTOR_RESPONSE_PERIOD_MS` (50 ms)
2. Detects new command via `command_updated` flag
3. Calls `calculate_steering_pwm_duty()` to calculate steering servo PWM from steering angle
4. Calls `calculate_motor_direction()` to determine motor direction (forward/backward/stop) and speed
5. Calls `apply_motor_control()` to apply PWM signals to hardware
6. Uses `Mutex` to safely access shared `rover_cmd` struct

**Subscription:**
- **Topic:** `pub_rovercontrol` (message type: `MainRocon`)
- **Fields:** steering angle, motor speed, direction flags

**Hardware Control:**
- Steering servo: PWM on **PA_3** via `steering_servo_pwm` (standard RC servo, 1.0-2.0 ms pulse)
- Right motor PWM: **PA_6** via `motor_right_pwm`
- Left motor PWM: **PE_11** via `motor_left_pwm`
- Motor direction pins (H-bridge): **PF_12** (`motor_right_enable_forward`), **PD_15** (`motor_right_enable_backward`), **PF_13** (`motor_left_enable_forward`), **PE_9** (`motor_left_enable_backward`)

**Thread Safety:**
- Callback stores incoming commands in rover_cmd struct with Mutex protection
- Task extracts commands with minimal lock duration
- PWM hardware operations are non-blocking

---

### Task 2: IMU Reader & Publisher (`imu_reader_task`)

**Purpose:** Continuously monitor vehicle acceleration and rotation, publish sensor data at fixed rate.

**Operation:**
1. Polls LSM6DSV16X sensor every `IMU_SAMPLE_PERIOD_MS` (10 ms = 100 Hz capability)
2. Reads 3-axis accelerometer and 3-axis gyroscope
3. Every `IMU_PUBLISH_INTERVAL` (10 samples = 100 ms interval), publishes to ROS2
4. **Resulting publish rate:** 10 Hz (100 ms between publications)
5. Uses `Mutex` to safely update shared `imu_data` struct
6. Console logging shows all IMU samples

**Publication:**
- **Topic:** `tp_imu_data_d5` (message type: `MainGyroData`)
- **Fields:** accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z (all int32_t)
- **Rate:** 10 Hz (configurable via `IMU_PUBLISH_INTERVAL`)

**Hardware Communication:**
- IMU: **LSM6DSV16X** on I2C (SDA=I2C_SDA, SCL=I2C_SCL)
- I2C bus initialized at 400 kHz

**Thread Safety:**
- Local copies of sensor readings prevent blocking on I2C operations during publishing
- Shared imu_data struct protected by Mutex for external reader access

---

## Sampling Rate Configuration

All timing constants are configurable at the top of `app.cpp`:

```cpp
const uint32_t MOTOR_RESPONSE_PERIOD_MS = 50;   // Motor control polling (~20 Hz)
const uint32_t IMU_SAMPLE_PERIOD_MS = 10;      // IMU sensor polling (100 Hz capability)
const uint32_t IMU_PUBLISH_INTERVAL = 10;      // Publish every N samples (10 Hz output)
```

**To adjust IMU publish rate:**
- Default configuration: 10 Hz (samples at 100 Hz, publishes every 10 samples)
- For 50 Hz publish rate: Set IMU_PUBLISH_INTERVAL = 2
- For 5 Hz publish rate: Set IMU_PUBLISH_INTERVAL = 20

---

## Shared Data Structures

### `RoverCommandData` (Protected by `rover_cmd_mutex`)
```cpp
struct RoverCommandData {
    uint8_t front_direction;      // 0=straight, 1=left, 2=right
    float steering_angle;         // Steering angle in degrees
    uint8_t back_direction;       // 0=forward, 1=backward, 2=stop
    uint8_t motor_speed;          // Motor speed 0-100%
    bool command_updated;         // Flag: new command available
};
```

### `IMUData` (Protected by `imu_data_mutex`)
```cpp
struct IMUData {
    int32_t accelerometer[3];     // X, Y, Z acceleration
    int32_t gyro[3];              // X, Y, Z angular rate
};
```

---

## Control Functions

### `frontControl(frontDirection, diff_degree)`
Converts steering angle to servo PWM duty cycle.

- **Input:** Steering angle in degrees relative to center (0-100°)
- **Output:** PWM duty cycle (5%-10% for standard RC servo)
- **Range:** 90°-110° servo range maps to 1.0-2.0 ms pulse (50 kHz carrier)

### `backControl(backDirection, dutycycle_PWM)`
Determines motor direction and speed from control signals.

- **Directions:** 0=forward (EN_A=1, EN_B=0), 1=backward (EN_A=0, EN_B=1), 2=stop (EN_A=0, EN_B=0)
- **Output:** (motor_duty, EN_A, EN_B) tuple for H-bridge control
- **Speed range:** 0-100% motor PWM duty cycle

### `motorDrive(duty, EN_A, EN_B, period_PWM, percent_dutycycle)`
Applies PWM and direction signals to motor hardware.

- Configures 50 kHz PWM carrier (20 µs period)
- Sets steering servo PWM
- Sets motor direction pins (EN_A, EN_B)
- Applies motor speed (0-100% duty)

---

## Initialization Sequence

1. **Network Connection:** 500 ms (Ethernet)
2. **ROS2 Node Setup:** 200-300 ms (DDS, subscriber, publisher)
3. **IMU Initialization:** 100-200 ms (I2C enable, sensor ID read)
4. **Thread Launch:** 50-100 ms (OS scheduling)
5. **Safety Delay:** 1000 ms (`osDelay(1000)`) - **total init time: ~2 seconds**

After initialization completes, both tasks begin operation independently.

---

## Comparison to Sensors Node (Domain 6)

| Aspect | Sensors Node | Chassis Controller |
|--------|-----------------|------------------------|
| **Tasks** | 3 (encoder, power, GNSS) | 2 (motor control, IMU) |
| **Publish Rate** | 4 Hz (aggregated) | 10 Hz (IMU only) |
| **Subscribe Rate** | None | Event-driven + 50 ms polling |
| **Thread Priorities** | All Normal | Motor=High, IMU=Normal |
| **Synchronization** | Mutex on sensor_data | Mutex on rover_cmd + imu_data |
| **I/O Pattern** | Periodic polling | Motor: polling, IMU: polling |

**Design Philosophy:**
Both nodes implement the independent task plus centralized main loop pattern to achieve:
- Responsiveness: Tasks operate at native hardware rates without blocking
- Scalability: Straightforward addition or removal of tasks without architectural changes
- Safety: Mutex-protected shared data prevents race conditions
- Separation of concerns: Each task manages a specific control function or sensor

---

## Hardware Pinout Summary

### Motor Control
- **Steering Servo PWM:** PA_3 (20 µs period, 50 kHz)
- **Right Motor PWM:** PA_6
- **Left Motor PWM:** PE_11
- **Right Motor Direction:** PF_12 (EN_A), PD_15 (EN_B)
- **Left Motor Direction:** PF_13 (EN_A), PE_9 (EN_B)

### IMU Communication
- **I2C SDA:** I2C_SDA (mapped to STM32 board default)
- **I2C SCL:** I2C_SCL (mapped to STM32 board default)
- **Sensor:** LSM6DSV16X (6-axis: 3x accel + 3x gyro)

### Network
- **Ethernet:** NUCLEO-F767ZI onboard RJ45 connector
- **Static IP:** 192.168.1.2 (configured in `mros2-platform.h`)
- **DDS Domain ID:** 5 (via ROS_DOMAIN_ID environment variable)

---

## Network Configuration

### IP Address Configuration

The board uses a static IP address on the Ethernet interface:

**Board (Chassis Controller - Domain 5):**
- IP Address: `192.168.1.2`
- Netmask: `255.255.255.0`
- Gateway: `192.168.1.1`
- Configuration file: `platform/mros2-platform.h`

**Host Machine:**
- IP Address: `192.168.1.x` (where x is any address from 3-254)
- Netmask: `255.255.255.0`
- Gateway: `192.168.1.1`

### Setting up Host Network Interface

Connect the NUCLEO-F767ZI to host via Ethernet cable. Configure the host interface:

```bash
# View available network interfaces
ip addr show

# Configure static IP on Ethernet interface (example: eth0)
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip link set eth0 up

# Verify connectivity
ping 192.168.1.2
```

### DDS Domain Configuration

**Board Side (Firmware):**
- Domain ID: 5
- Set via environment variable: `setenv("ROS_DOMAIN_ID", "5", 5)`
- Defined at startup in main() function

**Host Side (ROS2):**
- Must use same Domain ID for communication
- Set before launching ROS2 applications:
```bash
export ROS_DOMAIN_ID=5
ros2 topic list
```

### Multi-Board Network Setup

For two-board rover system:

| Board | Function | Domain ID | IP Address | Purpose |
|-------|----------|-----------|------------|---------|
| NUCLEO-F767ZI #1 | Sensors Node | 6 | 192.168.1.6 | GNSS/Power/Encoders |
| NUCLEO-F767ZI #2 | Chassis Controller | 5 | 192.168.1.2 | Motor/IMU Control |
| Host Machine | ROS2 Publisher | 5 & 6 | 192.168.1.100 | Command/Monitor |

**Note:** Each board must use the same Domain ID as its intended subscribers. Host can switch domains by setting ROS_DOMAIN_ID before each command.

---

## Network Troubleshooting

### Board Not Reachable
```bash
# Check if board is visible on network
ping 192.168.1.2

# Verify Ethernet cable connection
ethtool eth0  # Should show "Link detected: yes"

# Check firewall (may need to disable)
sudo ufw status
sudo ufw disable  # If blocking communication
```

### ROS2 Topics Not Visible
```bash
# Verify Domain ID matches on both sides
echo $ROS_DOMAIN_ID  # Should be 5 for chassis controller

# Check ROS2 middleware
ros2 daemon stop
ros2 daemon start

# List available topics
ros2 topic list
```

### Serial Console Connection
For debugging via USB serial console:
```bash
minicom -D /dev/ttyACM0 -b 115200
# Or use screen
screen /dev/ttyACM0 115200
```

---

## Building & Flashing

### Build
```bash
cd /home/yupi/almondmatcha/mros2-mbed-chassis-dynamics
sudo ./build.bash all NUCLEO_F767ZI chassis_controller
```

### Expected Output
```
-- built: /var/mbed/build/mros2-mbed.bin
-- built: /var/mbed/build/mros2-mbed.hex
Total Flash memory (text + data): 385,660 bytes (385.7 KB)
Total Static RAM memory (data + bss): 104,344 bytes (102 KB)
```

### Flash to Board
1. Connect NUCLEO-F767ZI via USB (NOD_F767ZI1 mount point appears)
2. Use STM32 STLink Utility or drag .bin to board mount point
3. Monitor serial output at 115200 baud for debug logs

---

## Testing & Monitoring

### 1. Verify Initialization
Monitor serial console output (115200 baud) for the following startup messages:
```
[MROS2_INFO] Network connected successfully
[MROS2_INFO] LSM6DSV16X Sensor ID: 0x6C
[MROS2_INFO] Motor Control Task started (osPriorityHigh)
[MROS2_INFO] IMU Reader Task started (osPriorityNormal)
[MROS2_INFO] All initialization complete - ready to operate
```

### 2. Test Motor Control
Publish control commands to the rover node from a ROS2 host machine:
```bash
ros2 topic pub /pub_rovercontrol msgs_ifaces/msg/MainRocon \
  '{mainrocon_msg: {fdr_msg: 0, ro_ctrl_msg: 0.0, bdr_msg: 0, spd_msg: 50}}'
```

### 3. Monitor IMU Output
Subscribe to IMU sensor data on ROS2 host:
```bash
ros2 topic echo /tp_imu_data_d5
```

### 4. Serial Debug Output
Access task-level debug logs via serial console:
```bash
minicom -D /dev/ttyACM0 -b 115200
```

---

## Future Enhancements

- Encoder feedback monitoring on motor outputs for closed-loop speed control
- Command timeout mechanism with automatic motor shutdown if no control signal received within 500 ms
- PID-based steering control using IMU feedback for trajectory tracking
- GNSS integration for autonomous navigation (subscribe to GNSS data from sensors node)
- Power management integration through subscription to power sensor data from Domain 6

---

## Development Notes

### Code Quality and Naming Conventions

This codebase follows professional embedded systems naming standards for maintainability:

**Function Naming (Verb-Noun Pattern):**
- `calculate_steering_pwm_duty()` - Returns steering servo PWM duty cycle from angle
- `calculate_motor_direction()` - Determines motor direction and speed from control commands
- `apply_motor_control()` - Applies PWM and direction signals to hardware

**Variable Naming (Descriptive, snake_case):**
- `steering_servo_pwm` - Steering servo PWM control object (replaces DirectPWM)
- `motor_right_pwm`, `motor_left_pwm` - Motor PWM control objects
- `motor_right_enable_forward`, `motor_right_enable_backward` - Motor direction pins (replaces EN_A, EN_B)
- `servo_center_angle` - Servo center position (replaces servo_center)
- `pwm_period_us` - PWM period with explicit unit (replaces period_PWM)
- `current_steering_angle` - Current steering angle in degrees (replaces degree)

**Benefits:**
- Self-documenting code
- Fixed typos (Mortor → motor)
- Consistent naming throughout
- Easier debugging and maintenance
- Aligns with industry best practices

### Thread Safety
- **Mutex Locking:** Kept to minimal scope to avoid blocking critical operations
- **Callback Pattern:** Receiver callback stores data with lock, task processes without lock
- **Atomic Operations:** Single-threaded operations (like bool flags) don't need Mutex

### Performance Tuning
- Motor task priority: **High** (responsive steering control)
- IMU task priority: **Normal** (lower latency tolerance)
- Motor polling: **50 ms** - balance between responsiveness and CPU overhead
- IMU sampling: **10 ms** - 100 Hz raw sensor data capability

### Known Limitations
### Known Limitations

- Motor control operates in open-loop mode without encoder feedback on motor speed
- Steering servo angle derives from control message input without position feedback verification
- IMU publish rate is fixed at compile time and cannot be dynamically adjusted at runtime

---

**Last Updated:** November 3, 2025  
**Architecture Version:** 2-Task Multi-Threaded (Mbed OS 6)
