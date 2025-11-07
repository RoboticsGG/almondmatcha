# mros2-mbed-chassis-dynamics

STM32 firmware for rover chassis motor control and IMU sensing using mROS2 (embedded ROS2).

## Quick Start

```bash
cd ~/almondmatcha/mros2-mbed-chassis-dynamics
sudo ./build.bash all NUCLEO_F767ZI chassis_controller
# Flash: Copy build/mros2-mbed.bin to NUCLEO board (drag-and-drop)
```

## Hardware

- **MCU:** NUCLEO-F767ZI (STM32F767ZI, Cortex-M7 @ 216 MHz)
- **Network:** 192.168.1.2 (Ethernet)
- **IMU Shield:** X-NUCLEO-IKS4A1 (LSM6DSV16X 6-axis sensor)
- **Domain:** ROS2 Domain 5

## Functions

| Task | Priority | Rate | Function |
|------|----------|------|----------|
| Motor Control | High | 50 ms | Steering servo + H-bridge motor control |
| IMU Reader | Normal | 10 ms | 6-axis accelerometer/gyroscope sampling |

## Building

### Prerequisites

- Docker (for mbed-os build environment)
- sudo access
- USB connection to NUCLEO board

### Build Command

```bash
cd ~/almondmatcha/mros2-mbed-chassis-dynamics
sudo ./build.bash all NUCLEO_F767ZI chassis_controller
```

**Build Output:**
- Binary: `build/mros2-mbed.bin` (385.7 KB flash, 102 KB RAM)
- Hex file: `build/mros2-mbed.hex`

### Build Time

~60 seconds (Docker compilation)

## Flashing

### Method 1: Drag-and-Drop (Easiest)

1. Connect NUCLEO board via USB
2. Board appears as `NOD_F767ZI` or `NODE_F767ZI` mass storage device
3. Copy `build/mros2-mbed.bin` to the board mount point
4. Board auto-resets and runs new firmware

### Method 2: STM32CubeProgrammer

1. Install STM32CubeProgrammer
2. Connect board via ST-LINK
3. Load `build/mros2-mbed.bin`
4. Program and verify

### Method 3: OpenOCD

```bash
openocd -f interface/stlink.cfg -f target/stm32f7x.cfg \
    -c "program build/mros2-mbed.bin 0x08000000 verify reset exit"
```

## Network Configuration

**Static IP:** 192.168.1.2  
**Netmask:** 255.255.255.0  
**Gateway:** 192.168.1.1  

**Configuration:** `platform/mros2-platform.h`

```cpp
#define MROS2_IP_ADDRESS_STATIC
#define MROS2_IP_ADDRESS    "192.168.1.2"
#define MROS2_SUBNET_MASK   "255.255.255.0"
#define MROS2_DEFAULT_GATEWAY "192.168.1.1"
```

**Domain ID:** 5 (configured in `workspace/chassis_controller/app.cpp`)

## Testing

### Serial Monitor

Connect via USB at 115200 baud:

```bash
minicom -D /dev/ttyACM0 -b 115200
# or
screen /dev/ttyACM0 115200
```

**Expected Output:**
```
[MROS2_INFO] Network connected successfully
[MROS2_INFO] ROS2 node initialized
[MROS2_INFO] LSM6DSV16X Sensor ID: 0x6C
[MROS2_INFO] Motor Control Task started (osPriorityHigh)
[MROS2_INFO] IMU Reader Task started (osPriorityNormal)
[MROS2_INFO] All initialization complete - ready to operate
```

### Verify Network Connectivity

```bash
# From host machine
ping 192.168.1.2

# Should respond if board is connected
```

### Check ROS2 Topics

```bash
export ROS_DOMAIN_ID=5
ros2 topic list

# Should see:
# /tpc_chassis_imu (published by STM32)
# /tpc_chassis_cmd (subscribed by STM32)
```

### Monitor IMU Data

```bash
export ROS_DOMAIN_ID=5
ros2 topic echo /tpc_chassis_imu
```

**Expected output:**
```yaml
accel_x: 100
accel_y: -50
accel_z: 1000
gyro_x: 5
gyro_y: -3
gyro_z: 2
---
```

### Send Motor Commands

```bash
export ROS_DOMAIN_ID=5
ros2 topic pub /tpc_chassis_cmd msgs_ifaces/msg/ChassisCtrl \
    '{fdr_msg: 0, ro_ctrl_msg: 5.0, bdr_msg: 0, spd_msg: 50}'
```

**Message Fields:**
- `fdr_msg`: Front direction (0=straight, 1=left, 2=right)
- `ro_ctrl_msg`: Steering angle in degrees (-45 to +45)
- `bdr_msg`: Back direction (0=forward, 1=backward, 2=stop)
- `spd_msg`: Motor speed 0-100%

## Pinout

### Motor Control

| Function | Pin | Note |
|----------|-----|------|
| Steering Servo PWM | PA_3 | 50 kHz, 1-2 ms pulse width |
| Right Motor PWM | PA_6 | H-bridge speed control |
| Left Motor PWM | PE_11 | H-bridge speed control |
| Right Motor Dir FWD | PF_12 | H-bridge enable A |
| Right Motor Dir REV | PD_15 | H-bridge enable B |
| Left Motor Dir FWD | PF_13 | H-bridge enable A |
| Left Motor Dir REV | PE_9 | H-bridge enable B |

### IMU Communication

| Signal | Pin | Note |
|--------|-----|------|
| I2C SDA | I2C_SDA | Default board I2C |
| I2C SCL | I2C_SCL | 400 kHz clock |
| Sensor | - | LSM6DSV16X @ 0x6A/0x6B |

### Network

| Signal | Interface |
|--------|-----------|
| Ethernet | Onboard RJ45 connector |

## ROS2 Communication

### Subscribed Topics

**`/tpc_chassis_cmd` (msgs_ifaces/ChassisCtrl) - 50 Hz**

Motor and steering commands from rover controller.

### Published Topics

**`/tpc_chassis_imu` (msgs_ifaces/ChassisIMU) - 10 Hz**

IMU sensor data (accelerometer + gyroscope).

**Data:**
- `accel_x, accel_y, accel_z`: Raw accelerometer values
- `gyro_x, gyro_y, gyro_z`: Raw gyroscope values

**Unit Conversion (LSM6DSV16X):**
- Accel: ±2g range → 0.061 mg/LSB
- Gyro: ±250 dps range → 8.75 mdps/LSB

## Sampling Rates

Configurable in `workspace/chassis_controller/app.cpp`:

```cpp
const uint32_t MOTOR_RESPONSE_PERIOD_MS = 50;   // Motor task: 20 Hz
const uint32_t IMU_SAMPLE_PERIOD_MS = 10;       // IMU sampling: 100 Hz
const uint32_t IMU_PUBLISH_INTERVAL = 10;       // Publish every 10 samples: 10 Hz
```

**To Change IMU Publish Rate:**
- 50 Hz: Set `IMU_PUBLISH_INTERVAL = 2`
- 20 Hz: Set `IMU_PUBLISH_INTERVAL = 5`
- 5 Hz: Set `IMU_PUBLISH_INTERVAL = 20`

## Troubleshooting

### Build Fails

**Symptom:** Docker build errors

**Solutions:**
```bash
# Ensure Docker is installed and running
sudo systemctl start docker
sudo systemctl status docker

# Clean build
cd ~/almondmatcha/mros2-mbed-chassis-dynamics
sudo rm -rf build
sudo ./build.bash all NUCLEO_F767ZI chassis_controller
```

### Board Not Detected

**Symptom:** Mass storage device doesn't appear

**Solutions:**
- Try different USB cable (data cable, not charge-only)
- Press reset button on NUCLEO board
- Check USB port permissions: `ls -l /dev/ttyACM*`

### No Network Connection

**Symptom:** Cannot ping 192.168.1.2

**Solutions:**
- Verify Ethernet cable connection
- Check network configuration in `platform/mros2-platform.h`
- Monitor serial console for network initialization errors
- Reflash firmware

### Topics Not Visible

**Symptom:** `ros2 topic list` doesn't show STM32 topics

**Solutions:**
```bash
# Verify domain ID
export ROS_DOMAIN_ID=5
ros2 topic list

# Check board network (ping)
ping 192.168.1.2

# Monitor serial console
minicom -D /dev/ttyACM0 -b 115200
# Look for "Publisher initialized" messages

# Restart ROS2 daemon
ros2 daemon stop && ros2 daemon start
```

### IMU Not Working

**Symptom:** No IMU data published

**Solutions:**
- Check serial console for "LSM6DSV16X Sensor ID: 0x6C"
- Verify IKS4A1 shield is properly seated on NUCLEO board
- Check I2C connections
- Reflash firmware

## Code Structure

```
mros2-mbed-chassis-dynamics/
├── README.md                     # This file
├── build.bash                    # Docker build script
├── mbed_app.json                 # Mbed OS configuration
├── CMakeLists.txt                # Build configuration
├── workspace/
│   └── chassis_controller/       # Main application
│       ├── app.cpp               # Main entry point, ROS2 init
│       ├── motor_control.h/cpp   # Motor control module
│       ├── led_status.h/cpp      # LED indicator module
│       └── README.md             # Detailed module documentation
├── platform/
│   ├── mros2-platform.h/cpp      # Network configuration
│   └── rtps/
│       └── config.h              # DDS/RTPS settings
├── libs/
│   └── X-Nucleo-IKS4A1_mbedOS/  # IMU sensor driver
└── mros2_add_msgs/
    ├── mros2_header_generator/   # Message header generator
    └── mros2_msgs/               # Generated message headers
```

## Detailed Documentation

See `workspace/chassis_controller/README.md` for:
- Modular architecture details
- Task descriptions and priorities
- Thread safety mechanisms
- Function reference
- Performance tuning

---

**MCU:** STM32F767ZI (NUCLEO-F767ZI)  
**Flash:** 385.7 KB  
**RAM:** 102 KB  
**Network:** 192.168.1.2  
**Domain:** 5  
**Framework:** mROS2 + Mbed OS 6
