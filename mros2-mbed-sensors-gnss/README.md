# mros2-mbed-sensors-gnss

STM32 firmware for rover sensor acquisition (encoders, power monitoring, GNSS) using mROS2.

## Quick Start

```bash
cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash all NUCLEO_F767ZI sensors_node
# Flash: Copy build/mros2-mbed.bin to NUCLEO board (drag-and-drop)
```

## Hardware

- **MCU:** NUCLEO-F767ZI (STM32F767ZI, Cortex-M7 @ 216 MHz)
- **Network:** 192.168.1.6 (Ethernet)
- **GNSS Module:** SimpleRTK2b (UART)
- **Power Sensor:** INA226 (I2C)
- **Domain:** ROS2 Domain 5

## Functions

| Task | Rate | Function |
|------|------|----------|
| Encoder Reader | 100 ms | Quadrature encoder polling |
| Power Monitor | 200 ms | INA226 voltage/current reading |
| GNSS Reader | 100 ms | SimpleRTK2b NMEA sentence parsing |
| Main Loop | 250 ms | Aggregate and publish all sensor data |

## Building

### Prerequisites

- Docker (for mbed-os build environment)
- sudo access
- USB connection to NUCLEO board

### Build Command

```bash
cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash all NUCLEO_F767ZI sensors_node
```

**Build Output:**
- Binary: `build/mros2-mbed.bin` (384.5 KB flash, 103 KB RAM)
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

**Static IP:** 192.168.1.6  
**Netmask:** 255.255.255.0  
**Gateway:** 192.168.1.1  

**Configuration:** `platform/mros2-platform.h`

```cpp
#define MROS2_IP_ADDRESS_STATIC
#define MROS2_IP_ADDRESS    "192.168.1.6"
#define MROS2_SUBNET_MASK   "255.255.255.0"
#define MROS2_DEFAULT_GATEWAY "192.168.1.1"
```

**Domain ID:** 5 (configured in `workspace/sensors_node/app.cpp`)

## Pinout

### GNSS Module (SimpleRTK2b)

| Signal | Arduino Pin | STM32 Pin | Note |
|--------|-------------|-----------|------|
| TX (u-blox) | D0 | PG9 (USART6_RX) | Board receives GNSS data |
| RX (u-blox) | D1 | PG14 (USART6_TX) | Board sends commands |
| Baud Rate | - | 115200 | NMEA 0183 protocol |

### Power Monitor (INA226)

| Signal | STM32 Pin | Note |
|--------|-----------|------|
| SDA | PB9 | I2C data |
| SCL | PB8 | I2C clock |
| Address | - | 0x40 (7-bit) |

### Quadrature Encoders

| Encoder | Channel A | Channel B | Note |
|---------|-----------|-----------|------|
| Left (A) | PA15 | PB5 | Motor A encoder |
| Right (B) | PB3 | PB4 | Motor B encoder |

### Network

| Signal | Interface |
|--------|-----------|
| Ethernet | Onboard RJ45 connector |

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
[MROS2_INFO] ROS2 node initialized (Domain 5)
[MROS2_INFO] Encoder Task started
[MROS2_INFO] Power Monitor Task started
[MROS2_INFO] GNSS Reader Task started
[MROS2_INFO] Main loop publishing at 4 Hz

MotorA: 12345 | MotorB: 12340 | Vbus: 12.500 V | I: 2.345 A
GNSS: $GPRMC,... (every 1 second)
```

### Verify Network Connectivity

```bash
# From host machine
ping 192.168.1.6

# Should respond if board is connected
```

### Check ROS2 Topics

```bash
export ROS_DOMAIN_ID=5
ros2 topic list

# Should see:
# /tpc_chassis_sensors (published by STM32)
```

### Monitor Sensor Data

```bash
export ROS_DOMAIN_ID=5
ros2 topic echo /tpc_chassis_sensors
```

**Expected output:**
```yaml
mt_lf_encode_msg: 12345      # Left encoder count
mt_rt_encode_msg: 12340      # Right encoder count
sys_volt_msg: 12.5           # Battery voltage (V)
sys_current_msg: 2.3         # System current (A)
---
```

## ROS2 Communication

### Published Topics

**`/tpc_chassis_sensors` (msgs_ifaces/ChassisSensors) - 4 Hz**

Aggregated sensor data from all tasks.

**Message Fields:**
- `mt_lf_encode_msg`: Left motor encoder count (int32)
- `mt_rt_encode_msg`: Right motor encoder count (int32)
- `sys_volt_msg`: System voltage in volts (float32)
- `sys_current_msg`: System current in amperes (float32)

### Subscribed Topics

None (this board only publishes sensor data).

## Sampling Rates

Configurable in `workspace/sensors_node/app.cpp`:

```cpp
const uint32_t ENCODER_SAMPLE_PERIOD_MS = 100;    // Encoder: 10 Hz
const uint32_t POWER_SAMPLE_PERIOD_MS = 200;      // Power: 5 Hz
const uint32_t GNSS_SAMPLE_PERIOD_MS = 100;       // GNSS: 10 Hz
const uint32_t MAIN_LOOP_PERIOD_MS = 250;         // Publish: 4 Hz
const uint32_t GNSS_PRINT_INTERVAL = 4;           // Print GNSS every 1s
```

**To Adjust Rates:**

Edit constants in `workspace/sensors_node/app.cpp` and rebuild.

## Troubleshooting

### Build Fails

**Symptom:** Docker build errors

**Solutions:**
```bash
# Ensure Docker is running
sudo systemctl start docker

# Clean build
cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo rm -rf build
sudo ./build.bash all NUCLEO_F767ZI sensors_node
```

### Board Not Detected

**Symptom:** Mass storage device doesn't appear

**Solutions:**
- Use data USB cable (not charge-only)
- Press reset button on NUCLEO board
- Check USB permissions: `ls -l /dev/ttyACM*`

### No Network Connection

**Symptom:** Cannot ping 192.168.1.6

**Solutions:**
- Verify Ethernet cable connection
- Check network configuration in `platform/mros2-platform.h`
- Monitor serial console for network initialization
- Verify IP doesn't conflict with other devices

### Topics Not Visible

**Symptom:** `ros2 topic list` doesn't show `/tpc_chassis_sensors`

**Solutions:**
```bash
# Verify domain ID
export ROS_DOMAIN_ID=5
ros2 topic list

# Check board network
ping 192.168.1.6

# Monitor serial console
minicom -D /dev/ttyACM0 -b 115200
# Look for "Publisher initialized" message

# Restart ROS2 daemon
ros2 daemon stop && ros2 daemon start
```

### No GNSS Data

**Symptom:** GNSS field in serial output shows default sentence

**Solutions:**
- Verify SimpleRTK2b connections (TX to PG9, RX to PG14)
- Check GNSS module power
- Ensure GNSS module has clear sky view for satellite lock
- Verify baud rate (115200) matches module configuration
- Monitor serial console for NMEA sentences

### Power Monitor Reads Zero

**Symptom:** Voltage and current both 0.0

**Solutions:**
- Check INA226 I2C connections (SDA to PB9, SCL to PB8)
- Verify INA226 power supply
- Check I2C address (0x40 by default)
- Use I2C scanner to detect device

### Encoders Not Incrementing

**Symptom:** Encoder counts stay at zero

**Solutions:**
- Verify encoder connections (Channel A and B pins)
- Test motor rotation (encoders should count)
- Check GPIO pin configuration in code
- Ensure quadrature encoder signals are clean (debounce if needed)

## Code Structure

```
mros2-mbed-sensors-gnss/
├── README.md                     # This file
├── build.bash                    # Docker build script
├── mbed_app.json                 # Mbed OS configuration
├── CMakeLists.txt                # Build configuration
├── workspace/
│   └── sensors_node/             # Main application
│       ├── app.cpp               # Main entry, ROS2 init, main loop
│       ├── encoder_control.h/cpp # Encoder polling task
│       ├── power_monitor.h/cpp   # INA226 power monitoring task
│       ├── gnss_reader.h/cpp     # GNSS NMEA parsing task
│       └── README.md             # Detailed module documentation
├── platform/
│   ├── mros2-platform.h/cpp      # Network configuration
│   └── rtps/
│       └── config.h              # DDS/RTPS settings (Domain 5)
└── mros2_add_msgs/
    ├── mros2_header_generator/   # Message header generator
    └── mros2_msgs/               # Generated message headers
```

## Architecture

Three independent tasks update shared data structure:

```cpp
struct SensorData {
    int32_t encoder_A;           // Updated by encoder_task @ 10 Hz
    int32_t encoder_B;
    float bus_voltage;           // Updated by power_task @ 5 Hz
    float current;
    char nmea_sentence[256];     // Updated by gnss_task @ 10 Hz
} sensor_data;  // Protected by Mutex
```

Main loop reads this structure every 250 ms and publishes aggregated data to ROS2.

## Detailed Documentation

See `workspace/sensors_node/README.md` for:
- Task architecture details
- Sensor integration
- Thread synchronization
- Hardware specifications

---

**MCU:** STM32F767ZI (NUCLEO-F767ZI)  
**Flash:** 384.5 KB  
**RAM:** 103 KB  
**Network:** 192.168.1.6  
**Domain:** 5  
**Framework:** mROS2 + Mbed OS 6
