# mros2-mbed-sensors-gnss

STM32 NUCLEO-F767ZI multi-threaded sensors and GNSS/RTK node for autonomous rover system.

## Overview

This node runs three independent sensor polling tasks and aggregates data for ROS 2 publishing:

- **Encoder Task** (10 Hz, 100ms) - Reads quadrature encoders for left and right motors
- **Power Monitor Task** (5 Hz, 200ms) - Reads voltage and current via I2C INA226
- **GNSS Reader Task** (10 Hz, 100ms) - Reads NMEA sentences from SimpleRTK2b receiver
- **Main Loop** (4 Hz, 250ms) - Aggregates sensor data and publishes to ROS 2

All tasks use Mutex-protected shared data structure for thread-safe access.

## Quick Build

```bash
cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash all NUCLEO_F767ZI sensors_node
```

Output binary: `build/NUCLEO_F767ZI/sensors_node.bin`

## Hardware Configuration

### USART6 - SimpleRTK2b (GNSS/RTK)
| Signal | Arduino Pin | STM32 Pin |
|--------|-------------|-----------|
| TX1 (U-blox) | D0 | PG9 (USART6_RX) |
| RX1 (U-blox) | D1 | PG14 (USART6_TX) |
| Baud Rate | - | 115200 |
| Protocol | - | NMEA 0183 |

### I2C - INA226 (Power Monitoring)
| Signal | STM32 Pin |
|--------|-----------|
| SDA | PB9 |
| SCL | PB8 |
| Address | 0x40 |

### GPIO - Quadrature Encoders
| Encoder | Channel A | Channel B |
|---------|-----------|-----------|
| Left (A) | PA15 | PB5 |
| Right (B) | PB3 | PB4 |

## Deployment

1. Connect board via USB (will appear as DAPLINK/NODE_F767ZI)
2. Copy `build/NUCLEO_F767ZI/sensors_node.bin` to board mount point
3. Press reset button
4. Monitor serial console (115200 baud) for startup messages

## ROS 2 Publishing

**Topic:** `tp_sensdata_d5` (MainSensData)  
**Rate:** 250ms (4 Hz)

**Fields:**
- `mt_lf_encode_msg` - Left encoder count
- `mt_rt_encode_msg` - Right encoder count
- `sys_volt_msg` - Bus voltage (V)
- `sys_current_msg` - System current (A)

## Console Output

Main loop prints every 250ms:
```
MotorA: 1234 | MotorB: 5678 | Vbus: 12.500 V | I: 2.345 A
GNSS: $GPRMC,... (every 1 second)
```

## Architecture

Three independent threads update the shared `sensor_data` structure:
```cpp
struct {
    int32_t encoder_A;          // Updated by encoder_read_task @ 10Hz
    int32_t encoder_B;          // Updated by encoder_read_task @ 10Hz
    float bus_voltage;          // Updated by power_monitor_task @ 5Hz
    float current;              // Updated by power_monitor_task @ 5Hz
    char nmea_sentence[256];    // Updated by gnss_reader_task @ 10Hz (default: "$GNRMC,...")
} sensor_data;  // Protected by Mutex
```

Main loop reads this structure every 250ms and publishes to ROS 2 with fresh aggregated data.

## Sampling Rate Configuration

All task polling periods are defined as named constants in `app.cpp`:

```cpp
const uint32_t ENCODER_SAMPLE_PERIOD_MS = 100;    // Encoder task @ 10 Hz
const uint32_t POWER_SAMPLE_PERIOD_MS = 200;      // Power monitor task @ 5 Hz
const uint32_t GNSS_SAMPLE_PERIOD_MS = 100;       // GNSS reader task @ 10 Hz
const uint32_t MAIN_LOOP_PERIOD_MS = 250;         // Main publishing loop @ 4 Hz
const uint32_t GNSS_PRINT_INTERVAL = 4;           // Print GNSS every 4 main loops (1 second)
```

To adjust sampling rates, edit these constants at the top of `workspace/sensors_node/app.cpp` in the `SAMPLING_RATE_CONSTANTS` section.

## Build and Flash Information

- **Binary Size:** 384,460 bytes (384 KB)
- **RAM Usage:** 105,840 bytes (103 KB)
- **Compile Time:** ~60 seconds (using Docker)
- **Flash Method:** Drag-and-drop to DAPLINK mount or use OpenOCD

## Network Configuration

The STM32 board communicates with the host via Ethernet using ROS 2 DDS (Data Distribution Service).

### Default Network Settings

```
Board IP Address:  192.168.11.2
Host IP Address:   192.168.11.x (any address on same network)
Netmask:           255.255.255.0
Gateway:           192.168.11.1
Baud Rate (Serial): 115200
```

### Network Setup Instructions

1. **Hardware**: Connect board and host computer via Ethernet cable to the same network
2. **Network Interface**: Ensure both devices can reach the 192.168.11.0/24 subnet
3. **Firewall**: Disable firewall on host for DDS communication
   ```bash
   # Ubuntu
   sudo ufw disable
   ```
4. **Single Network**: If your host is connected to multiple networks (WiFi + Ethernet), disable WiFi during testing

### Changing IP Configuration

To use a different IP address, edit `platform/mros2-platform.h`:

```cpp
// Change these values in platform/mros2-platform.h:
#define MROS2_IP_ADDRESS_STATIC         // Comment out for DHCP
#define MROS2_IP_ADDRESS    "192.168.11.2"
#define MROS2_NETMASK       "255.255.255.0"
#define MROS2_GATEWAY       "192.168.11.1"
```

For DHCP (automatic IP assignment):
```cpp
// In platform/mros2-platform.h, comment out:
// #define MROS2_IP_ADDRESS_STATIC
```

## Testing ROS 2 Communication

After flashing the board:

1. Monitor serial console (115200 baud) to see initialization messages
2. On host, use ROS 2 tools to verify connection:
   ```bash
   # List all ROS 2 topics
   ros2 topic list
   
   # Subscribe to sensor data
   ros2 topic echo /tp_sensdata_d5
   ```

## References

- [mROS 2 GitHub](https://github.com/mROS-base/mros2)
- [Mbed OS 6 Documentation](https://github.com/ARMmbed/mbed-os)
- [SimpleRTK2b Product Page](https://www.u-blox.com/en/product/simplertk2b)
- [NUCLEO-F767ZI Datasheet](https://www.st.com/en/evaluation-tools/nucleo-f767zi.html)
