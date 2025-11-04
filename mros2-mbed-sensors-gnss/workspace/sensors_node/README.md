# STM32 Sensors Node - Modular Architecture

**Board:** NUCLEO-F767ZI (STM32F767ZI microcontroller)  
**ROS Domain ID:** 6  
**Application Name:** Sensors Node (Domain 6)  
**Build Output:** `mros2-mbed.bin` (384.5 KB flash)

---

## Overview

This firmware implements a 3-task independent multi-threaded sensor acquisition system for autonomous rover navigation. The architecture uses modular, reusable components for quadrature encoders, power monitoring, and GNSS/RTK positioning. Each task operates independently at its native polling rate and feeds aggregated sensor data to a centralized publishing loop for ROS2 distribution.

**Design Pattern:** Modular architecture with separated concerns—each sensor interface is encapsulated in its own compilation unit with a well-defined public API.

---

## Module Architecture

### File Organization

```
sensors_node/
├── app.cpp                      ROS2 orchestration and sensor aggregation
├── encoder_control.h/cpp        Quadrature encoder interface
├── power_monitor.h/cpp          INA226 power monitoring interface
├── gnss_reader.h/cpp            NMEA/GNSS receiver interface
├── CMakeLists.txt               Build configuration
└── README.md                    Documentation
```

### Module Responsibilities

| Module | Responsibility | Hardware | Task Rate |
|--------|-----------------|----------|-----------|
| `encoder_control` | Quadrature encoder reading via interrupts | PA_15, PB_5, PB_3, PB_4 (GPIO) | 10 Hz (polled) |
| `power_monitor` | Bus voltage & current via I2C | PB_9, PB_8 (I2C @ 400 kHz) | 5 Hz (polled) |
| `gnss_reader` | NMEA sentence reading from USART6 | PG_14, PG_9 (USART6 @ 115200) | 10 Hz (polled) |
| `app.cpp` | Task launching, aggregation, ROS2 publishing | - | 4 Hz (main loop) |

---

## Task Architecture

### Task 1: Encoder Reader (10 Hz)

**Polling Period:** 100ms  
**Priority:** Normal  
**Stack Size:** 2 KB  
**Function:** Reads quadrature encoders via hardware interrupt handlers

**Operational Loop:**
1. Query encoder A count from volatile atomic counter
2. Query encoder B count from volatile atomic counter
3. Update shared data structure under mutex protection
4. Sleep 100ms before next poll

**Public API:**
- `encoder_init()` - Initialize interrupt handlers
- `encoder_get_count_a()` - Retrieve encoder A count
- `encoder_get_count_b()` - Retrieve encoder B count

**Hardware Interface:**
- Encoder A: PA_15 (Channel A), PB_5 (Channel B)
- Encoder B: PB_3 (Channel A), PB_4 (Channel B)
- Logic: Quadrature decoding with direction detection on rising/falling edges

### Task 2: Power Monitor (5 Hz)

**Polling Period:** 200ms  
**Priority:** Normal  
**Stack Size:** 3 KB  
**Function:** Acquires bus voltage and system current via I2C

**Operational Loop:**
1. Issue I2C read for bus voltage register (0x02)
2. Issue I2C read for shunt voltage register (0x01)
3. Convert raw ADC values to physical units using calibration constants
4. Update shared data structure under mutex protection
5. Sleep 200ms before next poll

**Public API:**
- `power_monitor_init()` - Configure I2C bus at 400 kHz
- `power_monitor_read_bus_voltage()` - Retrieve voltage in Volts
- `power_monitor_read_current()` - Retrieve current in Amperes

**Hardware Interface:**
- I2C SDA: PB_9
- I2C SCL: PB_8
- I2C Frequency: 400 kHz
- Device Address: 0x40 (INA226)
- Shunt Resistor: 0.1 Ohms
- Voltage Scale: 1.25 mV/LSB
- Current Scale: 2.5 µV/LSB (shunt voltage)

### Task 3: GNSS Reader (10 Hz)

**Polling Period:** 100ms  
**Priority:** Normal  
**Stack Size:** 4 KB  
**Function:** Acquires NMEA sentences from RTK-enabled GNSS receiver

**Operational Loop:**
1. Poll USART6 serial port for available data (non-blocking)
2. Buffer incoming characters into line buffer
3. Detect complete NMEA sentence (starts with '$', ends with CR/LF)
4. Extract and store sentence in shared data structure under mutex protection
5. Sleep 100ms before next poll

**Public API:**
- `gnss_reader_init()` - Configure USART6 at 115200 baud
- `gnss_reader_read_nmea()` - Non-blocking NMEA sentence acquisition
- `gnss_reader_get_default_sentence()` - Retrieve placeholder sentence

**Hardware Interface:**
- USART6 TX Pin: PG_14 (Arduino D1)
- USART6 RX Pin: PG_9 (Arduino D0)
- Baud Rate: 115200
- Protocol: NMEA 0183
- Device: SimpleRTK2b u-blox F9P

**Supported NMEA Sentence Types:**
- `$GPRMC` - Recommended Minimum Navigation Information
- `$GPGGA` - Global Positioning System Fix Data
- `$GPGSA` - GPS DOP and Active Satellites
- `$GPGSV` - GPS Satellites in View

### Main Loop (4 Hz)

**Polling Period:** 250ms  
**Priority:** Normal (ROS2 context)  
**Function:** Aggregates sensor data and publishes via ROS2 DDS

**Operational Loop:**
1. Acquire mutex and snapshot all sensor data
2. Construct MainSensData message with current values
3. Publish message to ROS2 topic at configured rate
4. Output formatted console debug information
5. Throttle and output GNSS data (every 1 second)
6. Sleep 250ms before next aggregation cycle

**Published Interface:**
- **Topic:** `tp_sensdata_d6` (ROS2 DDS)
- **Message Type:** `msgs_ifaces/msg/MainSensData`
- **Publication Rate:** 4 Hz (250ms)

**Message Payload:**
- `mt_lf_encode_msg` (int32_t) - Left wheel encoder count
- `mt_rt_encode_msg` (int32_t) - Right wheel encoder count
- `sys_volt_msg` (float) - Bus voltage in Volts
- `sys_current_msg` (float) - System current in Amperes

---

## Shared Data Structure

The sensor_data structure maintains aggregate state across all independent tasks with mutex-protected synchronization:

```cpp
struct {
    int32_t encoder_A;              // Updated by encoder_read_task at 10 Hz
    int32_t encoder_B;              // Updated by encoder_read_task at 10 Hz
    float bus_voltage;              // Updated by power_monitor_task at 5 Hz
    float current;                  // Updated by power_monitor_task at 5 Hz
    char nmea_sentence[256];        // Updated by gnss_reader_task at 10 Hz
} sensor_data;

Mutex sensor_data_mutex;            // Ensures atomic read/write access
```

**Synchronization Model:** All access to shared state is protected by `sensor_data_mutex`. Individual task writers hold the mutex for minimal duration (copy operations only). The main loop acquires the mutex once per cycle to snapshot all current values.

---

## Sampling Rate Configuration

All task polling periods are defined as constants at the top of `app.cpp`:

```cpp
const uint32_t ENCODER_SAMPLE_PERIOD_MS = 100;    // Encoder task @ 10 Hz
const uint32_t POWER_SAMPLE_PERIOD_MS = 200;      // Power monitor task @ 5 Hz
const uint32_t GNSS_SAMPLE_PERIOD_MS = 100;       // GNSS reader task @ 10 Hz
const uint32_t MAIN_LOOP_PERIOD_MS = 250;         // Main publishing loop @ 4 Hz
const uint32_t GNSS_PRINT_INTERVAL = 4;           // Print GNSS every 4 main loops (1 sec)
```

**To adjust sampling rates:**
1. Edit these constants in `workspace/sensors_node/app.cpp`
2. Rebuild: `./build.bash all NUCLEO_F767ZI sensors_node`

---

## Quick Build

```bash
cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash all NUCLEO_F767ZI sensors_node
```

**Output Binary:** `build/NUCLEO_F767ZI/sensors_node.bin` (384.5 KB)

---

## Module API Reference

### encoder_control.h

```cpp
void encoder_init();                          // Initialize interrupt handlers
int32_t encoder_get_count_a();                // Read encoder A count
int32_t encoder_get_count_b();                // Read encoder B count
void encoder_reset_count_a();                 // Reset encoder A to zero
void encoder_reset_count_b();                 // Reset encoder B to zero
```

### power_monitor.h

```cpp
void power_monitor_init();                    // Initialize I2C (400 kHz)
float power_monitor_read_bus_voltage();       // Read bus voltage (V)
float power_monitor_read_current();           // Read system current (A)
int power_monitor_read_voltage_and_current(float* v, float* i);  // Read both
int16_t power_monitor_read_register(uint8_t reg);  // Low-level I2C read
```

### gnss_reader.h

```cpp
void gnss_reader_init();                      // Initialize USART6 (115200)
size_t gnss_reader_read_nmea(char* buf, size_t size);  // Non-blocking read
const char* gnss_reader_get_default_sentence();  // Get placeholder
int gnss_reader_is_valid_sentence(const char* s); // Validate NMEA
```

---

## Deployment

1. **Connect board** via USB (appears as DAPLINK/NODE_F767ZI)
2. **Copy binary** to board mount point:
   ```bash
   cp build/NUCLEO_F767ZI/sensors_node.bin /mnt/DAPLINK/
   ```
3. **Press reset button** on board
4. **Monitor serial console** at 115200 baud for startup messages

---

## Network Configuration

**Static IP:** 192.168.1.6  
**Domain ID:** 6  
**Netmask:** 255.255.255.0  

On host machine:

```bash
export ROS_DOMAIN_ID=6
ros2 topic echo /tp_sensdata_d6
```

---

## Console Output Example

```
MotorA: 1234 | MotorB: 5678 | Vbus: 12.500 V | I: 2.345 A
GNSS: $GPRMC,092345.00,4717.113210,N,00833.915187,E,0.146,90.00,050721,,,A*54
MotorA: 1240 | MotorB: 5684 | Vbus: 12.510 V | I: 2.350 A
MotorA: 1246 | MotorB: 5690 | Vbus: 12.495 V | I: 2.340 A
```

---

## Testing Checklist

- [ ] Build completes without errors or warnings
- [ ] Binary size approximately 384 KB (within STM32F767 flash constraints)
- [ ] Serial console displays initialization messages for all three sensor tasks
- [ ] Encoder counts increment and decrement correctly as wheels rotate
- [ ] Bus voltage readings remain stable in expected operating range (11-14 V)
- [ ] Current readings vary appropriately with motor load conditions (0-5 A typical)
- [ ] GNSS displays valid NMEA sentences at regular 1-second intervals
- [ ] ROS2 topic `tp_sensdata_d6` publishes at configured 4 Hz rate
- [ ] No mutex deadlock conditions observed during extended operation

---

## Modular Design Benefits

* **Reusability** - Each sensor module is self-contained and can be integrated into other projects
* **Testability** - Individual modules support isolated unit testing without full ROS2 setup
* **Maintainability** - Sensor logic is encapsulated with clear boundaries and minimal coupling
* **Scalability** - New sensors can be added with corresponding new modules; existing code remains unchanged
* **Code Clarity** - Clear separation of concerns across distinct compilation units  

---

## Future Enhancements

- [ ] NMEA sentence parsing to extract latitude, longitude, and altitude coordinates
- [ ] Encoder error detection with stuck wheel and sensor degradation handling
- [ ] Power monitor threshold alerts for low battery or overcurrent conditions
- [ ] Sensor fusion module combining GNSS, encoder, and IMU data streams
- [ ] Configuration file system for hardware parameters and calibration constants
- [ ] Embedded self-test suite executed at startup
- [ ] Performance timing instrumentation and latency analysis

---

## References

- [Mbed OS 6 Documentation](https://github.com/ARMmbed/mbed-os)
- [mROS 2 GitHub](https://github.com/mROS-base/mros2)
- [SimpleRTK2b Product Page](https://www.u-blox.com/en/product/simplertk2b)
- [STM32F767 Datasheet](https://www.st.com/en/microcontrollers-microprocessors/stm32f767ii.html)

---

**Last Updated:** November 4, 2025  
**Architecture Version:** 2.0 (Modular Design)  
**Previous Version:** 1.0 (Monolithic Implementation)
