# STM32 Sensors Node - Modular Architecture

**Board:** NUCLEO-F767ZI (STM32F767ZI microcontroller)  
**ROS Domain ID:** 6  
**Application Name:** Sensors Node (Domain 6)  
**Build Output:** `mros2-mbed.bin` (384.5 KB flash)

---

## Overview

This firmware implements a 3-task independent multi-threaded sensor acquisition system for autonomous rover navigation. The architecture uses modular, reusable components for quadrature encoders, power monitoring, and GNSS/RTK positioning. All tasks run independently with their own polling rates and feed aggregated sensor data to a main publishing loop.

**Modular Design:** The code is organized into focused modules (encoder_control, power_monitor, gnss_reader) that each manage a specific sensor interface. The main app.cpp focuses purely on task orchestration and ROS2 integration.

---

## Module Architecture

### File Organization

```
sensors_node/
├── app.cpp                      (ROS2 orchestration + main loop)
├── encoder_control.h/cpp        (Quadrature encoder API)
├── power_monitor.h/cpp          (INA226 power monitoring API)
├── gnss_reader.h/cpp            (NMEA/GNSS reader API)
├── CMakeLists.txt               (Build configuration)
└── README.md                    (This file)
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
**Stack:** 2 KB

```
Loop:
  1. Read encoder_A_count (from interrupt handler)
  2. Read encoder_B_count (from interrupt handler)
  3. Store in sensor_data struct (mutex-protected)
  4. Sleep 100ms
```

**API Used:**
- `encoder_init()` - Attach interrupt handlers
- `encoder_get_count_a()` - Read encoder A count
- `encoder_get_count_b()` - Read encoder B count

**Hardware:**
- Encoder A: PA_15 (Channel A), PB_5 (Channel B)
- Encoder B: PB_3 (Channel A), PB_4 (Channel B)
- Quadrature logic: Rising/falling edge detection determines direction

### Task 2: Power Monitor (5 Hz)

**Polling Period:** 200ms  
**Priority:** Normal  
**Stack:** 3 KB

```
Loop:
  1. Read INA226 bus voltage register (0x02)
  2. Read INA226 shunt voltage register (0x01)
  3. Convert ADC values to physical units (V, A)
  4. Store in sensor_data struct (mutex-protected)
  5. Sleep 200ms
```

**API Used:**
- `power_monitor_init()` - Configure I2C @ 400 kHz
- `power_monitor_read_bus_voltage()` - Read voltage (Volts)
- `power_monitor_read_current()` - Read current (Amps)

**Hardware:**
- I2C SDA: PB_9
- I2C SCL: PB_8
- INA226 I2C Address: 0x40
- Shunt Resistor: 0.1 Ohms
- Voltage LSB: 1.25 mV/LSB
- Current LSB: 2.5 µV/LSB (via shunt voltage)

### Task 3: GNSS Reader (10 Hz)

**Polling Period:** 100ms  
**Priority:** Normal  
**Stack:** 4 KB

```
Loop:
  1. Poll USART6 serial port for available data
  2. Buffer incoming characters until complete NMEA sentence
  3. Extract NMEA sentence (starts with '$', ends with '\r' or '\n')
  4. Store in sensor_data struct (mutex-protected)
  5. Sleep 100ms
```

**API Used:**
- `gnss_reader_init()` - Configure USART6 @ 115200 baud
- `gnss_reader_read_nmea()` - Non-blocking NMEA sentence read
- `gnss_reader_get_default_sentence()` - Get placeholder sentence

**Hardware:**
- USART6 TX: PG_14 (Arduino D1)
- USART6 RX: PG_9 (Arduino D0)
- Baud Rate: 115200
- Protocol: NMEA 0183
- Receiver: SimpleRTK2b u-blox F9P

**NMEA Sentences (Examples):**
- `$GPRMC` - Recommended Minimum Navigation Information
- `$GPGGA` - Global Positioning System Fix Data
- `$GPGSA` - GPS DOP and Active Satellites
- `$GPGSV` - GPS Satellites in View

### Main Loop (4 Hz)

**Polling Period:** 250ms  
**Priority:** Normal (ROS2 spin)

```
Loop:
  1. Lock mutex, read all sensor_data
  2. Build MainSensData message with all sensor values
  3. Publish to ROS2 topic "tp_sensdata_d6"
  4. Print console debug information
  5. Print GNSS every 1 second (throttled)
  6. Sleep 250ms
```

**Published Topic:** `tp_sensdata_d6` (MainSensData)

**Message Fields:**
- `mt_lf_encode_msg` - Left encoder count (int32_t)
- `mt_rt_encode_msg` - Right encoder count (int32_t)
- `sys_volt_msg` - Bus voltage in Volts (float)
- `sys_current_msg` - System current in Amps (float)

---

## Shared Data Structure

```cpp
struct {
    int32_t encoder_A;              // Updated by encoder_read_task @ 10Hz
    int32_t encoder_B;              // Updated by encoder_read_task @ 10Hz
    float bus_voltage;              // Updated by power_monitor_task @ 5Hz
    float current;                  // Updated by power_monitor_task @ 5Hz
    char nmea_sentence[256];        // Updated by gnss_reader_task @ 10Hz
} sensor_data;

Mutex sensor_data_mutex;            // Protects all access
```

All access to `sensor_data` is protected by `sensor_data_mutex` to prevent race conditions between task writers and main loop reader.

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

- [ ] Build completes without errors
- [ ] Binary size ~384 KB (appropriate for STM32 flash)
- [ ] Serial console shows all 3 tasks launching
- [ ] Encoder counts increment/decrement as wheels turn
- [ ] Bus voltage readings appear stable (11-14 V typical)
- [ ] Current readings vary with motor load (0-5 A typical)
- [ ] GNSS displays valid NMEA sentences every 1 second
- [ ] ROS2 topic `tp_sensdata_d6` publishes at 4 Hz
- [ ] No mutex deadlocks observed in console output

---

## Modular Design Benefits

✅ **Reusability** - Each sensor module can be used in other projects  
✅ **Testability** - Individual modules can be unit tested in isolation  
✅ **Maintainability** - Sensor logic is encapsulated and independent  
✅ **Scalability** - Easy to add new sensors or remove existing ones  
✅ **Clarity** - Clear separation of concerns between modules  

---

## Future Enhancements

- [ ] NMEA sentence parsing (extract latitude, longitude, altitude)
- [ ] Encoder error detection (stuck encoder detection)
- [ ] Power monitor alerts (low battery, overcurrent)
- [ ] Sensor fusion (combine GNSS + encoder + IMU)
- [ ] Configuration file for hardware parameters
- [ ] Self-tests on startup
- [ ] Performance timing annotations

---

## References

- [Mbed OS 6 Documentation](https://github.com/ARMmbed/mbed-os)
- [mROS 2 GitHub](https://github.com/mROS-base/mros2)
- [SimpleRTK2b Product Page](https://www.u-blox.com/en/product/simplertk2b)
- [STM32F767 Datasheet](https://www.st.com/en/microcontrollers-microprocessors/stm32f767ii.html)

---

**Last Updated:** November 4, 2025  
**Architecture Version:** 2.0 (Modular)  
**Previous Version:** 1.0 (Monolithic app.cpp)
