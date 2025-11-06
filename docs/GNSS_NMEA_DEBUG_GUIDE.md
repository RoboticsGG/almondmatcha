# GNSS NMEA Message Debugging Guide

This guide helps you diagnose why NMEA messages from the SimpleRTK2b GNSS receiver aren't being properly displayed on the STM32 board serial output.

## Quick Start

### 1. Build and Flash the Debug Firmware

```bash
# Build sensors node with debug output
cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash all NUCLEO_F767ZI sensors_node

# Copy binary to STM32 via USB mass storage
cp build/mros2-mbed.bin /run/media/$USER/MBED/
sync
```

### 2. Connect Serial Console

```bash
# After board resets, open minicom
minicom -b 115200 -D /dev/ttyACM0
```

Or use screen:
```bash
screen /dev/ttyACM0 115200
```

### 3. Power On and Observe

You should see output like:
```
[GNSS] USART6 initialized (PG_14=TX, PG_9=RX) at 115200 baud
[GNSS] Received NMEA #1 (78 bytes): $GPRMC,231812.00,4717.30319,N,00833.87878,E,0.1,11.96,230131,,,A,V*17
[GNSS] Received NMEA #2 (55 bytes): $GNRMC,231812.00,...
[LOOP 1] MotorA: 0 | MotorB: 0 | Vbus: 12.5 V | I: 2.3 A
[GNSS-MAIN] Current NMEA: $GPRMC,...
```

## Diagnostic Output Explained

### USART6 Initialization

```
[GNSS] USART6 initialized (PG_14=TX, PG_9=RX) at 115200 baud
```

**What it means:**
- USART6 is correctly configured on pins PG_14 (TX) and PG_9 (RX)
- Baud rate is set to 115200
- Serial interface is ready to receive data

**If you don't see this:** The gnss_reader_init() function failed to execute. Check if encoder_init() or power_monitor_init() crashed before reaching GNSS initialization.

---

### NMEA Reception

```
[GNSS] Received NMEA #1 (78 bytes): $GPRMC,231812.00,4717.30319,N,...
```

**What it means:**
- A complete NMEA sentence was received
- `#1` = this is the 1st NMEA sentence (counter increases per sentence)
- `78 bytes` = length of the sentence
- Full sentence is printed for inspection

**Common NMEA sentence types:**
- `$GPRMC` - RMC (Recommended Minimum Navigation Information)
- `$GPGGA` - GGA (Global Positioning System Fix Data)  
- `$GPGSA` - GSA (GPS DOP and Active Satellites)
- `$GPGSV` - GSV (GPS Satellites in View)
- `$GNRMC` - RMC (with GNSS prefix for multi-constellation)

**If you don't see NMEA messages:**

1. **Check GNSS receiver power:**
   ```bash
   # Monitor board power supply voltage
   # SimpleRTK2b usually needs 3.3V stable supply
   ```

2. **Check GNSS antenna connection:**
   - Is the u.FL antenna connected?
   - Is there a signal booster?

3. **Check USART6 physical connection:**
   - PG_14 (TX on STM32) → RX on SimpleRTK2b
   - PG_9 (RX on STM32) → TX on SimpleRTK2b
   - GND must be connected

4. **Check baud rate:**
   - Default SimpleRTK2b baud rate is 115200
   - If receiver is configured differently, it won't transmit properly
   - Try reconfiguring the receiver to 115200 baud

5. **Add raw serial debug:**
   
   Modify `gnss_reader_init()` to send a test message:
   ```cpp
   void gnss_reader_init() {
       gnss_serial.baud(GNSS_USART_BAUD_RATE);
       printf("[GNSS] USART6 initialized at %d baud\r\n", GNSS_USART_BAUD_RATE);
       
       // Send test command to GNSS receiver (optional)
       // gnss_serial.write("$\r\n", 3);  // Echo test
       
       // Give receiver time to respond
       osDelay(100);
   }
   ```

---

### Main Loop Status

```
[LOOP 1] MotorA: 0 | MotorB: 0 | Vbus: 12.5 V | I: 2.3 A
```

**What it means:**
- Main loop iteration #1
- Motor encoder counts (should increment if motors are running)
- Bus voltage (should be 12V+ for healthy power)
- System current (typical range 0.5-5A depending on motor load)

**If you don't see LOOP messages:** Main loop isn't running. Check if sensor tasks crashed during initialization.

---

### Periodic GNSS Display

```
[GNSS-MAIN] Current NMEA: $GPRMC,231812.00,4717.30319,N,...
```

**What it means:**
- Printed once per second (every 4 main loop iterations)
- Shows the last NMEA sentence received by GNSS task
- Updates when new sentences are received

**If GNSS is stuck at default:** No new NMEA data is being received. Troubleshoot GNSS receiver.

---

## Troubleshooting Flowchart

```
Do you see [GNSS] USART6 initialized?
├─ NO  → gnss_reader_init() not called
│         Check: Power? Crashes in encoder_init/power_monitor_init?
│
└─ YES → Do you see [GNSS] Received NMEA?
         ├─ NO  → GNSS receiver not sending data
         │        Check: Antenna connected? Power to receiver? USART pins?
         │        Try: Reconfigure receiver to 115200 baud
         │
         └─ YES → Do you see increasing NMEA count?
                  ├─ NO  → Same sentence repeated
                  │        Check: Receiver transmitting continuously?
                  │        Try: Power cycle receiver
                  │
                  └─ YES → Do you see good GPS fix?
                           Check NMEA sentence fields:
                           - $GPRMC,TIME,LAT,LON,SPEED,COURSE,DATE,MAG_VAR,MODE*CHECKSUM
                           - MODE = "A" (Active/good) or "V" (Void/no fix)
```

## Hardware Pinout

### STM32F767ZI USART6 (for GNSS)
| Signal | Pin     | Function       |
|--------|---------|----------------|
| TX     | PG_14   | Serial TX      |
| RX     | PG_9    | Serial RX      |
| GND    | GND     | Ground         |
| +3.3V  | 3V3     | GNSS Power     |

### SimpleRTK2b Connectors
| Connector | Pin | Function       |
|-----------|-----|----------------|
| Debug     | RX  | Serial RX      |
| Debug     | TX  | Serial TX      |
| Debug     | GND | Ground         |
| Power     | 3.3V| Power supply   |

---

## Manual GNSS Testing

If you want to verify the GNSS receiver without the STM32:

```bash
# Connect GNSS receiver directly to PC via USB-UART adapter
screen /dev/ttyUSB0 115200

# Should see continuous NMEA sentences like:
# $GPRMC,123519,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
# $GPRMC,123520,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6B
```

---

## Expected Behavior Over Time

### First 10 seconds (Startup)
```
[GNSS] USART6 initialized at 115200 baud
[GNSS] Waiting 6 seconds for DDS discovery...
[GNSS] Discovery wait complete
[GNSS] Encoder interrupt handlers configured
[GNSS] Power monitor I2C initialized
[GNSS] GNSS serial interface configured on USART6
[GNSS] Launching independent sensor tasks...
[GNSS] Encoder reader task launched
[GNSS] Power monitor task launched
[GNSS] GNSS reader task launched
[GNSS] ready to pub/sub message
```

### After 15 seconds (Normal Operation)
```
[LOOP 1] MotorA: 0 | MotorB: 0 | Vbus: 12.5 V | I: 2.3 A
[GNSS] Received NMEA #1 (78 bytes): $GPRMC,...
[LOOP 2] MotorA: 0 | MotorB: 0 | Vbus: 12.5 V | I: 2.3 A
[LOOP 3] MotorA: 0 | MotorB: 0 | Vbus: 12.5 V | I: 2.3 A
[LOOP 4] MotorA: 0 | MotorB: 0 | Vbus: 12.5 V | I: 2.3 A
[GNSS-MAIN] Current NMEA: $GPRMC,...
[GNSS] Received NMEA #2 (78 bytes): $GPRMC,...
```

---

## Advanced Debugging

### Add More Verbose GNSS Output

Edit `gnss_reader.cpp` to print raw bytes:

```cpp
size_t gnss_reader_read_nmea(char* output_buffer, size_t buffer_size) {
    if (output_buffer == NULL || buffer_size == 0) {
        return 0;
    }
    
    while (gnss_serial.readable()) {
        uint8_t ch_buffer;
        ssize_t read_result = gnss_serial.read(&ch_buffer, 1);
        
        if (read_result <= 0) break;
        
        char ch = static_cast<char>(ch_buffer);
        
        // Debug: Print every character received
        // printf("[GNSS-BYTE] 0x%02X ('%c')\r\n", ch, (ch >= 32 && ch < 127) ? ch : '?');
        
        // ... rest of parsing logic
    }
    
    return 0;
}
```

### Monitor Task Status

Add to main loop:

```cpp
// Check if GNSS task is making progress
static uint32_t last_loop_count = 0;
if (loop_count == last_loop_count) {
    printf("[WARN] Main loop not progressing!\r\n");
} else {
    last_loop_count = loop_count;
}
```

---

## References

- **SimpleRTK2b Documentation:** https://docs.emlid.com/rtk2b
- **NMEA Protocol:** https://www.trimble.com/oem/gpsoverview/
- **STM32F767ZI Datasheet:** https://www.st.com/resource/en/datasheet/stm32f767zi.pdf
- **Mbed Serial API:** https://os.mbed.com/docs/mbed-os/latest/apis/serial.html

---

**Last Updated:** 2025-01-06  
**Applies to:** mros2-mbed-sensors-gnss workspace  
**Status:** ✅ Debug output added to firmware
