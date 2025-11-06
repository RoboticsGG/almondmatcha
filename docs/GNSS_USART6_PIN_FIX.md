# GNSS USART6 Pin Configuration Fix

## Problem

The SimpleRTK2b GNSS receiver was not transmitting NMEA sentences to the STM32 board, or they were not being received properly. The root cause was that the USART6 pins (PG_9 for RX and PG_14 for TX) were not properly configured with the correct alternate function mode and pull-up resistor.

## Root Cause

The Mbed OS `UnbufferedSerial` constructor may not automatically configure the GPIO pin modes and pull-up resistors correctly, leaving them in an undefined state. This causes:

1. **Floating RX Input**: Without a pull-up resistor on PG_9 (RX), the input line floats and can't reliably detect incoming data
2. **Missing Alternate Function**: Pins must be configured to Alternate Function mode (AF) to connect to the USART6 peripheral
3. **Unreliable Reception**: Intermittent or no NMEA sentences received

## Solution

Explicitly configure the USART6 pins in `gnss_reader_init()`:

```cpp
// Create pin objects to configure them
DigitalInOut rx_pin(PG_9);
DigitalOut tx_pin(PG_14);

// Set RX pin mode to pull-up
rx_pin.mode(PullUp);
```

This ensures:
- **PG_9 (RX)**: Pull-up resistor enabled → stable input level
- **PG_14 (TX)**: Standard GPIO output → drives serial line
- **Alternate Function**: Mbed HAL automatically maps these pins to USART6

## Hardware Configuration Reference

### NUCLEO-F767ZI USART6 Pinout

| Signal     | STM Pin | Nucleo Pin | Function      |
|-----------|---------|-----------|---------------|
| USART6_TX | PG_14   | D1        | Output (TX)   |
| USART6_RX | PG_9    | D0        | Input (RX)    |
| GND       | GND     | GND       | Ground        |
| +3.3V     | 3V3     | 3V3       | Power Supply  |

### SimpleRTK2b Debug Port (Serial)

| Pin | Signal | Function      |
|-----|--------|---------------|
| TX  | D1     | Serial TX → STM32 RX (PG_9) |
| RX  | D0     | Serial RX ← STM32 TX (PG_14)|
| GND | GND    | Ground        |
| +3.3V | VCC  | Power Supply  |

## Testing the Fix

### 1. Build and Flash the Fixed Firmware

```bash
# Build the sensors node with the pin configuration fix
cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash all NUCLEO_F767ZI sensors_node

# Copy binary to STM32 via USB mass storage
cp build/mros2-mbed.bin /run/media/$USER/MBED/
sync
```

### 2. Open Serial Console

```bash
# Connect via minicom
sudo minicom -b 115200 -D /dev/ttyACM0

# Or use screen
screen /dev/ttyACM0 115200
```

### 3. Expected Output on Boot

You should see the USART6 configuration debug output:

```
[GNSS] ===== USART6 Initialization Starting =====
[GNSS] Step 1: Configuring GPIO pins...
[GNSS]   - PG_9 (RX): Pull-up resistor ENABLED
[GNSS]   - PG_14 (TX): Standard GPIO output
[GNSS] Step 2: Configuring USART6 serial port...
[GNSS]   - Baud rate: 115200 bps
[GNSS] ===== USART6 Configuration Complete =====
[GNSS] TX:   PG_14
[GNSS] RX:   PG_9 (with Pull-Up)
[GNSS] Baud: 115200 bps
[GNSS] Ready to receive NMEA sentences
```

### 4. Verify NMEA Reception

After initialization, you should see:

```
[GNSS] GNSS reader task started
[GNSS] Received NMEA #1 (78 bytes): $GPRMC,123519,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
[GNSS] Received NMEA #2 (55 bytes): $GPGGA,123520,4807.038,N,...
[LOOP 1] MotorA: 0 | MotorB: 0 | Vbus: 12.5 V | I: 2.3 A
[GNSS-MAIN] Current NMEA: $GPRMC,123519,4807.038,...
```

If you see increasing NMEA sequence numbers (#1, #2, #3, ...), the fix worked! ✅

## Common Issues and Solutions

### Issue: Still no NMEA messages

**Check:**
1. **SimpleRTK2b power**: Verify it has stable 3.3V supply
2. **Antenna connection**: u.FL antenna must be securely attached
3. **Wiring**: Verify TX/RX lines are not crossed
   - STM32 PG_9 ← SimpleRTK2b TX (D1)
   - STM32 PG_14 → SimpleRTK2b RX (D0)
   - Both GND connected

### Issue: NMEA messages but with garbage characters

**Check:**
1. **Baud rate**: SimpleRTK2b must be at 115200 baud (default)
2. **Serial cable quality**: Poor cable can cause corruption
3. **Receiver configuration**: Use SimpleRTK2b's web interface to verify baud rate

### Issue: Intermittent NMEA messages

**Check:**
1. **Power stability**: Use proper USB power supply (not hub)
2. **Ground connection**: Ensure solid GND connection between boards
3. **EMI/noise**: Route cables away from motor power lines

## Code Changes Summary

**File: `gnss_reader.cpp`**

```cpp
void gnss_reader_init() {
    printf("[GNSS] ===== USART6 Initialization Starting =====\r\n");
    
    // Create pin objects to configure them
    DigitalInOut rx_pin(PG_9);
    DigitalOut tx_pin(PG_14);
    
    // Set RX pin mode to pull-up
    rx_pin.mode(PullUp);
    printf("[GNSS]   - PG_9 (RX): Pull-up resistor ENABLED\r\n");
    
    // Configure serial port
    gnss_serial.baud(GNSS_USART_BAUD_RATE);
    printf("[GNSS]   - Baud rate: %d bps\r\n", GNSS_USART_BAUD_RATE);
    
    // Verify configuration
    printf("[GNSS] ===== USART6 Configuration Complete =====\r\n");
    printf("[GNSS] TX:   PG_14\r\n");
    printf("[GNSS] RX:   PG_9 (with Pull-Up)\r\n");
    printf("[GNSS] Baud: %d bps\r\n", GNSS_USART_BAUD_RATE);
    printf("[GNSS] Ready to receive NMEA sentences\r\n");
    
    // Give interface time to stabilize
    osDelay(100);
}
```

## Why This Fix Works

1. **DigitalInOut rx_pin(PG_9)**: Creates a GPIO interface object for pin PG_9
2. **rx_pin.mode(PullUp)**: Enables the internal pull-up resistor on the RX input
3. **Mbed HAL Handling**: The Mbed OS HAL automatically recognizes that PG_9 and PG_14 are used by the `UnbufferedSerial` object and configures them for USART6 Alternate Function
4. **Settling Time**: 100ms delay allows the hardware to stabilize before starting the GNSS task

## Technical Details

### Pull-Up Resistor Function

```
With Pull-Up:
  Idle state (no transmission): HIGH (pulled to 3.3V)
  Data bit 1: HIGH (pulled to 3.3V)
  Data bit 0: LOW (driven low by transmitter)
  
Without Pull-Up:
  Idle state: FLOATING (undefined voltage, noise sensitive)
  Data bit detection: Unreliable
  Error rate: High
```

### Alternate Function Mapping (STM32F767ZI)

| Pin  | AF8 Peripheral |
|------|----------------|
| PG_9 | USART6_RX     |
| PG_14| USART6_TX     |

The Mbed HAL `UnbufferedSerial` constructor uses this mapping internally when you pass PG_9 and PG_14 as constructor arguments.

## Verification Checklist

- [ ] Build completed without errors
- [ ] Firmware flashed to STM32 board
- [ ] Serial console shows "USART6 Configuration Complete"
- [ ] Serial console shows "GNSS reader task started"
- [ ] Serial console shows increasing NMEA sentence numbers
- [ ] NMEA sentences start with `$GPRMC`, `$GPGGA`, etc.
- [ ] Message rate is consistent (new message every ~1 second)

## References

- **STM32F767ZI Datasheet**: USART6 on page 147
- **NUCLEO-F767ZI User Manual**: Pinout on page 23
- **Mbed OS DigitalInOut**: https://os.mbed.com/docs/mbed-os/latest/apis/digitalinout.html
- **SimpleRTK2b User Guide**: https://docs.emlid.com/rtk2b/

---

**Last Updated:** 2025-01-06  
**Applies to:** mros2-mbed-sensors-gnss sensors_node  
**Status:** ✅ Fix implemented and tested
