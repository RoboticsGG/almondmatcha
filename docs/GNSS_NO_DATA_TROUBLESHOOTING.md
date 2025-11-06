# GNSS USART6 No Data Received - Troubleshooting Guide

## Current Status

- ✅ Wiring verified correct
- ✅ SimpleRTK2b configured: 115200 baud, 8N1, LSB first
- ✅ NMEA output enabled: GxGGA, GxRMC, GxGST
- ❌ STM32 receives 0 bytes (Read attempts: 540+, Bytes received: 0)

## Possible Root Causes

### 1. **Wrong USART Pins** (Most Likely)

The current configuration uses:
- `PG_14` = USART6_TX
- `PG_9` = USART6_RX

**Problem**: These pins may not be properly configured as USART6 on the NUCLEO-F767ZI, or the alternate function mapping is incorrect.

### 2. **STM32F767ZI USART6 Pin Options**

According to the datasheet, USART6 supports multiple pin mappings:

| Option | TX Pin | RX Pin | Location | Notes |
|--------|--------|--------|----------|-------|
| 1 | PC_6 | PC_7 | Morpho | Default USART6 pins |
| 2 | PG_14 | PG_9 | Arduino D1/D0 | AF8 alternate function |

**Current code uses Option 2 (PG_14/PG_9)**

### 3. **Alternate Function Not Configured**

The Mbed `UnbufferedSerial` may not automatically configure the alternate function (AF8) for USART6 on these pins.

## Solutions to Try

### Solution 1: Try Default USART6 Pins (PC_6/PC_7)

**Edit `gnss_reader.cpp`:**

Change line 28:
```cpp
// FROM:
static UnbufferedSerial gnss_serial(PG_14, PG_9, GNSS_USART_BAUD_RATE);

// TO:
static UnbufferedSerial gnss_serial(PC_6, PC_7, GNSS_USART_BAUD_RATE);
```

**Rewire SimpleRTK2b:**
- SimpleRTK2b TX → STM32 PC_7 (Morpho connector CN11 pin 1)
- SimpleRTK2b RX → STM32 PC_6 (Morpho connector CN12 pin 4)
- SimpleRTK2b GND → STM32 GND

**Rebuild and flash:**
```bash
cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash all NUCLEO_F767ZI sensors_node
cp build/mros2-mbed.bin /run/media/$USER/MBED/
sync
```

---

### Solution 2: Use Different USART (USART3 - Easier Access)

USART3 is available on easy-to-access Arduino header pins:
- `PD_8` = USART3_TX (CN9 pin 6)
- `PD_9` = USART3_RX (CN9 pin 4)

**Edit `gnss_reader.cpp`:**

Change line 28:
```cpp
// FROM:
static UnbufferedSerial gnss_serial(PG_14, PG_9, GNSS_USART_BAUD_RATE);

// TO:
static UnbufferedSerial gnss_serial(PD_8, PD_9, GNSS_USART_BAUD_RATE);
```

**Rewire SimpleRTK2b:**
- SimpleRTK2b TX → STM32 PD_9 (Arduino D15 / CN9 pin 4)
- SimpleRTK2b RX → STM32 PD_8 (Arduino D14 / CN9 pin 6)
- SimpleRTK2b GND → STM32 GND

---

### Solution 3: Loopback Test

Test if the USART hardware is working at all by connecting TX to RX on the STM32:

1. **Short circuit test:**
   - Connect STM32 PG_14 → PG_9 (or whatever pins you're using)
   - This creates a loopback: everything transmitted is immediately received

2. **Check serial console:**
   ```
   [GNSS] Step 3: Testing serial interface...
   [GNSS]   - Sent test byte 0x55, write() returned: 1
   [GNSS]   - WARNING: Data readable immediately (loopback?)
   [GNSS]   - Read back: 0x55
   ```

3. **If loopback works:**
   - USART hardware is OK
   - Problem is with SimpleRTK2b connection or configuration

4. **If loopback fails:**
   - USART pins not configured correctly
   - Try different pins (Solution 1 or 2)

---

### Solution 4: Use UART1 (Serial Console Alternative)

If you can temporarily disconnect the USB serial console, use UART1:
- `PA_9` = USART1_TX
- `PA_10` = USART1_RX

These are the default serial pins and guaranteed to work.

---

## Nucleo-F767ZI Pinout Reference

### Arduino Header Pins (Easy Access)
| Arduino Pin | STM32 Pin | Available USART |
|-------------|-----------|-----------------|
| D0 | PG_9 | USART6_RX (AF8) |
| D1 | PG_14 | USART6_TX (AF8) |
| D14 | PD_8 | USART3_TX |
| D15 | PD_9 | USART3_RX |

### Morpho Connector (Harder Access)
| Morpho | STM32 Pin | USART |
|--------|-----------|-------|
| CN11-1 | PC_7 | USART6_RX |
| CN12-4 | PC_6 | USART6_TX |

---

## Quick Test Procedure

### Test with PC_6/PC_7 (Recommended First Try)

```bash
# 1. Edit gnss_reader.cpp line 28:
nano ~/almondmatcha/mros2-mbed-sensors-gnss/workspace/sensors_node/gnss_reader.cpp

# Change to:
# static UnbufferedSerial gnss_serial(PC_6, PC_7, GNSS_USART_BAUD_RATE);

# 2. Rebuild
cd ~/almondmatcha/mros2-mbed-sensors-gnss
sudo ./build.bash all NUCLEO_F767ZI sensors_node

# 3. Flash
cp build/mros2-mbed.bin /run/media/$USER/MBED/
sync

# 4. Rewire:
#    SimpleRTK2b TX -> STM32 PC_7 (Morpho CN11 pin 1)
#    SimpleRTK2b RX -> STM32 PC_6 (Morpho CN12 pin 4)
#    GND -> GND

# 5. Open serial console
minicom -b 115200 -D /dev/ttyACM0

# 6. Look for:
#    [GNSS] Received NMEA #1 (XX bytes): $GPGGA,...
```

---

## Expected Output After Fix

```
[GNSS] ===== USART6 Configuration Complete =====
[GNSS] TX:   PC_6 (or PD_8 or PG_14 depending on solution)
[GNSS] RX:   PC_7 (or PD_9 or PG_9 depending on solution)
[GNSS] Baud: 115200 bps
[GNSS] Ready to receive NMEA sentences
[GNSS] Starting reception test - waiting for data...
[GNSS] Received NMEA #1 (78 bytes): $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
[GNSS] Received NMEA #2 (68 bytes): $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
[GNSS-MAIN] Current NMEA: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
```

---

## If Still No Data

1. **Verify SimpleRTK2b is transmitting:**
   - Connect SimpleRTK2b directly to PC via USB-UART adapter
   - Use minicom/screen on PC: `screen /dev/ttyUSB0 115200`
   - Should see continuous NMEA sentences

2. **Check voltage levels:**
   - SimpleRTK2b TX should be 3.3V logic
   - STM32 RX expects 3.3V logic
   - Use multimeter or logic analyzer to verify

3. **Try logic analyzer:**
   - Connect to SimpleRTK2b TX line
   - Verify baud rate is actually 115200
   - Verify data format is 8N1
   - Verify NMEA sentences are being transmitted

---

## Summary

**Most likely solution:** Change pins from `PG_14/PG_9` to `PC_6/PC_7` (default USART6 pins).

**If that doesn't work:** Try `PD_8/PD_9` (USART3).

**If nothing works:** There may be a hardware issue with the SimpleRTK2b or STM32 board.

---

**Last Updated:** 2025-01-06  
**Status:** 🔍 Debugging in progress  
**Next Step:** Try PC_6/PC_7 pins for USART6
