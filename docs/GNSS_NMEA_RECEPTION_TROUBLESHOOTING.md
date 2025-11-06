# GNSS NMEA Reception Troubleshooting Guide

## Problem

The GNSS reader is showing only the default NMEA string and never receives any messages from the SimpleRTK2b GNSS receiver:

```
[GNSS-MAIN] Current NMEA: $GNRMC,,,,,,,,,,,N*71
```

This indicates the USART6 serial port is not actually receiving data from the receiver.

## Diagnostic Steps

### Step 1: Check the Status Counter

Flash the new firmware and observe the serial output. You should see:

```
[GNSS-STATUS] Read attempts: 10, Sentences received: 0
[GNSS-STATUS] Read attempts: 20, Sentences received: 0
[GNSS-STATUS] Read attempts: 30, Sentences received: 0
```

**What it means:**
- **Attempts increasing**: GNSS reader task is running ✅
- **Sentences = 0**: USART6 is NOT receiving data ❌

### Step 2: Verify Physical Connections

Check your wiring carefully:

```
NUCLEO-F767ZI            SimpleRTK2b Debug Port
-----------------------------------------
PG_9  (D0)  ← ← ← ← ← TX (D1)
PG_14 (D1)  → → → → → RX (D0)
GND   (GND) ← ← ← ← ← GND
3V3   (3V3) ← ← ← ← ← VCC
```

**Common mistakes:**
- TX/RX crossed (not matching above)
- Loose or missing GND connection
- Antenna not connected to SimpleRTK2b
- SimpleRTK2b not powered properly

### Step 3: Verify SimpleRTK2b is Transmitting

Connect the SimpleRTK2b directly to your PC with a USB-UART adapter to verify it's actually sending NMEA:

```bash
screen /dev/ttyUSB0 115200
```

You should see NMEA sentences like:
```
$GPRMC,123519,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
$GPGGA,123520,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
```

If you don't see anything, the receiver is NOT transmitting. Check:
- Power supply (should have solid red LED)
- Antenna connection
- Receiver configuration (baud rate, serial output)

### Step 4: Verify Baud Rate

The code uses **115200 baud** for USART6. Check if SimpleRTK2b is configured for 115200:

```bash
# Check SimpleRTK2b configuration via web interface:
# http://192.168.0.2  (if on same network)
# Or use RTK settings app
```

If SimpleRTK2b is at different baud rate (e.g., 9600), change it to 115200.

### Step 5: Check USART6 Pin Mapping

The Arduino D0/D1 pins map to:
- **D0 (PG_9)**: USART6_RX - Receives data from SimpleRTK2b TX
- **D1 (PG_14)**: USART6_TX - Transmits to SimpleRTK2b RX

Verify on the Nucleo-F767ZI pinout diagram.

### Step 6: Try Alternative Pin Configuration

If nothing works, the pin mapping might be different than expected. Try swapping the RX/TX pins by modifying `gnss_reader.cpp`:

```cpp
// Current configuration (lines 20-21):
static UnbufferedSerial gnss_serial(PG_14, PG_9, GNSS_USART_BAUD_RATE);

// Try swapping (UnbufferedSerial(TX, RX, baud)):
// static UnbufferedSerial gnss_serial(PG_9, PG_14, GNSS_USART_BAUD_RATE);
```

If this fixes it, you've found a pin mapping issue!

## Diagnostic Output Interpretation

### Expected Output (Everything Working)

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
[GNSS] Starting reception test - waiting for data...

[GNSS] GNSS reader task started
[GNSS-STATUS] Read attempts: 10, Sentences received: 0
[GNSS-STATUS] Read attempts: 20, Sentences received: 1
[GNSS] Received NMEA #1 (78 bytes): $GPRMC,123519,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
[GNSS-STATUS] Read attempts: 30, Sentences received: 2
[GNSS] Received NMEA #2 (78 bytes): $GPRMC,123520,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6B
```

### Problem Output (USART Not Receiving)

```
[GNSS] ===== USART6 Initialization Starting =====
...configuration messages...
[GNSS] GNSS reader task started
[GNSS-STATUS] Read attempts: 10, Sentences received: 0
[GNSS-STATUS] Read attempts: 20, Sentences received: 0
[GNSS-STATUS] Read attempts: 30, Sentences received: 0
[GNSS-STATUS] Read attempts: 40, Sentences received: 0
```

**Action:**
- Counter keeps incrementing = task is running
- Sentences always 0 = USART not receiving data
- Check physical connections (Step 2)
- Verify SimpleRTK2b is transmitting (Step 3)
- Try alternative pin configuration (Step 6)

## Checklist

- [ ] USART6 initialization messages appear
- [ ] Status counter increments (confirms task running)
- [ ] SimpleRTK2b is powered (check LEDs)
- [ ] SimpleRTK2b antenna is connected
- [ ] TX/RX/GND wired correctly
- [ ] Baud rate is 115200 on SimpleRTK2b
- [ ] Serial cable is not damaged
- [ ] Try alternative USART port if available
- [ ] Verify SimpleRTK2b transmits when connected to PC directly

## Hardware Alternatives

If USART6 won't work, other USART ports available on NUCLEO-F767ZI:

| USART | TX Pin | RX Pin  | Available |
|-------|--------|---------|-----------|
| USART1 | PA_9  | PA_10  | Yes (needs reconfiguration) |
| USART2 | PA_2  | PA_3   | Yes (used for serial printf) |
| USART3 | PC_10 | PC_11  | Yes |
| USART6 | PG_14 | PG_9   | Yes (currently using) |

To use different USART, modify `gnss_reader.cpp`:

```cpp
// Example: Use USART1 instead
static UnbufferedSerial gnss_serial(PA_9, PA_10, GNSS_USART_BAUD_RATE);
```

## Network Diagnostics

If using Ethernet between STM32 and SimpleRTK2b, check:

```bash
# Can you ping the SimpleRTK2b from rover PC?
ping 192.168.0.2  # (adjust IP as needed)

# Check network connectivity
arp -a | grep 192.168

# Monitor network traffic
sudo tcpdump -i eth0 port 9000  # or whatever RTK port
```

## Advanced Debugging

To enable character-by-character hex logging, uncomment in `gnss_reader.cpp`:

```cpp
// In gnss_reader_read_nmea() function:
// Debug: Print every character received
printf("[GNSS-BYTE] 0x%02X ('%c')\r\n", (unsigned char)ch, (ch >= 32 && ch < 127) ? ch : '?');
```

This will show:

```
[GNSS-BYTE] 0x24 ('$')  ← Start of sentence
[GNSS-BYTE] 0x47 ('G')
[GNSS-BYTE] 0x50 ('P')
[GNSS-BYTE] 0x52 ('R')
...
[GNSS-BYTE] 0x0D ('\r')  ← End of sentence
[GNSS-BYTE] 0x0A ('\n')
```

If you see hex bytes but only repeated garbage, likely a **baud rate mismatch**.

## References

- **NUCLEO-F767ZI Pinout**: https://www.st.com/resource/en/user_manual/dm00244518-nucleo-144-boards-stmicroelectronics.pdf
- **STM32F767ZI USART**: https://www.st.com/resource/en/datasheet/stm32f767zi.pdf (USART section)
- **SimpleRTK2b Serial Protocol**: https://docs.emlid.com/rtk2b/
- **Mbed UnbufferedSerial**: https://os.mbed.com/docs/mbed-os/latest/apis/serial.html

---

**Last Updated:** 2025-01-06  
**Status:** Diagnostic tools added - follow flowchart above to troubleshoot
