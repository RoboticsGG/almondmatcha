/**
 * @file gnss_reader.cpp
 * @brief SimpleRTK2b GNSS reader implementation using Mbed OS UnbufferedSerial
 * 
 * Pin Configuration:
 * OPTION 1 (Current): PG_14=TX, PG_9=RX (USART6 AF8)
 * OPTION 2 (Try if failing): PC6=TX, PC7=RX (USART6 AF8 - default pins)
 * 
 * NOTE: If PG_14/PG_9 don't work, try PC6/PC7 by changing the constructor below
 */

#include "gnss_reader.h"
#include "mbed.h"
#include <cstring>

// ============================================================================
// SERIAL INTERFACE & BUFFERS
// ============================================================================

/** 
 * @brief USART6 serial port instance
 * 
 * NUCLEO-F767ZI USART6 pin options:
 * Option 1: TX=PG_14 (D1), RX=PG_9  (D0) <- Arduino header pins
 * Option 2: TX=PC_6  (D51),RX=PC_7 (D52) <- Morpho connector
 * 
 * Currently using Option 1 (Arduino D0/D1 pins for easy access)
 * If this doesn't work, try Option 2 by changing to: UnbufferedSerial gnss_serial(PC_6, PC_7, ...)
 */
static UnbufferedSerial gnss_serial(PG_14, PG_9, GNSS_USART_BAUD_RATE);

/** @brief Internal NMEA sentence buffer */
static char nmea_buffer[GNSS_NMEA_BUFFER_SIZE];

/** @brief Current write position in nmea_buffer */
static size_t nmea_buffer_index = 0;

// ============================================================================
// DIAGNOSTIC COUNTERS
// ============================================================================

static uint32_t read_attempts = 0;
static uint32_t bytes_received_total = 0;
static uint32_t nmea_sentences_completed = 0;
static uint8_t last_raw_byte = 0;
static uint32_t consecutive_no_data = 0;

// Diagnostic counters
static volatile uint32_t gnss_total_read_calls = 0;
static volatile uint32_t gnss_total_readable_true = 0;
static volatile uint32_t gnss_total_bytes = 0;
static volatile uint32_t gnss_total_read_errors = 0;

/** @brief Default NMEA sentence (RMC format with no fix) */
static const char* default_nmea = "$GNRMC,,,,,,,,,,,N*71";

// ============================================================================
// GNSS INITIALIZATION
// ============================================================================

void gnss_reader_init() {
    printf("[GNSS] ===== USART6 Initialization Starting =====\r\n");
    
    // Step 1: Configure pins using DigitalInOut with alternate function
    // PG_9 = RX (with pull-up)
    // PG_14 = TX (standard output)
    printf("[GNSS] Step 1: Configuring GPIO pins...\r\n");
    
    // Create pin objects to configure them
    DigitalInOut rx_pin(PG_9);
    DigitalOut tx_pin(PG_14);
    
    // Set RX pin mode to pull-up
    rx_pin.mode(PullUp);
    printf("[GNSS]   - PG_9 (RX): Pull-up resistor ENABLED\r\n");
    
    printf("[GNSS]   - PG_14 (TX): Standard GPIO output\r\n");
    
    // Step 2: Configure serial port
    printf("[GNSS] Step 2: Configuring USART6 serial port...\r\n");
    gnss_serial.baud(GNSS_USART_BAUD_RATE);
    printf("[GNSS]   - Baud rate: %d bps\r\n", GNSS_USART_BAUD_RATE);
    
    // Step 3: Verify configuration
    printf("[GNSS] ===== USART6 Configuration Complete =====\r\n");
    printf("[GNSS] TX:   PG_14\r\n");
    printf("[GNSS] RX:   PG_9 (with Pull-Up)\r\n");
    printf("[GNSS] Baud: %d bps\r\n", GNSS_USART_BAUD_RATE);
    printf("[GNSS] Ready to receive NMEA sentences\r\n");
    
    // Give interface time to stabilize
    osDelay(100);
    
    printf("[GNSS] Starting reception test - waiting for data...\r\n");
}

// ============================================================================
// NMEA SENTENCE READING IMPLEMENTATION
// ============================================================================

size_t gnss_reader_read_nmea(char* output_buffer, size_t buffer_size) {
    if (output_buffer == NULL || buffer_size == 0) {
        return 0;  // Invalid parameters
    }
    gnss_total_read_calls++;

    // Check if serial port is readable
    if (!gnss_serial.readable()) {
        // Occasionally print status to help debugging (every 256 calls)
        if ((gnss_total_read_calls & 0xFF) == 0) {
            printf("[GNSS-DBG] read_calls=%lu readable=0 total_bytes=%lu errors=%lu\r\n",
                   (unsigned long)gnss_total_read_calls,
                   (unsigned long)gnss_total_bytes,
                   (unsigned long)gnss_total_read_errors);
        }
        return 0;  // No data available
    }

    // serial is readable
    gnss_total_readable_true++;

    // Read all available data from serial port
    uint32_t bytes_read = 0;
    while (gnss_serial.readable()) {
        uint8_t ch_buffer;
        ssize_t read_result = gnss_serial.read(&ch_buffer, 1);
        
        if (read_result <= 0) {
            gnss_total_read_errors++;
            // read returned 0 or negative; break to avoid busy-loop
            break;  // No data available or error
        }

        bytes_read++;
        gnss_total_bytes++;
        char ch = static_cast<char>(ch_buffer);
        
        // Debug: Print every character received to help diagnose
        printf("[GNSS-BYTE] #%lu: 0x%02X ('%c')\r\n", 
               (unsigned long)gnss_total_bytes,
               (unsigned char)ch, 
               (ch >= 32 && ch < 127) ? ch : '?');
        
        // Start of a new NMEA sentence
        if (ch == '$') {
            nmea_buffer_index = 0;
            nmea_buffer[nmea_buffer_index++] = ch;
        }
        // End of NMEA sentence (carriage return or line feed)
        else if ((ch == '\r' || ch == '\n') && nmea_buffer_index > 0) {
            // Add null terminator and prepare for output
            nmea_buffer[nmea_buffer_index] = '\0';
            
            // Copy to output buffer if it fits
            if (nmea_buffer_index < buffer_size) {
                strncpy(output_buffer, nmea_buffer, buffer_size - 1);
                output_buffer[buffer_size - 1] = '\0';  // Ensure null termination
                
                size_t result = nmea_buffer_index;
                nmea_buffer_index = 0;
                return result;  // Return length of complete NMEA sentence
            }
            
            nmea_buffer_index = 0;  // Reset on buffer overflow
        }
        // Regular character in NMEA sentence
        else if (nmea_buffer_index > 0 && nmea_buffer_index < GNSS_NMEA_BUFFER_SIZE - 1) {
            nmea_buffer[nmea_buffer_index++] = ch;
        }
    }
    // If we read bytes but didn't form a complete sentence, print a brief summary
    if (bytes_read > 0) {
        // Print first few bytes in hex to help identify data
        printf("[GNSS-DBG] bytes_read=%lu total_bytes=%lu\r\n",
               (unsigned long)bytes_read, (unsigned long)gnss_total_bytes);
        // Optionally print raw bytes (print at most 32 bytes)
        // (uncomment the loop below if you need detailed raw output)
        /*
        size_t preview = (bytes_read < 32) ? bytes_read : 32;
        for (size_t i = 0; i < preview; ++i) {
            printf("%02X ", (unsigned char)nmea_buffer[i]);
        }
        printf("\r\n");
        */
    }

    return 0;  // No complete NMEA sentence available yet
}

// ============================================================================
// GNSS HELPER FUNCTIONS
// ============================================================================

const char* gnss_reader_get_default_sentence() {
    return default_nmea;
}

int gnss_reader_is_valid_sentence(const char* nmea_sentence) {
    if (nmea_sentence == NULL) {
        return 0;  // NULL pointer
    }
    
    size_t len = strlen(nmea_sentence);
    
    // Must start with '$'
    if (len == 0 || nmea_sentence[0] != '$') {
        return 0;
    }
    
    // Must have at least 5 characters (e.g., "$GNRMC")
    if (len < 5) {
        return 0;
    }
    
    return 1;  // Valid sentence structure
}
