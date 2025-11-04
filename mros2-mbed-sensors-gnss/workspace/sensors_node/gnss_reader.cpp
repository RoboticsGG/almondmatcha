/**
 * @file gnss_reader.cpp
 * @brief SimpleRTK2b GNSS reader implementation using Mbed OS UnbufferedSerial
 */

#include "gnss_reader.h"
#include "mbed.h"
#include <cstring>

// ============================================================================
// SERIAL INTERFACE & BUFFERS
// ============================================================================

/** @brief USART6 serial port instance (TX=PG_14, RX=PG_9) */
static UnbufferedSerial gnss_serial(PG_14, PG_9, GNSS_USART_BAUD_RATE);

/** @brief Internal NMEA sentence buffer */
static char nmea_buffer[GNSS_NMEA_BUFFER_SIZE];

/** @brief Current write position in nmea_buffer */
static size_t nmea_buffer_index = 0;

/** @brief Default NMEA sentence (RMC format with no fix) */
static const char* default_nmea = "$GNRMC,,,,,,,,,,,N*71";

// ============================================================================
// GNSS INITIALIZATION
// ============================================================================

void gnss_reader_init() {
    // Configure baud rate (should already be set in constructor)
    gnss_serial.baud(GNSS_USART_BAUD_RATE);
}

// ============================================================================
// NMEA SENTENCE READING IMPLEMENTATION
// ============================================================================

size_t gnss_reader_read_nmea(char* output_buffer, size_t buffer_size) {
    if (output_buffer == NULL || buffer_size == 0) {
        return 0;  // Invalid parameters
    }
    
    // Read all available data from serial port
    while (gnss_serial.readable()) {
        uint8_t ch_buffer;
        ssize_t read_result = gnss_serial.read(&ch_buffer, 1);
        
        if (read_result <= 0) {
            break;  // No data available or error
        }
        
        char ch = static_cast<char>(ch_buffer);
        
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
