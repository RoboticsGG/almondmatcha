/**
 * @file gnss_reader.cpp
 * @brief SimpleRTK2b GNSS reader implementation using Mbed OS BufferedSerial
 *
 * Pin Configuration for Arduino Shield Mount:
 * - Arduino D0 (PG_9)  = USART6_RX (receives data FROM SimpleRTK2b UART1)
 * - Arduino D1 (PG_14) = USART6_TX (sends data TO SimpleRTK2b UART1)
 *
 * NOTE: Uses BufferedSerial (not UnbufferedSerial) so that bytes arriving
 * during the 500 ms task sleep are stored in the interrupt-driven ring
 * buffer and not lost.  The serial object is created inside gnss_reader_init()
 * (after main() / RTOS started) to avoid static-init-order issues.
 */

#include "gnss_reader.h"
#include "mbed.h"
#include <cstring>

// ============================================================================
// SERIAL INTERFACE & BUFFERS
// ============================================================================

/**
 * @brief USART6 BufferedSerial instance (interrupt-driven RX ring buffer).
 * Allocated in gnss_reader_init() — pointer is NULL until then.
 *
 * Arduino Shield:
 *   TX = PG_14 (D1), RX = PG_9 (D0), AF8 = USART6
 * Constructor: BufferedSerial(TX, RX, baud)
 */
static BufferedSerial* gnss_serial = nullptr;

/** @brief Internal NMEA sentence assembly buffer */
static char nmea_buffer[GNSS_NMEA_BUFFER_SIZE];

/** @brief Current write position in nmea_buffer */
static size_t nmea_buffer_index = 0;

/** @brief Default NMEA sentence (RMC format with no fix) */
static const char* default_nmea = "$GNRMC,,,,,,,,,,,N*71";

// ============================================================================
// GNSS INITIALIZATION
// ============================================================================

void gnss_reader_init() {
    // Create BufferedSerial here (after RTOS is running) so the interrupt
    // handler is registered cleanly.  Mbed's pinmap already configures
    // PG_14/PG_9 as USART6 AF8 — no manual HAL GPIO setup needed.
    gnss_serial = new BufferedSerial(PG_14, PG_9, GNSS_USART_BAUD_RATE);

    // Non-blocking reads: gnss_serial->read() returns -EAGAIN when the
    // ring buffer is empty, allowing the polling loop to exit cleanly.
    gnss_serial->set_blocking(false);

    gnss_serial->set_format(
        /* bits */      8,
        /* parity */    SerialBase::None,
        /* stop bits */ 1
    );

    printf("[GNSS] USART6 BufferedSerial on D0/D1 (PG9/PG14), 115200 8N1\r\n");

    // Flush any garbage bytes from line-capacitance on startup
    osDelay(200);
    {
        uint8_t discard;
        while (gnss_serial->read(&discard, 1) > 0) { /* drain */ }
    }
}

// ============================================================================
// NMEA SENTENCE READING IMPLEMENTATION
// ============================================================================

size_t gnss_reader_read_nmea(char* output_buffer, size_t buffer_size) {
    if (output_buffer == NULL || buffer_size == 0 || gnss_serial == nullptr) {
        return 0;
    }

    // Drain all bytes currently in the BufferedSerial ring buffer.
    // Because set_blocking(false) is set, read() returns -EAGAIN (negative)
    // when the buffer is empty, so the loop exits cleanly.
    static const int MAX_READ_ITERATIONS = 512;
    int iterations = 0;

    while (iterations++ < MAX_READ_ITERATIONS) {
        uint8_t ch_buffer;
        ssize_t read_result = gnss_serial->read(&ch_buffer, 1);
        
        if (read_result <= 0) {
            // No more data available
            break;
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
