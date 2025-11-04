/**
 * @file gnss_reader.h
 * @brief SimpleRTK2b GNSS/RTK receiver NMEA sentence reader interface
 * 
 * Provides high-level API for reading NMEA sentences from a SimpleRTK2b
 * GNSS receiver connected via USART6 serial port. Implements buffering
 * and line-oriented parsing. Sentences are read by the GNSS task at 10 Hz.
 * 
 * Thread Safety: Uses internal static buffers. Caller must protect against
 * concurrent reads via external synchronization (e.g., mutex in app.cpp).
 * 
 * Hardware:
 * - USART6 TX: PG_14 (Arduino D1)
 * - USART6 RX: PG_9 (Arduino D0)
 * - Baud Rate: 115200
 * 
 * NMEA Protocol:
 * - Start: '$'
 * - End: '\r' or '\n'
 * - Common Sentences:
 *   - $GPRMC - Recommended Minimum Navigation Information
 *   - $GPGGA - Global Positioning System Fix Data
 *   - $GPGSA - GPS DOP and Active Satellites
 *   - $GPGSV - GPS Satellites in View
 */

#ifndef GNSS_READER_H
#define GNSS_READER_H

#include <cstddef>

// ============================================================================
// GNSS CONFIGURATION
// ============================================================================

/** @brief NMEA sentence maximum buffer size */
#define GNSS_NMEA_BUFFER_SIZE 256

/** @brief USART6 baud rate for SimpleRTK2b */
#define GNSS_USART_BAUD_RATE 115200

// ============================================================================
// GNSS INITIALIZATION
// ============================================================================

/**
 * @brief Initialize GNSS serial interface (USART6)
 * 
 * Configures USART6 (PG_14=TX, PG_9=RX) at 115200 baud.
 * Should be called once during initialization.
 */
void gnss_reader_init();

// ============================================================================
// NMEA SENTENCE READING
// ============================================================================

/**
 * @brief Read a complete NMEA sentence from the GNSS receiver
 * 
 * Performs non-blocking read of available serial data and buffers
 * characters until a complete NMEA sentence is detected.
 * 
 * NMEA format:
 * - Starts with '$'
 * - Ends with '\r' or '\n'
 * - Maximum length: GNSS_NMEA_BUFFER_SIZE
 * 
 * @param[out] output_buffer Pointer to buffer for output NMEA sentence
 * @param buffer_size Size of output buffer (should be at least GNSS_NMEA_BUFFER_SIZE)
 * 
 * @return Length of NMEA sentence (0 if no complete sentence yet)
 * 
 * @note Returns 0 when called but no complete sentence is available.
 *       Caller must call this repeatedly to accumulate data.
 */
size_t gnss_reader_read_nmea(char* output_buffer, size_t buffer_size);

/**
 * @brief Get the default placeholder NMEA sentence
 * 
 * Returns the default NMEA sentence used when no GNSS fix is available.
 * Useful for initialization or testing.
 * 
 * Default: "$GNRMC,,,,,,,,,,,N*71" (RMC sentence with no fix)
 * 
 * @return Pointer to static default NMEA string
 */
const char* gnss_reader_get_default_sentence();

// ============================================================================
// GNSS DATA PARSING HELPERS (Optional - for future enhancement)
// ============================================================================

/**
 * @brief Check if a NMEA sentence is valid
 * 
 * Performs basic validation:
 * - Starts with '$'
 * - Ends with valid sentence terminator
 * 
 * @param nmea_sentence NMEA string to validate
 * @return 1 if valid, 0 if invalid
 */
int gnss_reader_is_valid_sentence(const char* nmea_sentence);

#endif  // GNSS_READER_H
