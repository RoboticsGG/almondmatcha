/**
 * @file gnss_reader.cpp
 * @brief SimpleRTK2b GNSS reader implementation using Mbed OS UnbufferedSerial
 * 
 * Pin Configuration for Arduino Shield Mount:
 * - Arduino D0 (PG_9)  = USART6_RX (receives data FROM SimpleRTK2b)
 * - Arduino D1 (PG_14) = USART6_TX (sends data TO SimpleRTK2b)
 * 
 * NOTE: SimpleRTK2b as Arduino shield uses D0/D1, cannot use alternate pins
 */

#include "gnss_reader.h"
#include "mbed.h"
#include <cstring>

// Include STM32 HAL for direct GPIO configuration
#include "stm32f7xx_hal.h"

// ============================================================================
// SERIAL INTERFACE & BUFFERS
// ============================================================================

/** 
 * @brief USART6 serial port instance
 * Arduino Shield Configuration:
 * - D0 (PG_9)  = USART6_RX (AF8)
 * - D1 (PG_14) = USART6_TX (AF8)
 * 
 * Constructor order: (TX, RX, baudrate)
 * Try both TX,RX and RX,TX if one doesn't work
 */
static UnbufferedSerial gnss_serial(PG_14, PG_9, GNSS_USART_BAUD_RATE);
// Alternative if swapped: static UnbufferedSerial gnss_serial(PG_9, PG_14, GNSS_USART_BAUD_RATE);

/** @brief Internal NMEA sentence buffer */
static char nmea_buffer[GNSS_NMEA_BUFFER_SIZE];

/** @brief Current write position in nmea_buffer */
static size_t nmea_buffer_index = 0;

/** @brief Default NMEA sentence (RMC format with no fix) */
static const char* default_nmea = "$GNRMC,,,,,,,,,,,N*71";

// ============================================================================
// GNSS INITIALIZATION
// ============================================================================

/**
 * @brief Configure GPIO pins for USART6 using STM32 HAL
 * Must be called before gnss_serial operations
 */
static void configure_usart6_gpio() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO Port G clock
    __HAL_RCC_GPIOG_CLK_ENABLE();
    
    // Configure PG9 (USART6_RX) - Arduino D0
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        // Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // Enable pull-up resistor
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;   // AF8 for USART6
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    
    // Configure PG14 (USART6_TX) - Arduino D1
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        // Alternate Function Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;            // No pull-up needed for TX
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;   // AF8 for USART6
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

void gnss_reader_init() {
    // Configure GPIO pins using STM32 HAL (explicit AF8)
    configure_usart6_gpio();
    
    // Configure serial port
    gnss_serial.baud(GNSS_USART_BAUD_RATE);
    gnss_serial.format(
        /* bits */ 8,
        /* parity */ SerialBase::None,
        /* stop bits */ 1
    );
    
    printf("[GNSS] USART6 initialized on Arduino D0/D1 (115200 bps, 8N1)\r\n");
    
    // Give interface time to stabilize
    osDelay(100);
}

// ============================================================================
// NMEA SENTENCE READING IMPLEMENTATION
// ============================================================================

size_t gnss_reader_read_nmea(char* output_buffer, size_t buffer_size) {
    if (output_buffer == NULL || buffer_size == 0) {
        return 0;  // Invalid parameters
    }

    // Read all available data from serial port without checking readable() repeatedly
    // This prevents us from sleeping mid-sentence
    static const int MAX_READ_ITERATIONS = 200;  // Safety limit to prevent infinite loop
    int iterations = 0;
    
    while (iterations++ < MAX_READ_ITERATIONS) {
        // Try to read one byte
        uint8_t ch_buffer;
        ssize_t read_result = gnss_serial.read(&ch_buffer, 1);
        
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
