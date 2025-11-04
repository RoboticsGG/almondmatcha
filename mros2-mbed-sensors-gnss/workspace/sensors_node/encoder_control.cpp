/**
 * @file encoder_control.cpp
 * @brief Quadrature encoder implementation using Mbed OS InterruptIn
 */

#include "encoder_control.h"
#include "mbed.h"

// ============================================================================
// ENCODER PIN DEFINITIONS - QUADRATURE INPUTS
// ============================================================================

InterruptIn enc_a_ch_a(PA_15);  // Encoder A - Channel A
InterruptIn enc_a_ch_b(PB_5);   // Encoder A - Channel B

InterruptIn enc_b_ch_a(PB_3);   // Encoder B - Channel A
InterruptIn enc_b_ch_b(PB_4);   // Encoder B - Channel B

// ============================================================================
// VOLATILE ENCODER COUNTERS
// ============================================================================

/** @brief Total encoder A count updated by interrupt handlers */
volatile int32_t encoder_a_count = 0;

/** @brief Total encoder B count updated by interrupt handlers */
volatile int32_t encoder_b_count = 0;

// ============================================================================
// ENCODER INTERRUPT HANDLERS
// ============================================================================

/**
 * @brief Encoder A rising edge handler
 * 
 * Quadrature logic:
 * - If channel B is LOW: forward direction -> increment
 * - If channel B is HIGH: reverse direction -> decrement
 */
static void encoder_a_rise_handler() {
    encoder_a_count += (enc_a_ch_b.read() == 0) ? 1 : -1;
}

/**
 * @brief Encoder A falling edge handler
 * 
 * Quadrature logic:
 * - If channel B is HIGH: forward direction -> increment
 * - If channel B is LOW: reverse direction -> decrement
 */
static void encoder_a_fall_handler() {
    encoder_a_count += (enc_a_ch_b.read() == 1) ? 1 : -1;
}

/**
 * @brief Encoder B rising edge handler
 * 
 * Quadrature logic:
 * - If channel B is LOW: forward direction -> increment
 * - If channel B is HIGH: reverse direction -> decrement
 */
static void encoder_b_rise_handler() {
    encoder_b_count += (enc_b_ch_b.read() == 0) ? 1 : -1;
}

/**
 * @brief Encoder B falling edge handler
 * 
 * Quadrature logic:
 * - If channel B is HIGH: forward direction -> increment
 * - If channel B is LOW: reverse direction -> decrement
 */
static void encoder_b_fall_handler() {
    encoder_b_count += (enc_b_ch_b.read() == 1) ? 1 : -1;
}

// ============================================================================
// ENCODER API IMPLEMENTATION
// ============================================================================

void encoder_init() {
    // Attach rise/fall edge handlers to encoder A
    enc_a_ch_a.rise(&encoder_a_rise_handler);
    enc_a_ch_a.fall(&encoder_a_fall_handler);
    
    // Attach rise/fall edge handlers to encoder B
    enc_b_ch_a.rise(&encoder_b_rise_handler);
    enc_b_ch_a.fall(&encoder_b_fall_handler);
}

int32_t encoder_get_count_a() {
    return encoder_a_count;
}

int32_t encoder_get_count_b() {
    return encoder_b_count;
}

void encoder_reset_count_a() {
    encoder_a_count = 0;
}

void encoder_reset_count_b() {
    encoder_b_count = 0;
}
