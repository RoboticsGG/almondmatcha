/**
 * @file encoder_control.h
 * @brief Quadrature encoder reader interface for wheel speed feedback
 * 
 * Provides high-level API for reading dual quadrature encoders (encoder A and B)
 * using interrupt-driven GPIO handlers. Encoders are polled by the encoder task
 * at 10 Hz and counts are stored in a shared data structure.
 * 
 * Thread Safety: Uses volatile atomics for counter updates from ISRs.
 * 
 * Hardware:
 * - Encoder A: PA_15 (Channel A), PB_5 (Channel B)
 * - Encoder B: PB_3 (Channel A), PB_4 (Channel B)
 */

#ifndef ENCODER_CONTROL_H
#define ENCODER_CONTROL_H

#include <cstdint>

// ============================================================================
// ENCODER PIN DEFINITIONS (Quadrature Encoder Inputs)
// ============================================================================

/** @brief Encoder A - Channel A input (PA_15) */
extern volatile int32_t encoder_a_count;

/** @brief Encoder B - Channel B input (PB_3) */
extern volatile int32_t encoder_b_count;

// ============================================================================
// ENCODER INITIALIZATION & INTERRUPT HANDLERS
// ============================================================================

/**
 * @brief Initialize encoder interrupt handlers
 * 
 * Attaches rise/fall edge handlers to both encoder pins.
 * Must be called once during initialization before encoder task starts.
 * 
 * Quadrature Logic:
 * - Rising edge on A: increment if B is LOW, decrement if B is HIGH
 * - Falling edge on A: increment if B is HIGH, decrement if B is LOW
 */
void encoder_init();

/**
 * @brief Get current count for encoder A
 * 
 * Returns the volatile counter updated by interrupt handlers.
 * Safe to call from any context (reading volatile atomic).
 * 
 * @return Encoder A total count (signed)
 */
int32_t encoder_get_count_a();

/**
 * @brief Get current count for encoder B
 * 
 * Returns the volatile counter updated by interrupt handlers.
 * Safe to call from any context (reading volatile atomic).
 * 
 * @return Encoder B total count (signed)
 */
int32_t encoder_get_count_b();

/**
 * @brief Reset encoder A counter to zero
 * 
 * Atomically sets the encoder A count to zero.
 * Useful for odometry reset or testing.
 */
void encoder_reset_count_a();

/**
 * @brief Reset encoder B counter to zero
 * 
 * Atomically sets the encoder B count to zero.
 * Useful for odometry reset or testing.
 */
void encoder_reset_count_b();

#endif  // ENCODER_CONTROL_H
