/**
 * @file led_status.h
 * @brief Status LED control interface
 *
 * Provides a simple API for controlling the board status LED (LED1)
 * to indicate initialization progress and error conditions.
 */

#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <chrono>
#include <cstdint>

/* =====================================
 * Status LED Control Functions
 * ===================================== */

/**
 * Set status LED state
 *
 * @param on true=LED on, false=LED off
 *
 * Thread-safe for basic on/off control (DigitalOut::write is atomic for single bit).
 * Use this function during normal operation to indicate system state.
 */
void set_status_led(bool on);

/**
 * Blink status LED for error indication (BLOCKING)
 *
 * @param blink_count Number of blink cycles
 * @param blink_delay Delay between LED state changes (milliseconds)
 *
 * WARNING: This function BLOCKS the calling thread with sleep_for().
 * MUST ONLY be used at startup in main() before operational tasks begin.
 * NOT safe to call from operational tasks — will starve other tasks.
 *
 * Typical usage in error paths:
 *   if (error) { set_status_led(false); status_led_blink_blocking(5); return -1; }
 */
void status_led_blink_blocking(uint32_t blink_count, std::chrono::milliseconds blink_delay);

#endif // LED_STATUS_H
