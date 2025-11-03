/**
 * @file led_status.cpp
 * @brief Status LED implementation
 */

#include "mbed.h"
#include "led_status.h"
#include <chrono>

/* =====================================
 * Status LED Hardware
 * ===================================== */

/// Global status LED object (LED1 on STM32 Nucleo-F767ZI)
DigitalOut status_led(LED1);

/* =====================================
 * Status LED Control Function Implementations
 * ===================================== */

void set_status_led(bool on) {
    /** Set status LED state: true=on, false=off
     *  Note: Safe from any thread context (DigitalOut::write is atomic for single bit).
     */
    status_led = on ? 1 : 0;
}

void status_led_blink_blocking(uint32_t blink_count, std::chrono::milliseconds blink_delay) {
    /** Blink status LED for error indication (BLOCKING - use only at startup before tasks run).
     *  WARNING: Blocks calling thread with sleep_for(); not safe to call from operational tasks.
     *  Recommended: Use only in main() error paths or initialization code.
     */
    for (uint32_t i = 0; i < blink_count; i++) {
        set_status_led(true);
        ThisThread::sleep_for(blink_delay);
        set_status_led(false);
        ThisThread::sleep_for(blink_delay);
    }
}
