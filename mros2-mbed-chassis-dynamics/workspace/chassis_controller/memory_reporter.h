/**
 * @file memory_reporter.h
 * @brief Periodic heap/stack memory reporter for STM32 — single-domain POC
 *
 * Spawns a low-priority background thread that prints a JSON line to the USB
 * serial port every MEM_REPORT_INTERVAL_MS milliseconds.  Line format:
 *
 *   {"type":"STM32_MEM","node":"<NODE_NAME>","ts_ms":<uptime>,"heap_used":<bytes>,
 *    "heap_max":<bytes>,"heap_free":<bytes>,"alloc_fail":<count>,
 *    "stack_free":<bytes>}
 *
 * The ws_base serial collector (tools/stm32_serial/collect_stm32_memory.py)
 * filters lines that start with `{"type":"STM32_MEM"` and saves them to CSV.
 * Every other serial output from the STM32 is simply ignored by the collector —
 * no suppression is needed on the firmware side.
 *
 * Usage (minimal additions to app.cpp):
 *   1. #include "memory_reporter.h"          // top of app.cpp
 *   2. memory_reporter_start("node_name");    // after all other threads started
 */

#pragma once

#include "mbed.h"
#include "mbed_stats.h"

// 1000 ms gives 1 sample per 2 SPDP cycles (500 ms) — sufficient resolution
// for heap trending without contributing to serial/stdout mutex pressure.
#ifndef MEM_REPORT_INTERVAL_MS
#define MEM_REPORT_INTERVAL_MS 1000
#endif

// Stack for the reporter thread — 1536 B required for printf with 7 format args
// ("type", node name, ts_ms, heap_used, heap_max, heap_free, alloc_fail, stack_free)
// ARM std printf lib needs ~800-1200 B per call; 640 B caused stack overflow
// and INVPC HardFault via interrupt-on-corrupted-SP during the printf call.
#ifndef MEM_REPORTER_STACK_SIZE
#define MEM_REPORTER_STACK_SIZE 2048
#endif

// Trim: keep the node name tag short so the JSON line stays under 256 chars
#ifndef MEM_REPORTER_NODE_NAME_MAX
#define MEM_REPORTER_NODE_NAME_MAX 32
#endif

namespace {

static char   _mem_reporter_node_name[MEM_REPORTER_NODE_NAME_MAX] = "stm32";
static Thread _mem_reporter_thread(osPriorityLow, MEM_REPORTER_STACK_SIZE,
                                   nullptr, "mem_rpt");

void _memory_reporter_task()
{
    mbed_stats_heap_t  heap_stats;
    mbed_stats_stack_t self_stack;
    Timer uptime;
    uptime.start();

    while (true) {
        mbed_stats_heap_get(&heap_stats);
        mbed_stats_stack_get(&self_stack);  // reporter thread's own stack

        uint32_t used  = heap_stats.current_size;
        uint32_t max   = heap_stats.max_size;
        uint32_t rsvd  = heap_stats.reserved_size;
        uint32_t free_ = (rsvd > used) ? (rsvd - used) : 0;
        uint32_t fail  = heap_stats.alloc_fail_cnt;

        // report_stack_free: space not yet used in this thread's stack
        uint32_t stack_free = self_stack.reserved_size - self_stack.max_size;

        int ts_ms = (int)chrono::duration_cast<chrono::milliseconds>(
                            uptime.elapsed_time()).count();

        // Pre-format into a local buffer and write atomically.
        // Using printf directly causes byte-level corruption because the
        // default UART TX buffer (32 B) overflows multiple times for a ~200 char
        // JSON line, and the USB CDC can't drain fast enough between fills.
        // snprintf + fwrite minimizes the number of UART buffer refills.
        char buf[256];
        int len = snprintf(buf, sizeof(buf),
               "\r\n{\"type\":\"STM32_MEM\",\"node\":\"%s\",\"ts_ms\":%d,"
               "\"heap_used\":%lu,\"heap_max\":%lu,\"heap_free\":%lu,"
               "\"alloc_fail\":%lu,\"stack_free\":%lu}\r\n",
               _mem_reporter_node_name,
               ts_ms,
               (unsigned long)used,
               (unsigned long)max,
               (unsigned long)free_,
               (unsigned long)fail,
               (unsigned long)stack_free);

        if (len > 0 && len < (int)sizeof(buf)) {
            fwrite(buf, 1, len, stdout);
            fflush(stdout);
        }

        // Drain delay: at 115200 baud, 256 chars ≈ 22ms. Give USB CDC time
        // to flush before sleeping — prevents back-pressure on next write.
        ThisThread::sleep_for(chrono::milliseconds(30));
        ThisThread::sleep_for(chrono::milliseconds(MEM_REPORT_INTERVAL_MS - 30));
    }
}

} // anonymous namespace

/**
 * @brief Start the memory reporter thread.
 * @param node_name  Short identifier printed in every JSON line (e.g. "chassis" or "sensors").
 *
 * Call once after mros2::init() and all application threads have been launched.
 */
inline void memory_reporter_start(const char* node_name)
{
    if (node_name && node_name[0] != '\0') {
        strncpy(_mem_reporter_node_name, node_name,
                MEM_REPORTER_NODE_NAME_MAX - 1);
        _mem_reporter_node_name[MEM_REPORTER_NODE_NAME_MAX - 1] = '\0';
    }
    _mem_reporter_thread.start(_memory_reporter_task);
}
