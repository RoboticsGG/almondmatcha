/**
 * @file memory_reporter.h
 * @brief Runtime statistics reporter for STM32 Nucleo-F767ZI
 *
 * Spawns a low-priority background thread that emits a JSON line to the USB
 * serial port every MEM_REPORT_INTERVAL_MS milliseconds.  Line format:
 *
 *   {"type":"STM32_STATS","node":"<NODE>","ts_ms":<uptime_ms>,
 *    "heap_used":<B>,"heap_max":<B>,"heap_free":<B>,"alloc_fail":<n>,
 *    "cpu_busy_pct":<0-100>,"cpu_idle_pct":<0-100>,
 *    "stack_peak_b":<B>,"stack_min_free_b":<B>,
 *    "udp_recv":<delta>,"udp_drop":<delta>,"udp_sent":<delta>,
 *    "eth_miss":<delta>}
 *
 * Field semantics:
 *   heap_*           — MbedOS heap allocator stats (cumulative peak, current)
 *   cpu_busy_pct     — CPU active % averaged over the last reporting interval
 *                      (100 - idle_thread - sleep - deep_sleep)
 *   stack_peak_b     — highest observed stack usage across all threads (bytes)
 *   stack_min_free_b — smallest remaining free stack margin (bytes);
 *                      <512 B indicates critical stack pressure
 *   udp_recv/drop/sent — lwIP UDP datagram counts DELTA since last interval
 *   eth_miss         — ETH DMA hardware missed/dropped frames DELTA
 *                      (auto-clears on read; covers host-buffer-full +
 *                      FIFO-overflow events before packets reach the IP stack)
 *
 * Required mbed_app.json additions:
 *   "macros": [..., "MBED_CPU_STATS_ENABLED=1"]
 *   "target_overrides"."*"."lwip.stats-enabled": true
 *
 * The ws_base serial collector (tools/collect_stm32_memory.py) filters lines
 * containing "type":"STM32_STATS" and writes them to CSV.  All other serial
 * output lands in the raw log only.
 *
 * Usage (unchanged from previous version):
 *   #include "memory_reporter.h"
 *   memory_reporter_start("chassis");   // after all application threads started
 */

#pragma once

#include "mbed.h"
#include "mbed_stats.h"

// lwIP UDP stats: available when "lwip.stats-enabled": true in mbed_app.json.
// mbed_config.h (generated, included via mbed.h) propagates LWIP_STATS=1.
#if defined(LWIP_STATS) && LWIP_STATS
#  include "lwip/stats.h"
#  define _MR_LWIP_STATS 1
#else
#  define _MR_LWIP_STATS 0
#endif

// ETH DMA missed-frame register: present on all STM32F7 Ethernet targets.
// The register auto-clears on each read (count since last read).
//   Bits [15:0]:  frames missed — host buffer full  (DMA Rx descriptor exhausted)
//   Bits [27:17]: frames missed — application FIFO overflow
#if defined(TARGET_NUCLEO_F767ZI) || defined(TARGET_STM32F767ZI) || \
    defined(TARGET_STM32F767xI)  || defined(TARGET_STM32F767XX)
#  define _MR_ETH_DMAMFBOCR 1
#endif

// ── Tuneable constants ────────────────────────────────────────────────────────
#ifndef MEM_REPORT_INTERVAL_MS
#  define MEM_REPORT_INTERVAL_MS 1000     // 1 s — one sample per SPDP period
#endif

// 3 KB: printf(~1.2 KB headroom) + per-thread stack-scan array (16×16 B)
// + CPU/lwIP stat structs + local variables.  2 KB caused stack overflow
// in earlier version; keep at 3 KB for headroom.
#ifndef MEM_REPORTER_STACK_SIZE
#  define MEM_REPORTER_STACK_SIZE 3072
#endif

#ifndef MEM_REPORTER_NODE_NAME_MAX
#  define MEM_REPORTER_NODE_NAME_MAX 32
#endif

// Max threads scanned for stack high-water mark (app uses ≤12; 16 is safe).
#define _MR_MAX_THREADS 16

// ─────────────────────────────────────────────────────────────────────────────
namespace {

static char   _mr_node_name[MEM_REPORTER_NODE_NAME_MAX] = "stm32";
static Thread _mr_thread(osPriorityLow, MEM_REPORTER_STACK_SIZE, nullptr, "mem_rpt");

// ── CPU delta state ───────────────────────────────────────────────────────────
#ifdef MBED_CPU_STATS_ENABLED
static mbed_stats_cpu_t _cpu_prev;
static bool             _cpu_first = true;
#endif

// ── lwIP UDP delta state ──────────────────────────────────────────────────────
#if _MR_LWIP_STATS
static uint32_t _udp_recv_prev = 0;
static uint32_t _udp_drop_prev = 0;
static uint32_t _udp_sent_prev = 0;
#endif

// ─────────────────────────────────────────────────────────────────────────────
void _mr_task()
{
    Timer uptime;
    uptime.start();

    while (true) {

        // ── 1. Heap stats ─────────────────────────────────────────────────────
        mbed_stats_heap_t heap;
        mbed_stats_heap_get(&heap);
        unsigned long heap_used  = (unsigned long)heap.current_size;
        unsigned long heap_max   = (unsigned long)heap.max_size;
        unsigned long heap_free  = (heap.reserved_size > heap.current_size)
                                   ? (unsigned long)(heap.reserved_size - heap.current_size)
                                   : 0UL;
        unsigned long alloc_fail = (unsigned long)heap.alloc_fail_cnt;

        // ── 2. All-thread stack watermark ─────────────────────────────────────
        // max_size     = peak bytes consumed (high-water mark since thread start)
        // reserved_size = total bytes allocated for this thread's stack
        mbed_stats_stack_t stacks[_MR_MAX_THREADS];
        size_t n_thr = mbed_stats_stack_get_each(stacks, _MR_MAX_THREADS);
        unsigned long stack_peak_b     = 0;
        unsigned long stack_min_free_b = 0xFFFFFFFFUL;
        for (size_t i = 0; i < n_thr; i++) {
            if (stacks[i].max_size > stack_peak_b)
                stack_peak_b = (unsigned long)stacks[i].max_size;
            unsigned long fr = (stacks[i].reserved_size > stacks[i].max_size)
                               ? (unsigned long)(stacks[i].reserved_size - stacks[i].max_size)
                               : 0UL;
            if (fr < stack_min_free_b) stack_min_free_b = fr;
        }
        if (stack_min_free_b == 0xFFFFFFFFUL) stack_min_free_b = 0;

        // ── 3. CPU busy % (delta over the reporting interval) ─────────────────
        unsigned cpu_busy_pct = 0;
        unsigned cpu_idle_pct = 100;
#ifdef MBED_CPU_STATS_ENABLED
        mbed_stats_cpu_t cpu_now;
        mbed_stats_cpu_get(&cpu_now);
        if (!_cpu_first) {
            uint64_t d_up    = cpu_now.uptime         - _cpu_prev.uptime;
            uint64_t d_idle  = cpu_now.idle_time       - _cpu_prev.idle_time;
            uint64_t d_sleep = cpu_now.sleep_time      - _cpu_prev.sleep_time;
            uint64_t d_deep  = cpu_now.deep_sleep_time - _cpu_prev.deep_sleep_time;
            if (d_up > 0) {
                uint64_t d_off = d_idle + d_sleep + d_deep;
                if (d_off > d_up) d_off = d_up;
                cpu_busy_pct = (unsigned)((100ULL * (d_up - d_off)) / d_up);
                cpu_idle_pct = (unsigned)(100U - cpu_busy_pct);
            }
        }
        _cpu_prev  = cpu_now;
        _cpu_first = false;
#endif

        // ── 4. lwIP UDP delta stats (packets since last interval) ─────────────
        // udp_drop > 0 means the IP stack received a UDP datagram but had no
        // matching socket / could not queue it — typically a buffer overrun.
        unsigned long udp_recv = 0, udp_drop = 0, udp_sent = 0;
#if _MR_LWIP_STATS
        uint32_t r_now = (uint32_t)lwip_stats.udp.recv;
        uint32_t d_now = (uint32_t)lwip_stats.udp.drop;
        uint32_t s_now = (uint32_t)lwip_stats.udp.xmit;   // field is xmit, not sent
        udp_recv = (unsigned long)(r_now - _udp_recv_prev);
        udp_drop = (unsigned long)(d_now - _udp_drop_prev);
        udp_sent = (unsigned long)(s_now - _udp_sent_prev);
        _udp_recv_prev = r_now;
        _udp_drop_prev = d_now;
        _udp_sent_prev = s_now;
#endif

        // ── 5. ETH DMA hardware missed-frame counter (read auto-clears) ───────
        // eth_miss > 0 means the Ethernet DMA ran out of descriptors or the
        // MAC FIFO overflowed — packets were dropped before reaching lwIP.
        unsigned long eth_miss = 0;
#ifdef _MR_ETH_DMAMFBOCR
        {
            uint32_t mfbocr = ETH->DMAMFBOCR;
            eth_miss = (unsigned long)((mfbocr & 0x0000FFFFu)      // host buf full
                                      + ((mfbocr >> 17) & 0x7FFu)); // FIFO overflow
        }
#endif

        // ── 6. Format and emit JSON line ──────────────────────────────────────
        int ts_ms = (int)chrono::duration_cast<chrono::milliseconds>(
                            uptime.elapsed_time()).count();

        char buf[512];
        int len = snprintf(buf, sizeof(buf),
            "\r\n{\"type\":\"STM32_STATS\",\"node\":\"%s\",\"ts_ms\":%d,"
            "\"heap_used\":%lu,\"heap_max\":%lu,\"heap_free\":%lu,\"alloc_fail\":%lu,"
            "\"cpu_busy_pct\":%u,\"cpu_idle_pct\":%u,"
            "\"stack_peak_b\":%lu,\"stack_min_free_b\":%lu,"
            "\"udp_recv\":%lu,\"udp_drop\":%lu,\"udp_sent\":%lu,"
            "\"eth_miss\":%lu}\r\n",
            _mr_node_name,
            ts_ms,
            heap_used, heap_max, heap_free, alloc_fail,
            cpu_busy_pct, cpu_idle_pct,
            stack_peak_b, stack_min_free_b,
            udp_recv, udp_drop, udp_sent,
            eth_miss);

        if (len > 0 && len < (int)sizeof(buf)) {
            fwrite(buf, 1, (size_t)len, stdout);
            fflush(stdout);
        }

        ThisThread::sleep_for(chrono::milliseconds(MEM_REPORT_INTERVAL_MS));
    }
}

} // anonymous namespace

/**
 * @brief Start the runtime statistics reporter thread.
 * @param node_name  Short identifier in every JSON line (e.g. "chassis").
 *
 * Call once after mros2::init() and all application threads have been started.
 */
inline void memory_reporter_start(const char* node_name)
{
    if (node_name && node_name[0] != '\0') {
        strncpy(_mr_node_name, node_name, MEM_REPORTER_NODE_NAME_MAX - 1);
        _mr_node_name[MEM_REPORTER_NODE_NAME_MAX - 1] = '\0';
    }
    _mr_thread.start(_mr_task);
}
