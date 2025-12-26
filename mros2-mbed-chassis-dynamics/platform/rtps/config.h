/*
The MIT License
Copyright (c) 2019 Lehrstuhl Informatik 11 - RWTH Aachen University
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE

This file is part of embeddedRTPS.

Author: i11 - Embedded Software, RWTH Aachen University
*/

#ifndef RTPS_CONFIG_H
#define RTPS_CONFIG_H

/* need to add for mros2 */
#include "mros2/freertos_conversion.h"

#include "rtps/common/types.h"

namespace rtps {

#define IS_LITTLE_ENDIAN 1

// define only if using FreeRTOS
#define OS_IS_FREERTOS

namespace Config {
const VendorId_t VENDOR_ID = {13, 37};
/* setting IP address for RTPS was moved to mros2-<pf>/target/mros2_target.cpp */
// const std::array<uint8_t, 4> IP_ADDRESS = {
//     192, 168, 11, 2}; // Needs to be set in lwipcfg.h too.
extern std::array<uint8_t, 4> IP_ADDRESS;
const GuidPrefix_t BASE_GUID_PREFIX{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13};

const uint8_t DOMAIN_ID = 5; // 230 possible with UDP
const uint8_t NUM_STATELESS_WRITERS = 4;
const uint8_t NUM_STATELESS_READERS = 4;

// OPTIMIZED CONFIGURATION - Domain 4/5/6 architecture
// Domain 5 participants: 11 actual (ws_rpi=6, ws_jetson=1, ws_base=1, STM32=2, rover_monitoring=1)
// Memory usage: ~180-200 KB, Free: ~310-330 KB (60%+ headroom)
const uint8_t NUM_STATEFUL_READERS = 32;              // Max endpoints for all remote writers
const uint8_t NUM_STATEFUL_WRITERS = 28;              // Max endpoints for all remote readers
const uint8_t MAX_NUM_PARTICIPANTS = 15;              // Domain 5: 11 actual + 4 margin (sufficient)
const uint8_t NUM_WRITERS_PER_PARTICIPANT = 20;       // Max publishers per node (ws_base heavy)
const uint8_t NUM_READERS_PER_PARTICIPANT = 20;       // Max subscribers per node (ws_base heavy)
const uint8_t NUM_WRITER_PROXIES_PER_READER = 28;     // Track all possible remote writers
const uint8_t NUM_READER_PROXIES_PER_WRITER = 28;     // Track all possible remote readers

// Discovery burst handling - Critical for preventing [MemoryPool] errors
const uint8_t MAX_NUM_UNMATCHED_REMOTE_WRITERS = 60;  // Handles simultaneous discovery
const uint8_t MAX_NUM_UNMATCHED_REMOTE_READERS = 80;  // Handles ws_base monitoring burst
    
const uint8_t MAX_NUM_READER_CALLBACKS = 3;  // This board only has 1 callback


const uint8_t HISTORY_SIZE_STATELESS = 2; 
const uint8_t HISTORY_SIZE_STATEFUL = 5;  // Reduced for memory efficiency 

const uint8_t MAX_TYPENAME_LENGTH = 60;
const uint8_t MAX_TOPICNAME_LENGTH = 40;

const int HEARTBEAT_STACKSIZE = 4096;              // byte - Halved from 8192, sufficient for mbed
const int THREAD_POOL_WRITER_STACKSIZE = 4096;     // byte - Halved from 8192, sufficient for mbed
const int THREAD_POOL_READER_STACKSIZE = 4096;     // byte - Halved from 8192, sufficient for mbed
const uint16_t SPDP_WRITER_STACKSIZE = 4096;       // byte - Halved from 8192, critical memory savings

const uint16_t SF_WRITER_HB_PERIOD_MS = 2000; // 2s heartbeat for writer detection
const uint16_t SPDP_RESEND_PERIOD_MS = 500;   // 500ms SPDP announcements for faster discovery
const uint8_t SPDP_CYCLECOUNT_HEARTBEAT =
    2; // skip x SPDP rounds before checking liveliness
const uint8_t SPDP_WRITER_PRIO = 24;
const uint8_t SPDP_MAX_NUMBER_FOUND_PARTICIPANTS = 14; // Match MAX_NUM_PARTICIPANTS
const uint8_t SPDP_MAX_NUM_LOCATORS = 5;
const Duration_t SPDP_DEFAULT_REMOTE_LEASE_DURATION = {
    100, 0}; // Default lease duration for remote participants, usually
             // overwritten by remote info
const Duration_t SPDP_MAX_REMOTE_LEASE_DURATION = {
    180,
    0}; // Absolute maximum lease duration, ignoring remote participant info

const Duration_t SPDP_LEASE_DURATION = {100, 0};

const int MAX_NUM_UDP_CONNECTIONS = 10;

const int THREAD_POOL_NUM_WRITERS = 1;
const int THREAD_POOL_NUM_READERS = 1;
const int THREAD_POOL_WRITER_PRIO = 24;
const int THREAD_POOL_READER_PRIO = 24;
const int THREAD_POOL_WORKLOAD_QUEUE_LENGTH = 20;

constexpr int OVERALL_HEAP_SIZE =
    THREAD_POOL_NUM_WRITERS * THREAD_POOL_WRITER_STACKSIZE +
    THREAD_POOL_NUM_READERS * THREAD_POOL_READER_STACKSIZE +
    MAX_NUM_PARTICIPANTS * SPDP_WRITER_STACKSIZE +
    NUM_STATEFUL_WRITERS * HEARTBEAT_STACKSIZE;
} // namespace Config
} // namespace rtps

#endif // RTPS_CONFIG_H
