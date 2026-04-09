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
const GuidPrefix_t BASE_GUID_PREFIX{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12};

const uint8_t DOMAIN_ID = 5; // 230 possible with UDP
const uint8_t NUM_STATELESS_WRITERS = 4;
const uint8_t NUM_STATELESS_READERS = 4;

// SINGLE-DOMAIN POC CONFIGURATION — sensors/GNSS board
// This board: 1 publisher (tpc_chassis_sensors), 0 subscribers
//
// NUM_STATEFUL_WRITERS / NUM_STATEFUL_READERS are the LOCAL endpoint pool sizes for
// THIS board only.  Each stateful writer spawns one heartbeat OS thread consuming
// HEARTBEAT_STACKSIZE bytes from the heap.  Setting these to ws_base-scale values
// (28/32) consumes 28×4096 = 114 KB in thread stacks alone and OOMs the Nucleo.
//
// embeddedRTPS internally creates 2 SEDP stateful writers (sedpPubWriter,
// sedpSubWriter) before any app publisher.  NUM_STATEFUL_WRITERS must be at least
// 2 (SEDP) + app publishers.  This board has 1 app publisher, so minimum = 3.
//
// OVERALL_HEAP_SIZE (thread stacks only):
//   1×4096 +  1×8192  (thread pool writer+reader)
//  +20×4096            (SPDP writer per participant)
//  + 3×4096            (heartbeat per stateful writer)
//  = 4096+8192+81920+12288 = 106,496 B  ← fits in Nucleo-F767ZI heap
const uint8_t NUM_STATEFUL_READERS = 2;               // SEDP: 2 internal; 0 app subscribers = 2 minimum
const uint8_t NUM_STATEFUL_WRITERS = 3;               // SEDP: 2 internal + 1 app publisher (tpc_chassis_sensors)
const uint8_t MAX_NUM_PARTICIPANTS = 20;              // D5 single-domain: 15 actual + 5 margin
const uint8_t NUM_WRITERS_PER_PARTICIPANT = 8;        // max publishers per remote node
const uint8_t NUM_READERS_PER_PARTICIPANT = 8;        // max subscribers per remote node
const uint8_t NUM_WRITER_PROXIES_PER_READER = 22;     // must cover SPDP_MAX_NUMBER_FOUND_PARTICIPANTS (19); was 5 — root cause of RX failure
const uint8_t NUM_READER_PROXIES_PER_WRITER = 15;     // tpc_chassis_sensors has ~8 subscribers

// Discovery burst handling
// WARNING: these pools are embedded in EVERY Participant slot (all 20) in static BSS.
// MAX_NUM_UNMATCHED_REMOTE_* × 20 participants × sizeof(TopicDataCompressed) ≈ 60 B
// → each +1 here costs 20×60=1.2 KB of BSS. Keep modest.
const uint8_t MAX_NUM_UNMATCHED_REMOTE_WRITERS = 20;  // ≥ MAX_NUM_PARTICIPANTS (16 actual)
const uint8_t MAX_NUM_UNMATCHED_REMOTE_READERS = 25;  // ≥ MAX_NUM_PARTICIPANTS + monitoring burst margin

const uint8_t MAX_NUM_READER_CALLBACKS = 2;  // This board has no callbacks (publish-only)


const uint8_t HISTORY_SIZE_STATELESS = 2; 
const uint8_t HISTORY_SIZE_STATEFUL = 5;  // Reduced for memory efficiency 

const uint8_t MAX_TYPENAME_LENGTH = 60;
const uint8_t MAX_TOPICNAME_LENGTH = 40;

const int HEARTBEAT_STACKSIZE = 4096;              // byte - Halved from 8192, sufficient for mbed
const int THREAD_POOL_WRITER_STACKSIZE = 4096;     // byte - Halved from 8192, sufficient for mbed
const int THREAD_POOL_READER_STACKSIZE = 8192;     // byte - Restored to 8192: 4096 causes HardFault
                                                   // during SEDP burst (3+ Linux nodes joining)
const uint16_t SPDP_WRITER_STACKSIZE = 4096;       // byte - Halved from 8192, critical memory savings

const uint16_t SF_WRITER_HB_PERIOD_MS = 2000; // 2s heartbeat for writer detection
const uint16_t SPDP_RESEND_PERIOD_MS = 500;   // 500ms SPDP announcements for faster discovery
const uint8_t SPDP_CYCLECOUNT_HEARTBEAT =
    2; // skip x SPDP rounds before checking liveliness
const uint8_t SPDP_WRITER_PRIO = 24;
const uint8_t SPDP_MAX_NUMBER_FOUND_PARTICIPANTS = 19; // MAX_NUM_PARTICIPANTS - 1 (excludes self)
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
const int THREAD_POOL_WORKLOAD_QUEUE_LENGTH = 40; // Doubled from 20 to absorb discovery burst

constexpr int OVERALL_HEAP_SIZE =
    THREAD_POOL_NUM_WRITERS * THREAD_POOL_WRITER_STACKSIZE +
    THREAD_POOL_NUM_READERS * THREAD_POOL_READER_STACKSIZE +
    MAX_NUM_PARTICIPANTS * SPDP_WRITER_STACKSIZE +
    NUM_STATEFUL_WRITERS * HEARTBEAT_STACKSIZE;
} // namespace Config
} // namespace rtps

#endif // RTPS_CONFIG_H
