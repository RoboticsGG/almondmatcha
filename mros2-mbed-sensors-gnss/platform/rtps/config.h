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

// SINGLE-DOMAIN POC CONFIGURATION — sensors/GNSS board (NUCLEO-F767ZI)
// This board: 1 publisher (tpc_chassis_sensors), 0 subscribers
//
// System topology (D5, single-domain):
//   RPi: 8 nodes | Jetson: 4 nodes | Base: 2 nodes | STM32×2: 1 each = 16 total
//   15 topics (+/rosout, /parameter_events per node = +28 writers, +14 readers)
//   Total remote DDS writers: ~51 | Total remote DDS readers: ~50
//   Heaviest node by writers: gnss_mission_monitor (10) | by readers: mission_monitoring_rpi (11)
//
// MAX_NUM_PARTICIPANTS = local participant pool (STM32 creates exactly 1).
// Remote participant tracking uses SPDP_MAX_NUMBER_FOUND_PARTICIPANTS (separate).
// Upstream embedded configs (config_stm.h, config_r5.h) all use 1.
//
// OVERALL_HEAP_SIZE (thread stacks only):
//   1×4096 +  1×8192  (thread pool writer+reader)
//  + 1×4096            (SPDP writer per local participant)
//  + 3×4096            (heartbeat per stateful writer)
//  = 4096+8192+4096+12288 = 28,672 B  ← fits easily in Nucleo-F767ZI heap
const uint8_t NUM_STATEFUL_READERS = 2;               // SEDP: 2 internal; 0 app subscribers = 2 minimum
const uint8_t NUM_STATEFUL_WRITERS = 3;               // SEDP: 2 internal + 1 app publisher (tpc_chassis_sensors)
const uint8_t MAX_NUM_PARTICIPANTS = 1;               // local participant pool — STM32 creates exactly 1
const uint8_t NUM_WRITERS_PER_PARTICIPANT = 16;       // max DDS writers per remote node (10 actual + margin)
const uint8_t NUM_READERS_PER_PARTICIPANT = 16;       // max DDS readers per remote node (11 actual + margin)
const uint8_t NUM_WRITER_PROXIES_PER_READER = 30;     // SEDP readers: 1 WriterProxy per remote participant
const uint8_t NUM_READER_PROXIES_PER_WRITER = 30;     // SEDP writers: 1 ReaderProxy per remote participant

// SEDP unmatched endpoint pools — disabled for static topology.
// These only serve late-joining local endpoints (never happens on STM32).
// addUnmatchedRemote*() has its own isFull() guard, so these never trigger
// the [MemoryPool] RESSOURCE LIMIT EXCEEDED error even if undersized.
// Set SEDP_VERBOSE=0 in patch 003 to compile out the tracking entirely.
const uint8_t MAX_NUM_UNMATCHED_REMOTE_WRITERS = 2;   // unused — static topology
const uint8_t MAX_NUM_UNMATCHED_REMOTE_READERS = 2;   // unused — static topology

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
const uint8_t SPDP_MAX_NUMBER_FOUND_PARTICIPANTS = 30; // 15 nodes + 3 daemons + ros2 CLI tools + margin
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
