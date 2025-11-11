/* mros2 example
 * Copyright (c) 2022 mROS-base
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// ============================================================================
// HEADER INCLUDES
// ============================================================================

#include "mros2.h"
#include "mros2-platform.h"
#include "msgs_ifaces/msg/chassis_sensors.hpp"
#include "encoder_control.h"
#include "power_monitor.h"
#include "gnss_reader.h"
#include <cstdlib>
#include <cstring>

// ============================================================================
// SAMPLING RATE CONSTANTS
// ============================================================================

const uint32_t ENCODER_SAMPLE_PERIOD_MS = 100;    // Encoder task @ 10 Hz
const uint32_t POWER_SAMPLE_PERIOD_MS = 200;      // Power monitor task @ 5 Hz
const uint32_t GNSS_SAMPLE_PERIOD_MS = 100;       // GNSS reader task @ 10 Hz
const uint32_t MAIN_LOOP_PERIOD_MS = 250;         // Main publishing loop @ 4 Hz
const uint32_t GNSS_PRINT_INTERVAL = 4;           // Print GNSS every 4 main loops (1 second)

// ============================================================================
// SHARED SENSOR DATA STRUCTURE
// ============================================================================

/**
 * @brief Thread-safe container for aggregated sensor readings
 * Protected by sensor_data_mutex for safe read/write between tasks
 * 
 * Updated by:
 * - encoder_read_task: encoder_A, encoder_B
 * - power_monitor_task: bus_voltage, current
 * - gnss_reader_task: nmea_sentence
 */
struct {
    int32_t encoder_A = 0;          // Encoder A count (from encoder task)
    int32_t encoder_B = 0;          // Encoder B count (from encoder task)
    float bus_voltage = 0.0f;       // Bus voltage in Volts (from power task)
    float current = 0.0f;           // System current in Amps (from power task)
    char nmea_sentence[GNSS_NMEA_BUFFER_SIZE] = "$GNRMC,,,,,,,,,,,N*71";  // GNSS NMEA string
} sensor_data;

Mutex sensor_data_mutex;            // Protects access to sensor_data struct

// ============================================================================
// ENCODER READING TASK
// ============================================================================

/**
 * @brief Encoder reading task - polls quadrature encoders
 * 
 * Periodically reads encoder counts via encoder_control module
 * and stores results in shared sensor_data structure.
 * Polling rate: 100ms (10 Hz)
 */
void encoder_read_task() {
    MROS2_INFO("Encoder reader task started");
    
    while (true) {
        // Read encoder counts from interrupt handlers
        int32_t enc_a = encoder_get_count_a();
        int32_t enc_b = encoder_get_count_b();
        
        // Store in shared data structure with mutex protection
        sensor_data_mutex.lock();
        sensor_data.encoder_A = enc_a;
        sensor_data.encoder_B = enc_b;
        sensor_data_mutex.unlock();
        
        // Sleep before next read
        ThisThread::sleep_for(chrono::milliseconds(ENCODER_SAMPLE_PERIOD_MS));
    }
}

// ============================================================================
// INA226 POWER MONITORING TASK
// ============================================================================

/**
 * @brief Power monitoring task - reads voltage and current via I2C
 * 
 * Periodically reads INA226 power monitor via power_monitor module
 * and stores results in shared sensor_data structure.
 * Polling rate: 200ms (5 Hz) - I2C communication is slower
 */
void power_monitor_task() {
    MROS2_INFO("Power monitor task started");
    
    while (true) {
        // Read voltage and current from INA226
        float voltage = power_monitor_read_bus_voltage();
        float current = power_monitor_read_current();
        
        // Store in shared data structure with mutex protection
        sensor_data_mutex.lock();
        sensor_data.bus_voltage = voltage;
        sensor_data.current = current;
        sensor_data_mutex.unlock();
        
        // Sleep before next read
        ThisThread::sleep_for(chrono::milliseconds(POWER_SAMPLE_PERIOD_MS));
    }
}

// ============================================================================
// GNSS READER TASK
// ============================================================================

/**
 * @brief GNSS reader task - reads NMEA sentences from SimpleRTK2b
 * 
 * Runs independently to continuously read NMEA sentences from the receiver
 * via gnss_reader module and stores them in shared sensor_data structure.
 * Polling rate: 100ms (10 Hz)
 */
void gnss_reader_task() {
    MROS2_INFO("GNSS reader task started");
    
    while (true) {
        // Read NMEA data from serial port
        char nmea_sentence[GNSS_NMEA_BUFFER_SIZE];
        size_t nmea_length = gnss_reader_read_nmea(nmea_sentence, GNSS_NMEA_BUFFER_SIZE);
        
        if (nmea_length > 0) {
            // Valid NMEA sentence received, store it with mutex protection
            sensor_data_mutex.lock();
            strncpy(sensor_data.nmea_sentence, nmea_sentence, GNSS_NMEA_BUFFER_SIZE - 1);
            sensor_data.nmea_sentence[GNSS_NMEA_BUFFER_SIZE - 1] = '\0';
            sensor_data_mutex.unlock();
        }
        
        // Sleep before next read
        ThisThread::sleep_for(chrono::milliseconds(GNSS_SAMPLE_PERIOD_MS));
    }
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================

int main()
{
  // ---- Network Initialization ----
  setenv("ROS_DOMAIN_ID", "5", 5);
  
  if (mros2_platform::network_connect()) {
    MROS2_ERROR("failed to connect and setup network! aborting,,,");
    return -1;
  } else {
    MROS2_INFO("successfully connect and setup network\r\n---");
  }

  // ---- Platform and mROS2 Initialization ----
  MROS2_INFO("%s start!", MROS2_PLATFORM_NAME);
  MROS2_INFO("app name: STM32 Sensors Node (Domain 5)");

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  // ---- Node and Publisher Setup ----
  mros2::Node node = mros2::Node::create_node("mros2_node_sensors_d6");
  
  // Create publisher for sensor data aggregation
  mros2::Publisher PubSensData = 
    node.create_publisher<msgs_ifaces::msg::ChassisSensors>("tpc_chassis_sensors", 10);

  // ===== DDS DISCOVERY COORDINATION FIX =====
  // Wait for DDS/RTPS participant discovery to complete
  // SPDP announcements sent every 500ms (SPDP_RESEND_PERIOD_MS)
  // Need at least 8-10 cycles for reliable discovery across all nodes
  // (ws_rpi(5) + ws_base(2) + ws_jetson(3) + STM32(2) = 12 participants)
  // This fixes intermittent "no messages received" and "[Memory pool] resource limit exceed"
  // by ensuring publishers/subscribers are fully matched before data transmission starts
  MROS2_INFO("Waiting 8 seconds for DDS participant discovery (12 participants)...");
  osDelay(8000);  // 8 seconds = 16 SPDP cycles @ 500ms for robust discovery
  MROS2_INFO("Discovery wait complete - initializing sensors");
  // ==========================================

  // ---- Initialize All Sensor Modules ----
  MROS2_INFO("Initializing sensor modules...");
  
  encoder_init();
  MROS2_INFO("Encoder interrupt handlers configured");
  
  power_monitor_init();
  MROS2_INFO("Power monitor I2C initialized (400 kHz)");
  
  gnss_reader_init();
  MROS2_INFO("GNSS serial interface configured on USART6 (115200 baud)");

  // ---- Launch Independent Sensor Tasks ----
  MROS2_INFO("Launching independent sensor tasks...");
  
  Thread encoder_thread(osPriorityNormal, 2048);  // 2KB stack
  encoder_thread.start(encoder_read_task);
  MROS2_INFO("Encoder reader task launched (10 Hz)");
  
  Thread power_thread(osPriorityNormal, 3072);    // 3KB stack
  power_thread.start(power_monitor_task);
  MROS2_INFO("Power monitor task launched (5 Hz)");
  
  Thread gnss_thread(osPriorityNormal, 4096);     // 4KB stack
  gnss_thread.start(gnss_reader_task);
  MROS2_INFO("GNSS reader task launched (10 Hz)");

  osDelay(1000);  // Wait for initialization to complete
  MROS2_INFO("ready to pub/sub message\r\n---");

  // ---- Main Sensor Publishing Loop ----
  // Main loop focuses on aggregating data from the three independent tasks
  // and publishing to ROS2 at 4 Hz (250ms interval).
  
  while (true) {
    // Read all sensor data from shared structure with mutex protection
    sensor_data_mutex.lock();
    int32_t enc_A = sensor_data.encoder_A;
    int32_t enc_B = sensor_data.encoder_B;
    float vbus = sensor_data.bus_voltage;
    float curr = sensor_data.current;
    char gnss_data[GNSS_NMEA_BUFFER_SIZE];
    strncpy(gnss_data, sensor_data.nmea_sentence, GNSS_NMEA_BUFFER_SIZE - 1);
    gnss_data[GNSS_NMEA_BUFFER_SIZE - 1] = '\0';
    sensor_data_mutex.unlock();

    // Prepare and publish sensor data message
    msgs_ifaces::msg::ChassisSensors msgs;
    msgs.mt_lf_encode_msg = enc_A;      // Left encoder count
    msgs.mt_rt_encode_msg = enc_B;      // Right encoder count
    msgs.sys_current_msg = curr;        // System current (A)
    msgs.sys_volt_msg = vbus;           // Bus voltage (V)
    PubSensData.publish(msgs);

    // Print sensor status (compact format) - overwrite previous line
    // This will overwrite the previous line, but if the new text is shorter, remnants may remain.
    // To fully clear the line, pad with spaces to the end.
    printf("\r[SENSORS] Enc(A:%ld B:%ld) Power(%.2fV %.2fA) GNSS:%s", 
       enc_A, enc_B, vbus, curr, gnss_data);
    // Pad with spaces to clear any leftover characters from previous output
    printf("%*s", 20, ""); // Adjust 20 to be enough for your longest line
    fflush(stdout);

    // Main loop runs at MAIN_LOOP_PERIOD_MS (4 Hz)
    ThisThread::sleep_for(chrono::milliseconds(MAIN_LOOP_PERIOD_MS));
  }

  mros2::spin();
  return 0;
}