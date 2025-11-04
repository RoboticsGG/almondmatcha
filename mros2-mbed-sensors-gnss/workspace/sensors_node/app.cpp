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
#include "msgs_ifaces/msg/main_sens_data.hpp"
#include "encoder_control.h"
#include "power_monitor.h"
#include "gnss_reader.h"
#include <cstdlib>
#include <cstring>

// ============================================================================
// SAMPLING RATE CONSTANTS
// ============================================================================

// Task polling periods (milliseconds)
const uint32_t ENCODER_SAMPLE_PERIOD_MS = 100;    // Encoder task @ 10 Hz
const uint32_t POWER_SAMPLE_PERIOD_MS = 200;      // Power monitor task @ 5 Hz
const uint32_t GNSS_SAMPLE_PERIOD_MS = 100;       // GNSS reader task @ 10 Hz
const uint32_t MAIN_LOOP_PERIOD_MS = 250;         // Main publishing loop @ 4 Hz
const uint32_t GNSS_PRINT_INTERVAL = 4;           // Print GNSS every 4 main loops (1 second)

// ============================================================================
// SAMPLING RATE CONSTANTS
// ============================================================================
// Task polling periods (milliseconds)
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
// SAMPLING RATE CONSTANTS
// ============================================================================
// Task polling periods (milliseconds)
const uint32_t ENCODER_SAMPLE_PERIOD_MS = 100;    // Encoder task @ 10 Hz
const uint32_t POWER_SAMPLE_PERIOD_MS = 200;      // Power monitor task @ 5 Hz
const uint32_t GNSS_SAMPLE_PERIOD_MS = 100;       // GNSS reader task @ 10 Hz
const uint32_t MAIN_LOOP_PERIOD_MS = 250;         // Main publishing loop @ 4 Hz
const uint32_t GNSS_PRINT_INTERVAL = 4;           // Print GNSS every 4 main loops (1 second)

// ============================================================================
// SHARED SENSOR DATA STRUCTURE
// ============================================================================

/**
 * @brief Thread-safe container for all sensor readings
 * Protected by sensor_data_mutex for safe read/write between tasks
 */
struct {
    int32_t encoder_A = 0;          // Encoder A count (from encoder task)
    int32_t encoder_B = 0;          // Encoder B count (from encoder task)
    float bus_voltage = 0.0f;       // Bus voltage in Volts (from power task)
    float current = 0.0f;           // System current in Amps (from power task)
    char nmea_sentence[NMEA_BUFFER_SIZE] = "$GNRMC,,,,,,,,,,,N*71";  // GNSS NMEA string (from GNSS task) - default placeholder when no fix
} sensor_data;

Mutex sensor_data_mutex;            // Protects access to sensor_data struct

// ============================================================================
// I2C HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Read a 16-bit register from INA226
 * @param reg Register address to read
 * @return 16-bit register value, or -1 on error
 */
int16_t read_register(uint8_t reg) {
    char cmd[1] = { reg };
    
    // Write register address
    if (i2c.write(INA226_ADDR, cmd, 1, true) != 0) {
        return -1;  // Error writing address
    }
    
    // Read 2 bytes (16-bit value)
    char data[2];
    if (i2c.read(INA226_ADDR, data, 2) != 0) {
        return -1;  // Error reading data
    }
    
    // Combine bytes: MSB first, then LSB
    return (data[0] << 8) | data[1];
}

// ============================================================================
// GNSS/RTK HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Read NMEA string from SimpleRTK2b GNSS receiver
 * 
 * Reads data from USART6 serial port and buffers complete NMEA sentences.
 * NMEA sentences start with '$' and end with '\n' or '\r\n'
 * 
 * @param output_buffer Pointer to buffer for storing complete NMEA sentence
 * @param buffer_size Maximum size of output buffer
 * @return Number of characters in the NMEA sentence (0 if no complete sentence yet)
 */
size_t read_nmea_string(char* output_buffer, size_t buffer_size) {
    // Read available data from serial port
    while (gnss_serial.readable()) {
        uint8_t ch_buffer;
        ssize_t read_result = gnss_serial.read(&ch_buffer, 1);  // Read 1 byte
        
        if (read_result <= 0) {
            break;  // No data available or error
        }
        
        char ch = (char)ch_buffer;
        
        // Start of a new NMEA sentence
        if (ch == '$') {
            nmea_buffer_index = 0;
            nmea_buffer[nmea_buffer_index++] = ch;
        }
        // End of NMEA sentence (carriage return or line feed)
        else if ((ch == '\r' || ch == '\n') && nmea_buffer_index > 0) {
            // Add null terminator
            nmea_buffer[nmea_buffer_index] = '\0';
            
            // Copy to output buffer if it fits
            if (nmea_buffer_index < buffer_size) {
                strcpy(output_buffer, nmea_buffer);
                size_t result = nmea_buffer_index;
                nmea_buffer_index = 0;
                return result;  // Return length of complete NMEA sentence
            }
            
            nmea_buffer_index = 0;  // Reset on buffer overflow
        }
        // Regular character in NMEA sentence
        else if (nmea_buffer_index > 0 && nmea_buffer_index < NMEA_BUFFER_SIZE - 1) {
            nmea_buffer[nmea_buffer_index++] = ch;
        }
    }
    
    return 0;  // No complete NMEA sentence available yet
}

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
            
            // TODO: Add NMEA sentence parsing here
            // Examples of common NMEA sentences:
            // $GPRMC - Recommended Minimum Navigation Information
            // $GPGGA - Global Positioning System Fix Data
            // $GPGSA - GPS DOP and Active Satellites
            // $GPGSV - GPS Satellites in View
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
  setenv("ROS_DOMAIN_ID", "6", 6);
  
  if (mros2_platform::network_connect()) {
    MROS2_ERROR("failed to connect and setup network! aborting,,,");
    return -1;
  } else {
    MROS2_INFO("successfully connect and setup network\r\n---");
  }

  // ---- Platform and mROS2 Initialization ----
  MROS2_INFO("%s start!", MROS2_PLATFORM_NAME);
  MROS2_INFO("app name: STM32 Sensors Node (Domain 6)");

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  // ---- Node and Publisher Setup ----
  mros2::Node node = mros2::Node::create_node("mros2_node_sensors_d6");
  
  // Create publisher for sensor data aggregation
  mros2::Publisher PubSensData = 
    node.create_publisher<msgs_ifaces::msg::MainSensData>("tp_sensdata_d6", 10);

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
  // All console printing is centralized here.
  
  uint32_t print_counter = 0;  // Counter to throttle GNSS printing
  
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
    msgs_ifaces::msg::MainSensData msgs;
    msgs.mainsensdata_msg.mt_lf_encode_msg = enc_A;      // Left encoder count
    msgs.mainsensdata_msg.mt_rt_encode_msg = enc_B;      // Right encoder count
    msgs.mainsensdata_msg.sys_current_msg = curr;        // System current (A)
    msgs.mainsensdata_msg.sys_volt_msg = vbus;           // Bus voltage (V)
    PubSensData.publish(msgs);

    // Print main sensor debug information
    printf("MotorA: %ld | MotorB: %ld | Vbus: %.3f V | I: %.3f A\r\n", 
           enc_A, enc_B, vbus, curr);
    
    // Print GNSS NMEA sentence every GNSS_PRINT_INTERVAL main loops (1 second)
    print_counter++;
    if (print_counter >= GNSS_PRINT_INTERVAL) {
        print_counter = 0;
        printf("GNSS: %s\r\n", gnss_data);
    }

    // Main loop runs at MAIN_LOOP_PERIOD_MS (4 Hz)
    ThisThread::sleep_for(chrono::milliseconds(MAIN_LOOP_PERIOD_MS));
  }

  mros2::spin();
  return 0;
}