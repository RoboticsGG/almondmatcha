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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "msgs_ifaces/msg/main_sens_data.hpp"
#include "msgs_ifaces/msg/sub_sens_data.hpp"
#include <cstdlib>

// ============================================================================
// PIN DEFINITIONS - ENCODER INPUTS
// ============================================================================
// Encoder A uses PA_15 and PB_5 (quadrature encoder)
// Encoder B uses PB_3 and PB_4 (quadrature encoder)

InterruptIn encA_A(PA_15);  // Encoder A - Channel A
InterruptIn encA_B(PB_5);   // Encoder A - Channel B

InterruptIn encB_A(PB_3);   // Encoder B - Channel A
InterruptIn encB_B(PB_4);   // Encoder B - Channel B

// ============================================================================
// GLOBAL VARIABLES - ENCODER COUNTERS
// ============================================================================

volatile int32_t countA = 0;  // Total count for Encoder A
volatile int32_t countB = 0;  // Total count for Encoder B

// ============================================================================
// ENCODER INTERRUPT HANDLERS
// ============================================================================
// Quadrature encoder logic:
// - Rising edge: check channel B to determine direction
// - Falling edge: check channel B to determine direction

void encA_A_rise() { 
  countA += (encA_B.read() == 0) ? 1 : -1;  // Increment or decrement based on phase
}

void encA_A_fall() { 
  countA += (encA_B.read() == 1) ? 1 : -1;  // Increment or decrement based on phase
}

void encB_A_rise() { 
  countB += (encB_B.read() == 0) ? 1 : -1;  // Increment or decrement based on phase
}

void encB_A_fall() { 
  countB += (encB_B.read() == 1) ? 1 : -1;  // Increment or decrement based on phase
}

// ============================================================================
// I2C CONFIGURATION - INA226 POWER MONITORING
// ============================================================================
// INA226 is used for voltage and current measurement
// I2C on PB_9 (SDA) and PB_8 (SCL)

I2C i2c(PB_9, PB_8);                // SDA, SCL
const int INA226_ADDR = 0x40 << 1;  // INA226 I2C address (0x40 shifted)
const float Rshunt = 0.1f;           // Shunt resistor value in Ohms

// ============================================================================
// SERIAL CONFIGURATION - GNSS/RTK (USART6)
// ============================================================================
// SimpleRTK2b connected via USART6 (Arduino D0/D1)
// TX1 (U-blox TX) -> PG9 (USART6_RX)
// RX1 (U-blox RX) -> PG14 (USART6_TX)

UnbufferedSerial gnss_serial(PG_14, PG_9, 115200);  // TX, RX, Baud rate
const size_t NMEA_BUFFER_SIZE = 256;
char nmea_buffer[NMEA_BUFFER_SIZE];
size_t nmea_buffer_index = 0;

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
 * @brief Quadrature encoder reader task
 * 
 * Periodically reads and stores current encoder counts (countA, countB) into
 * the shared sensor_data structure with Mutex protection.
 * Polling rate: ~100ms (10 Hz)
 */
void encoder_read_task() {
    MROS2_INFO("Encoder reader task started");
    
    while (true) {
        // Read current encoder counts (already updated by interrupt handlers)
        sensor_data_mutex.lock();
        sensor_data.encoder_A = countA;
        sensor_data.encoder_B = countB;
        sensor_data_mutex.unlock();
        
        // Sleep for ENCODER_SAMPLE_PERIOD_MS before next read
        ThisThread::sleep_for(chrono::milliseconds(ENCODER_SAMPLE_PERIOD_MS));
    }
}

// ============================================================================
// INA226 POWER MONITORING TASK
// ============================================================================

/**
 * @brief I2C power monitoring task
 * 
 * Periodically reads voltage and current from the INA226 power monitor via I2C
 * and stores results into the shared sensor_data structure with Mutex protection.
 * Polling rate: ~200ms (5 Hz) - I2C communication is slower than GPIO
 */
void power_monitor_task() {
    MROS2_INFO("Power monitor task started");
    
    while (true) {
        // Read raw ADC values from INA226
        int16_t bus_raw = read_register(0x02);    // Bus voltage register
        int16_t shunt_raw = read_register(0x01);  // Shunt voltage register
        
        if (bus_raw >= 0 && shunt_raw >= 0) {
            // Convert raw ADC values to physical units
            float bus_voltage = bus_raw * 1.25e-3f;    // 1.25 mV per LSB
            float shunt_voltage = shunt_raw * 2.5e-6f; // 2.5 µV per LSB
            float current = shunt_voltage / Rshunt;    // Calculate current from shunt voltage
            
            // Store in shared data structure with mutex protection
            sensor_data_mutex.lock();
            sensor_data.bus_voltage = bus_voltage;
            sensor_data.current = current;
            sensor_data_mutex.unlock();
        }
        
        // Sleep for POWER_SAMPLE_PERIOD_MS before next read (I2C is slower)
        ThisThread::sleep_for(chrono::milliseconds(POWER_SAMPLE_PERIOD_MS));
    }
}

// ============================================================================
// GNSS READER TASK
// ============================================================================

/**
 * @brief GNSS/RTK NMEA data reader task
 * 
 * Runs independently to continuously read NMEA sentences from the SimpleRTK2b
 * receiver and stores them into the shared sensor_data structure with Mutex protection.
 * Also prints GNSS data every 1 second.
 * Polling rate: ~100ms (10 Hz)
 */
void gnss_reader_task() {
    MROS2_INFO("GNSS reader task started");
    
    while (true) {
        // Read NMEA data from serial port
        char nmea_sentence[NMEA_BUFFER_SIZE];
        size_t nmea_length = read_nmea_string(nmea_sentence, NMEA_BUFFER_SIZE);
        
        if (nmea_length > 0) {
            // Valid NMEA sentence received, store it with mutex protection
            sensor_data_mutex.lock();
            strcpy(sensor_data.nmea_sentence, nmea_sentence);
            sensor_data_mutex.unlock();
            
            // TODO: Parse NMEA sentence here
            // Examples of common NMEA sentences:
            // $GPRMC - Recommended Minimum Navigation Information
            // $GPGGA - Global Positioning System Fix Data
            // $GPGSA - GPS DOP and Active Satellites
            // $GPGSV - GPS Satellites in View
        }
        
        // Sleep for GNSS_SAMPLE_PERIOD_MS between reads (printing is handled by main loop)
        ThisThread::sleep_for(chrono::milliseconds(GNSS_SAMPLE_PERIOD_MS));
    }
}

// ============================================================================
// MAIN FUNCTION
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
  MROS2_INFO("app name: STM32Layer5");

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  // ---- Node and Publisher Setup ----
  mros2::Node node = mros2::Node::create_node("mros2_node_2");
  
  // Create publisher for sensor data (only 1 message type allowed per node)
  mros2::Publisher PubSensData = 
    node.create_publisher<msgs_ifaces::msg::MainSensData>("tp_sensdata_d5", 10);

  // ---- Encoder Interrupt Configuration ----
  // Attach interrupt handlers to encoder pins
  encA_A.rise(&encA_A_rise);
  encA_A.fall(&encA_A_fall);
  encB_A.rise(&encB_A_rise);
  encB_A.fall(&encB_A_fall);

  // ---- I2C Configuration ----
  i2c.frequency(400000);  // Set I2C frequency to 400 kHz

  // ---- USART6 (GNSS/RTK) Serial Configuration ----
  // SimpleRTK2b GNSS receiver on USART6 (Arduino D0/D1)
  gnss_serial.baud(115200);  // Ensure baud rate is set
  MROS2_INFO("GNSS serial interface configured on USART6 (PG9/PG14 - Arduino D0/D1)");

  // ---- Launch Independent Sensor Tasks ----
  // All three tasks run independently with their own polling rates:
  // - Encoder task: 100ms (10 Hz) - fast, reads GPIO
  // - Power task: 200ms (5 Hz) - slower, uses I2C communication
  // - GNSS task: 100ms (10 Hz) - fast, reads serial data
  
  Thread encoder_thread(osPriorityNormal, 2048);  // 2KB stack for encoder task
  encoder_thread.start(encoder_read_task);
  MROS2_INFO("Encoder reader task launched");
  
  Thread power_thread(osPriorityNormal, 3072);    // 3KB stack for I2C power task
  power_thread.start(power_monitor_task);
  MROS2_INFO("Power monitor task launched");
  
  Thread gnss_thread(osPriorityNormal, 4096);     // 4KB stack for GNSS task
  gnss_thread.start(gnss_reader_task);
  MROS2_INFO("GNSS reader task launched");

  osDelay(1000);  // Wait for initialization to complete
  MROS2_INFO("ready to pub/sub message\r\n---");

  // ---- Main Sensor Publishing Loop ----
  // Main loop focuses on aggregating data from the three independent tasks
  // and publishing to ROS2. No direct sensor I/O occurs here.
  // All console printing is centralized in the main loop.
  
  uint32_t print_counter = 0;  // Counter to throttle GNSS printing (print every 1 second)
  
  while (true) {
    // Read all sensor data from shared structure with mutex protection
    sensor_data_mutex.lock();
    int32_t enc_A = sensor_data.encoder_A;
    int32_t enc_B = sensor_data.encoder_B;
    float vbus = sensor_data.bus_voltage;
    float curr = sensor_data.current;
    char gnss_data[NMEA_BUFFER_SIZE];
    strcpy(gnss_data, sensor_data.nmea_sentence);
    sensor_data_mutex.unlock();

    // Prepare and publish sensor data message
    msgs_ifaces::msg::MainSensData msgs;
    msgs.mainsensdata_msg.mt_lf_encode_msg = enc_A;      // Left encoder count
    msgs.mainsensdata_msg.mt_rt_encode_msg = enc_B;      // Right encoder count
    msgs.mainsensdata_msg.sys_current_msg = curr;        // System current (A)
    msgs.mainsensdata_msg.sys_volt_msg = vbus;           // Bus voltage (V)
    PubSensData.publish(msgs);

    // Print main sensor debug information to console (every MAIN_LOOP_PERIOD_MS)
    printf("MotorA: %ld | MotorB: %ld | Vbus: %.3f V | I: %.3f A\r\n", 
           enc_A, enc_B, vbus, curr);
    
    // Print GNSS NMEA sentence every GNSS_PRINT_INTERVAL main loops (1 second)
    print_counter++;
    if (print_counter >= GNSS_PRINT_INTERVAL) {
        print_counter = 0;
        printf("GNSS: %s\r\n", gnss_data);
    }

    // Main loop runs at MAIN_LOOP_PERIOD_MS (4 Hz) - slower than individual sensor tasks
    // This aggregates data from faster tasks (encoders @ 10Hz, power @ 5Hz, GNSS @ 10Hz)
    ThisThread::sleep_for(chrono::milliseconds(MAIN_LOOP_PERIOD_MS));
  }

  mros2::spin();
  return 0;
}