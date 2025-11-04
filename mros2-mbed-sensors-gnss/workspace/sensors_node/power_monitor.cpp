/**
 * @file power_monitor.cpp
 * @brief INA226 power monitor implementation using Mbed OS I2C
 */

#include "power_monitor.h"
#include "mbed.h"

// ============================================================================
// I2C INTERFACE INSTANCE
// ============================================================================

/** @brief I2C bus instance (SDA=PB_9, SCL=PB_8) */
static I2C i2c_bus(PB_9, PB_8);

// ============================================================================
// POWER MONITOR INITIALIZATION
// ============================================================================

void power_monitor_init() {
    // Configure I2C bus frequency to 400 kHz
    i2c_bus.frequency(400000);
}

// ============================================================================
// LOW-LEVEL I2C REGISTER READ
// ============================================================================

int16_t power_monitor_read_register(uint8_t reg) {
    // I2C address must be 8-bit (7-bit address shifted left by 1)
    const int i2c_addr_8bit = (INA226_I2C_ADDR << 1);
    
    // Step 1: Write register address to INA226
    char cmd[1] = { reg };
    if (i2c_bus.write(i2c_addr_8bit, cmd, 1, true) != 0) {
        return 0;  // I2C write error
    }
    
    // Step 2: Read 2 bytes (16-bit value) from INA226
    char data[2];
    if (i2c_bus.read(i2c_addr_8bit, data, 2) != 0) {
        return 0;  // I2C read error
    }
    
    // Combine bytes: MSB first (big-endian)
    int16_t value = (static_cast<int16_t>(data[0]) << 8) | (static_cast<uint8_t>(data[1]));
    return value;
}

// ============================================================================
// VOLTAGE & CURRENT READING IMPLEMENTATIONS
// ============================================================================

float power_monitor_read_bus_voltage() {
    // Register 0x02 contains bus voltage in LSB units of 1.25 mV
    int16_t bus_raw = power_monitor_read_register(INA226_REG_BUS_VOLTAGE);
    
    if (bus_raw == 0 && bus_raw != power_monitor_read_register(INA226_REG_BUS_VOLTAGE)) {
        return 0.0f;  // I2C error detected
    }
    
    // Convert raw value to voltage in Volts
    float bus_voltage = bus_raw * INA226_BUS_VOLTAGE_LSB;
    return bus_voltage;
}

float power_monitor_read_current() {
    // Register 0x01 contains shunt voltage in LSB units of 2.5 µV
    int16_t shunt_raw = power_monitor_read_register(INA226_REG_SHUNT_VOLTAGE);
    
    if (shunt_raw == 0 && shunt_raw != power_monitor_read_register(INA226_REG_SHUNT_VOLTAGE)) {
        return 0.0f;  // I2C error detected
    }
    
    // Convert raw shunt voltage to Volts
    float shunt_voltage = shunt_raw * INA226_SHUNT_VOLTAGE_LSB;
    
    // Calculate current using Ohm's law: I = V / R
    float current = shunt_voltage / INA226_SHUNT_RESISTOR;
    return current;
}

int power_monitor_read_voltage_and_current(float* voltage, float* current) {
    if (voltage == NULL || current == NULL) {
        return -1;  // Invalid pointer
    }
    
    // Read both registers
    int16_t bus_raw = power_monitor_read_register(INA226_REG_BUS_VOLTAGE);
    int16_t shunt_raw = power_monitor_read_register(INA226_REG_SHUNT_VOLTAGE);
    
    // Check for I2C errors (both readings should not be zero)
    if (bus_raw == 0 && shunt_raw == 0) {
        return -1;  // Likely I2C error
    }
    
    // Convert voltage
    *voltage = bus_raw * INA226_BUS_VOLTAGE_LSB;
    
    // Convert current
    float shunt_voltage = shunt_raw * INA226_SHUNT_VOLTAGE_LSB;
    *current = shunt_voltage / INA226_SHUNT_RESISTOR;
    
    return 0;  // Success
}
