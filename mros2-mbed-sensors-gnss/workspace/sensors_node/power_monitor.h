/**
 * @file power_monitor.h
 * @brief INA226 power monitoring interface for voltage and current sensing
 * 
 * Provides high-level API for reading bus voltage and system current
 * from a INA226 power monitor via I2C. Sampling is performed by the 
 * power monitor task at 5 Hz.
 * 
 * Thread Safety: Power readings are protected by mutex in app.cpp
 * 
 * Hardware:
 * - I2C Bus: PB_9 (SDA), PB_8 (SCL) at 400 kHz
 * - INA226 I2C Address: 0x40
 * - Shunt Resistor: 0.1 Ohms (user-configurable)
 */

#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <cstdint>

// ============================================================================
// POWER MONITOR CONFIGURATION
// ============================================================================

/** @brief INA226 I2C device address (7-bit) */
#define INA226_I2C_ADDR 0x40

/** @brief Shunt resistor value in Ohms */
#define INA226_SHUNT_RESISTOR 0.1f

/** @brief Bus voltage LSB scale: 1.25 mV per LSB */
#define INA226_BUS_VOLTAGE_LSB 1.25e-3f

/** @brief Shunt voltage LSB scale: 2.5 µV per LSB */
#define INA226_SHUNT_VOLTAGE_LSB 2.5e-6f

// ============================================================================
// INA226 REGISTER ADDRESSES
// ============================================================================

#define INA226_REG_SHUNT_VOLTAGE 0x01  // Shunt voltage (mV)
#define INA226_REG_BUS_VOLTAGE   0x02  // Bus voltage (V)
#define INA226_REG_POWER         0x03  // Power (W)
#define INA226_REG_CURRENT       0x04  // Current (A)
#define INA226_REG_CALIBRATION   0x05  // Calibration register

// ============================================================================
// POWER MONITOR INITIALIZATION & I2C CONTROL
// ============================================================================

/**
 * @brief Initialize I2C interface for INA226 power monitor
 * 
 * Configures I2C bus (PB_9=SDA, PB_8=SCL) at 400 kHz.
 * Should be called once during initialization.
 */
void power_monitor_init();

// ============================================================================
// VOLTAGE & CURRENT READING FUNCTIONS
// ============================================================================

/**
 * @brief Read bus voltage from INA226
 * 
 * Reads the bus voltage register (0x02) and converts raw ADC value
 * to physical voltage in Volts using LSB scale (1.25 mV/LSB).
 * 
 * @return Bus voltage in Volts (0.0f on I2C error)
 */
float power_monitor_read_bus_voltage();

/**
 * @brief Read system current from INA226
 * 
 * Reads the shunt voltage register (0x01) and converts raw ADC value
 * to current using the formula: I = V_shunt / R_shunt
 * 
 * Shunt LSB: 2.5 µV/LSB
 * Default R_shunt: 0.1 Ohms
 * 
 * @return System current in Amperes (0.0f on I2C error)
 */
float power_monitor_read_current();

/**
 * @brief Read both bus voltage and current in one operation
 * 
 * More efficient than calling read_bus_voltage() and read_current()
 * separately since both reads use the same I2C transaction.
 * 
 * @param[out] voltage Pointer to store bus voltage (Volts)
 * @param[out] current Pointer to store system current (Amps)
 * @return 0 on success, -1 on I2C error
 */
int power_monitor_read_voltage_and_current(float* voltage, float* current);

// ============================================================================
// LOW-LEVEL I2C HELPERS (Exported for Testing)
// ============================================================================

/**
 * @brief Read a 16-bit register from INA226
 * 
 * Performs I2C write of register address followed by I2C read of 16-bit value.
 * Value is returned in big-endian format (MSB first).
 * 
 * @param reg Register address (0x01-0x05)
 * @return 16-bit register value (0 on error)
 */
int16_t power_monitor_read_register(uint8_t reg);

#endif  // POWER_MONITOR_H
