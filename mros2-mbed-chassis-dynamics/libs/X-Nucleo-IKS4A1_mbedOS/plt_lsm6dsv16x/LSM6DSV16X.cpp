/*
MIT License

Copyright (c) [2024] 
Organization: Perlatecnica APS ETS
Author: Mauro D'Angelo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "LSM6DSV16X.h"
#include <cstdint>
#include "mbed.h"

/**
 * @brief Constructor for the LSM6DSV16X class.
 * 
 * Initializes the LSM6DSV16X object with the specified I2C pins.
 * 
 * @param sda PinName of the data line for I2C communication.
 * @param scl PinName of the clock line for I2C communication.
 */
LSM6DSV16X::LSM6DSV16X(PinName sda, PinName scl) {
    i2c = new I2C(sda, scl);
    initialize();
}

/**
 * @brief Constructor for the LSM6DSV16X class.
 * 
 * Initializes the LSM6DSV16X object with the specified SPI pins.
 * 
 * @param mosi PinName of the Master Out Slave In (MOSI) line for SPI communication.
 * @param miso PinName of the Master In Slave Out (MISO) line for SPI communication.
 * @param sck PinName of the Serial Clock (SCK) line for SPI communication.
 * @param cs PinName of the Chip Select (CS) line for SPI communication.
 */
LSM6DSV16X::LSM6DSV16X(PinName mosi, PinName miso, PinName sck, PinName cs) {
    // Inizializzazione del sensore tramite SPI
    initialize();
}

/**
 * @brief Destructor for the LSM6DSV16X class.
 * 
 * Cleans up resources allocated for I2C and SPI communication.
 */
LSM6DSV16X::~LSM6DSV16X() {
    delete i2c; // Deallocate memory for I2C communication
    delete spi; // Deallocate memory for SPI communication
    delete cs_pin; // Deallocate memory for the Chip Select pin
}

/**
 * @brief Initialize the LSM6DSV16X device.
 * 
 * Configures communication settings based on the selected interface (I2C or SPI).
 */
void LSM6DSV16X::initialize() {
    if (i2c) {
        i2c->frequency(400000); // Set I2C frequency to 400kHz
    } else if (spi) {
        cs_pin->write(0); // Set Chip Select pin low to enable communication
        cs_pin->write(1); // Set Chip Select pin high to disable communication
    }
}

/**
 * @brief Get the step count.
 * 
 * @param[out] StepCount Pointer to a variable where the step count will be stored.
 * @return LSM6DSV16XStatusTypeDef Status of the operation.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_Step_Count(uint16_t *StepCount)
{
  if (stpcnt_steps_get(StepCount) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the X-axis acceleration in milli-g.
 * 
 * @param[out] Acceleration Pointer to an array where the X, Y, and Z axis acceleration values will be stored.
 *                          The values are in milli-g (1 g = 9.81 m/s^2).
 *                          The array should have space for at least 3 elements.
 * @return LSM6DSV16XStatusTypeDef Status of the operation.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_X_Axes(int32_t *Acceleration)
{
  lsm6dsv16x_axis3bit16_t data_raw;
  float sensitivity = Convert_X_Sensitivity(acc_fs);

  /* Read raw data values. */
  if (acceleration_raw_get(data_raw.i16bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Get LSM6DSV16X actual sensitivity. */
  if (sensitivity == 0.0f) {
    return LSM6DSV16X_ERROR;
  }

  /* Calculate the data. */
  Acceleration[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  Acceleration[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  Acceleration[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSV16X_OK;
} 

/**
 * @brief Set the gyroscope output data rate (ODR) when the gyroscope is disabled.
 * 
 * This method sets the gyroscope output data rate (ODR) to the specified value when the gyroscope is disabled.
 * 
 * @param Odr The desired gyroscope output data rate (ODR) in Hz.
 * @return LSM6DSV16XStatusTypeDef Indicates whether the operation was successful or encountered an error.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_G_ODR_When_Disabled(float Odr)
{
  gyro_odr = (Odr <=    7.5f) ? LSM6DSV16X_ODR_AT_7Hz5
             : (Odr <=   15.0f) ? LSM6DSV16X_ODR_AT_15Hz
             : (Odr <=   30.0f) ? LSM6DSV16X_ODR_AT_30Hz
             : (Odr <=   60.0f) ? LSM6DSV16X_ODR_AT_60Hz
             : (Odr <=  120.0f) ? LSM6DSV16X_ODR_AT_120Hz
             : (Odr <=  240.0f) ? LSM6DSV16X_ODR_AT_240Hz
             : (Odr <=  480.0f) ? LSM6DSV16X_ODR_AT_480Hz
             : (Odr <=  960.0f) ? LSM6DSV16X_ODR_AT_960Hz
             : (Odr <= 1920.0f) ? LSM6DSV16X_ODR_AT_1920Hz
             : (Odr <= 3840.0f) ? LSM6DSV16X_ODR_AT_3840Hz
             :                    LSM6DSV16X_ODR_AT_7680Hz;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the gyroscope raw data when data is available.
 * 
 * This method waits until gyroscope data is available before retrieving the raw data.
 * 
 * @param Value Pointer to an array to store the raw gyroscope data.
 * @return LSM6DSV16XStatusTypeDef Indicates whether the operation was successful or encountered an error.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_G_AxesRaw_When_Aval(int16_t *Value)
{
  lsm6dsv16x_data_ready_t drdy;
  do {
    if (flag_data_ready_get(&drdy) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
  } while (!drdy.drdy_gy);

  if (Get_G_AxesRaw(Value) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Set the gyroscope output data rate (ODR) when the gyroscope is enabled.
 * 
 * @param Odr The desired output data rate (ODR) in Hz.
 * @return LSM6DSV16XStatusTypeDef Indicates whether the operation was successful or encountered an error.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_G_ODR_When_Enabled(float Odr)
{
  lsm6dsv16x_data_rate_t new_odr;

  new_odr = (Odr <=    7.5f) ? LSM6DSV16X_ODR_AT_7Hz5
            : (Odr <=   15.0f) ? LSM6DSV16X_ODR_AT_15Hz
            : (Odr <=   30.0f) ? LSM6DSV16X_ODR_AT_30Hz
            : (Odr <=   60.0f) ? LSM6DSV16X_ODR_AT_60Hz
            : (Odr <=  120.0f) ? LSM6DSV16X_ODR_AT_120Hz
            : (Odr <=  240.0f) ? LSM6DSV16X_ODR_AT_240Hz
            : (Odr <=  480.0f) ? LSM6DSV16X_ODR_AT_480Hz
            : (Odr <=  960.0f) ? LSM6DSV16X_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? LSM6DSV16X_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? LSM6DSV16X_ODR_AT_3840Hz
            :                    LSM6DSV16X_ODR_AT_7680Hz;

  return (LSM6DSV16XStatusTypeDef) gy_data_rate_set(new_odr);
}

/**
 * @brief Convert the gyroscope full-scale sensitivity to a floating-point value.
 * 
 * @param full_scale The full-scale range of the gyroscope.
 * @return float The sensitivity value corresponding to the given full-scale range.
 */
float LSM6DSV16X::Convert_G_Sensitivity(lsm6dsv16x_gy_full_scale_t full_scale)
{
  float Sensitivity = 0.0f;
  /* Store the sensitivity based on actual full scale. */
  switch (full_scale) {
    case LSM6DSV16X_125dps:
      Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_125DPS;
      break;

    case LSM6DSV16X_250dps:
      Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_250DPS;
      break;

    case LSM6DSV16X_500dps:
      Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_500DPS;
      break;

    case LSM6DSV16X_1000dps:
      Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_1000DPS;
      break;

    case LSM6DSV16X_2000dps:
      Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_2000DPS;
      break;

    case LSM6DSV16X_4000dps:
      Sensitivity = LSM6DSV16X_GYRO_SENSITIVITY_FS_4000DPS;
      break;
  }
  return Sensitivity;
}


/**
 * @brief Get the raw accelerometer data along the X, Y, and Z axes.
 * 
 * @param Value Pointer to an array to store the raw data values [X, Y, Z].
 * @return LSM6DSV16XStatusTypeDef Status of the operation.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_X_AxesRaw(int16_t *Value)
{
  lsm6dsv16x_axis3bit16_t data_raw;

  /* Read raw data values. */
  if (acceleration_raw_get(data_raw.i16bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LSM6DSV16X_OK;
}

/**
 * @brief Initialize the LSM6DSV16X device.
 * 
 * Configures various settings of the LSM6DSV16X sensor upon startup.
 * 
 * @return Status of the initialization process.
 *         - LSM6DSV16X_OK if initialization is successful.
 *         - LSM6DSV16X_ERROR if initialization fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::begin()
{
  if (spi) {
    // Configure CS pin
    //pinMode(cs_pin, OUTPUT);
    //digitalWrite(cs_pin, HIGH);
  }

  /* Enable register address auto-increment during multiple byte access with a serial interface. */
  if (auto_increment_set(PROPERTY_ENABLE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable Block Data Update (BDU). */
  if (Enable_Block_Data_Update() != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* FIFO mode selection. */
  if (fifo_mode_set(LSM6DSV16X_BYPASS_MODE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Select default output data rate for accelerometer. */
  acc_odr = LSM6DSV16X_ODR_AT_120Hz;

  /* Output data rate selection for accelerometer - power down. */
  if (xl_data_rate_set(LSM6DSV16X_ODR_OFF) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Full scale selection for accelerometer. */
  if (xl_full_scale_set(LSM6DSV16X_2g) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Select default output data rate for gyroscope. */
  gyro_odr = LSM6DSV16X_ODR_AT_120Hz;

  /* Output data rate selection for gyroscope - power down. */
  if (gy_data_rate_set(LSM6DSV16X_ODR_OFF) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Full scale selection for gyroscope. */
  if (gy_full_scale_set(LSM6DSV16X_2000dps) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  initialized = 1U;

  return LSM6DSV16X_OK;
}


/**
 * @brief Reset the FIFO of the LSM6DSV16X device.
 * 
 * Resets the FIFO and sets the FIFO mode to the specified mode.
 * 
 * @return Status of the FIFO reset operation.
 *         - LSM6DSV16X_OK if the FIFO reset is successful.
 *         - LSM6DSV16X_ERROR if the FIFO reset fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Reset()
{
  if (fifo_mode_set(LSM6DSV16X_BYPASS_MODE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (fifo_mode_set(fifo_mode) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}


/**
 * @brief Get the full status of the FIFO in the LSM6DSV16X device.
 * 
 * Retrieves the full status of the FIFO and stores it in the provided variable.
 * 
 * @param[out] Status Pointer to a variable where the full status of the FIFO will be stored.
 *                    - 0x00 if FIFO is not full.
 *                    - 0x01 if FIFO is full.
 * 
 * @return Status of the FIFO full status retrieval operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_Full_Status(uint8_t *Status)
{
  lsm6dsv16x_fifo_status2_t val;

  if (readRegister(LSM6DSV16X_FIFO_STATUS2, (uint8_t *)&val, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *Status = val.fifo_full_ia;

  return LSM6DSV16X_OK;
}


/**
 * @brief Set the interrupt configuration for FIFO full on INT1 pin.
 * 
 * Configures the interrupt settings for FIFO full on the INT1 pin of the LSM6DSV16X device.
 * 
 * @param Status Status to set for FIFO full interrupt on INT1 pin.
 *               - 0x00 to disable FIFO full interrupt on INT1 pin.
 *               - 0x01 to enable FIFO full interrupt on INT1 pin.
 * 
 * @return Status of the interrupt configuration setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Set_INT1_FIFO_Full(uint8_t Status)
{
  lsm6dsv16x_int1_ctrl_t reg;

  if (readRegister(LSM6DSV16X_INT1_CTRL, (uint8_t *)&reg, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  reg.int1_fifo_full = Status;

  if (writeRegister(LSM6DSV16X_INT1_CTRL, (uint8_t *)&reg, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}


/**
 * @brief Set the interrupt configuration for FIFO full on INT2 pin.
 * 
 * Configures the interrupt settings for FIFO full on the INT2 pin of the LSM6DSV16X device.
 * 
 * @param Status Status to set for FIFO full interrupt on INT2 pin.
 *               - 0x00 to disable FIFO full interrupt on INT2 pin.
 *               - 0x01 to enable FIFO full interrupt on INT2 pin.
 * 
 * @return Status of the interrupt configuration setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Set_INT2_FIFO_Full(uint8_t Status)
{
  lsm6dsv16x_int2_ctrl_t reg;

  if (readRegister(LSM6DSV16X_INT2_CTRL, (uint8_t *)&reg, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  reg.int2_fifo_full = Status;

  if (writeRegister(LSM6DSV16X_INT2_CTRL, (uint8_t *)&reg, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Set the watermark level for the FIFO.
 * 
 * Sets the watermark level for the FIFO of the LSM6DSV16X device.
 * 
 * @param Watermark Watermark level to set for the FIFO.
 * 
 * @return Status of the watermark level setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Set_Watermark_Level(uint8_t Watermark)
{
  return (LSM6DSV16XStatusTypeDef) fifo_watermark_set(Watermark);
}

/**
 * @brief Set the stop on FIFO threshold status.
 * 
 * Configures whether to stop collecting data when the FIFO threshold is reached.
 * 
 * @param Status Status to set for stop on FIFO threshold.
 *               - 0x00 to disable stop on FIFO threshold.
 *               - 0x01 to enable stop on FIFO threshold.
 * 
 * @return Status of the stop on FIFO threshold setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Set_Stop_On_Fth(uint8_t Status)
{
  return (LSM6DSV16XStatusTypeDef) fifo_stop_on_wtm_set(Status);
}


/**
 * @brief Set the FIFO mode.
 * 
 * Sets the mode of the FIFO in the LSM6DSV16X device.
 * 
 * @param Mode Mode to set for the FIFO.
 *             - 0: Bypass mode
 *             - 1: FIFO mode
 *             - 3: Stream-to-FIFO mode
 *             - 4: Bypass-to-stream mode
 *             - 6: Stream mode
 *             - 7: Bypass-to-FIFO mode
 * 
 * @return Status of the FIFO mode setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Set_Mode(uint8_t Mode)
{
  lsm6dsv16x_fifo_mode_t newMode = LSM6DSV16X_BYPASS_MODE;

  switch (Mode) {
    case 0:
      newMode = LSM6DSV16X_BYPASS_MODE;
      break;
    case 1:
      newMode = LSM6DSV16X_FIFO_MODE;
      break;
    case 3:
      newMode = LSM6DSV16X_STREAM_TO_FIFO_MODE;
      break;
    case 4:
      newMode = LSM6DSV16X_BYPASS_TO_STREAM_MODE;
      break;
    case 6:
      newMode = LSM6DSV16X_STREAM_MODE;
      break;
    case 7:
      newMode = LSM6DSV16X_BYPASS_TO_FIFO_MODE;
      break;
    default:
      return LSM6DSV16X_ERROR;
  }
  fifo_mode = newMode;
  if (fifo_mode_set(fifo_mode) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the X-axis acceleration data from FIFO.
 * 
 * Retrieves the X-axis acceleration data from the FIFO of the LSM6DSV16X device.
 * 
 * @param[out] Acceleration Pointer to an array where the X-axis acceleration data will be stored.
 * 
 * @return Status of the X-axis acceleration data retrieval operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_X_Axes(int32_t *Acceleration)
{
  lsm6dsv16x_axis3bit16_t data_raw;
  float sensitivity = Convert_X_Sensitivity(acc_fs);
  float acceleration_float[3];

  if (FIFO_Get_Data(data_raw.u8bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (sensitivity == 0.0f) {
    return LSM6DSV16X_ERROR;
  }
  acceleration_float[0] = (float)data_raw.i16bit[0] * sensitivity;
  acceleration_float[1] = (float)data_raw.i16bit[1] * sensitivity;
  acceleration_float[2] = (float)data_raw.i16bit[2] * sensitivity;

  Acceleration[0] = (int32_t)acceleration_float[0];
  Acceleration[1] = (int32_t)acceleration_float[1];
  Acceleration[2] = (int32_t)acceleration_float[2];

  return LSM6DSV16X_OK;
}


/**
 * @brief Set the batch data rate for X-axis acceleration in FIFO.
 * 
 * Sets the batch data rate for X-axis acceleration data in the FIFO of the LSM6DSV16X device.
 * 
 * @param Bdr Batch data rate to set for X-axis acceleration.
 * 
 * @return Status of the batch data rate setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Set_X_BDR(float Bdr)
{
  lsm6dsv16x_fifo_xl_batch_t new_bdr;

  new_bdr = (Bdr <= 0.0f) ? LSM6DSV16X_XL_NOT_BATCHED
            : (Bdr <= 1.8f) ? LSM6DSV16X_XL_BATCHED_AT_1Hz875
            : (Bdr <= 7.5f) ? LSM6DSV16X_XL_BATCHED_AT_7Hz5
            : (Bdr <= 15.0f) ? LSM6DSV16X_XL_BATCHED_AT_15Hz
            : (Bdr <= 30.0f) ? LSM6DSV16X_XL_BATCHED_AT_30Hz
            : (Bdr <= 60.0f) ? LSM6DSV16X_XL_BATCHED_AT_60Hz
            : (Bdr <= 120.0f) ? LSM6DSV16X_XL_BATCHED_AT_120Hz
            : (Bdr <= 240.0f) ? LSM6DSV16X_XL_BATCHED_AT_240Hz
            : (Bdr <= 480.0f) ? LSM6DSV16X_XL_BATCHED_AT_480Hz
            : (Bdr <= 960.0f) ? LSM6DSV16X_XL_BATCHED_AT_960Hz
            : (Bdr <= 1920.0f) ? LSM6DSV16X_XL_BATCHED_AT_1920Hz
            : (Bdr <= 3840.0f) ? LSM6DSV16X_XL_BATCHED_AT_3840Hz
            : LSM6DSV16X_XL_BATCHED_AT_7680Hz;

  return (LSM6DSV16XStatusTypeDef) fifo_xl_batch_set(new_bdr);
}

/**
 * @brief Get the gyroscope data from FIFO.
 * 
 * Retrieves the gyroscope data from the FIFO of the LSM6DSV16X device.
 * 
 * @param[out] AngularVelocity Pointer to an array where the gyroscope data will be stored.
 * 
 * @return Status of the gyroscope data retrieval operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_G_Axes(int32_t *AngularVelocity)
{
  lsm6dsv16x_axis3bit16_t data_raw;
  float sensitivity = Convert_G_Sensitivity(gyro_fs);
  float angular_velocity_float[3];

  if (FIFO_Get_Data(data_raw.u8bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (sensitivity == 0.0f) {
    return LSM6DSV16X_ERROR;
  }

  angular_velocity_float[0] = (float)data_raw.i16bit[0] * sensitivity;
  angular_velocity_float[1] = (float)data_raw.i16bit[1] * sensitivity;
  angular_velocity_float[2] = (float)data_raw.i16bit[2] * sensitivity;

  AngularVelocity[0] = (int32_t)angular_velocity_float[0];
  AngularVelocity[1] = (int32_t)angular_velocity_float[1];
  AngularVelocity[2] = (int32_t)angular_velocity_float[2];

  return LSM6DSV16X_OK;
}

/**
 * @brief Set the batch data rate for gyroscope in FIFO.
 * 
 * Sets the batch data rate for gyroscope data in the FIFO of the LSM6DSV16X device.
 * 
 * @param Bdr Batch data rate to set for gyroscope.
 * 
 * @return Status of the batch data rate setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Set_G_BDR(float Bdr)
{
  lsm6dsv16x_fifo_gy_batch_t new_bdr;

  new_bdr = (Bdr <= 0.0f) ? LSM6DSV16X_GY_NOT_BATCHED
            : (Bdr <= 1.8f) ? LSM6DSV16X_GY_BATCHED_AT_1Hz875
            : (Bdr <= 7.5f) ? LSM6DSV16X_GY_BATCHED_AT_7Hz5
            : (Bdr <= 15.0f) ? LSM6DSV16X_GY_BATCHED_AT_15Hz
            : (Bdr <= 30.0f) ? LSM6DSV16X_GY_BATCHED_AT_30Hz
            : (Bdr <= 60.0f) ? LSM6DSV16X_GY_BATCHED_AT_60Hz
            : (Bdr <= 120.0f) ? LSM6DSV16X_GY_BATCHED_AT_120Hz
            : (Bdr <= 240.0f) ? LSM6DSV16X_GY_BATCHED_AT_240Hz
            : (Bdr <= 480.0f) ? LSM6DSV16X_GY_BATCHED_AT_480Hz
            : (Bdr <= 960.0f) ? LSM6DSV16X_GY_BATCHED_AT_960Hz
            : (Bdr <= 1920.0f) ? LSM6DSV16X_GY_BATCHED_AT_1920Hz
            : (Bdr <= 3840.0f) ? LSM6DSV16X_GY_BATCHED_AT_3840Hz
            : LSM6DSV16X_GY_BATCHED_AT_7680Hz;

  return (LSM6DSV16XStatusTypeDef) fifo_gy_batch_set(new_bdr);
}


/**
 * @brief Get the status of the FIFO.
 * 
 * Retrieves the status of the FIFO in the LSM6DSV16X device.
 * 
 * @param[out] Status Pointer to a structure where the FIFO status will be stored.
 * 
 * @return Status of the FIFO status retrieval operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_Status(lsm6dsv16x_fifo_status_t *Status)
{
  return (LSM6DSV16XStatusTypeDef)fifo_status_get(Status);
}

/**
 * @brief Get the gravity vector data from FIFO.
 * 
 * Retrieves the gravity vector data from the FIFO of the LSM6DSV16X device.
 * 
 * @param[out] gvec Pointer to an array where the gravity vector data will be stored.
 * 
 * @return Status of the gravity vector data retrieval operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_Gravity_Vector(float *gvec)
{
  lsm6dsv16x_axis3bit16_t data_raw;

  if (FIFO_Get_Data(data_raw.u8bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  gvec[0] = from_sflp_to_mg(data_raw.i16bit[0]);
  gvec[1] = from_sflp_to_mg(data_raw.i16bit[1]);
  gvec[2] = from_sflp_to_mg(data_raw.i16bit[2]);

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the gyroscope bias data from FIFO.
 * 
 * Retrieves the gyroscope bias data from the FIFO of the LSM6DSV16X device.
 * 
 * @param[out] gbias Pointer to an array where the gyroscope bias data will be stored.
 * 
 * @return Status of the gyroscope bias data retrieval operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_Gyroscope_Bias(float *gbias)
{
  lsm6dsv16x_axis3bit16_t data_raw;

  if (FIFO_Get_Data(data_raw.u8bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  gbias[0] = from_fs125_to_mdps(data_raw.i16bit[0]);
  gbias[1] = from_fs125_to_mdps(data_raw.i16bit[1]);
  gbias[2] = from_fs125_to_mdps(data_raw.i16bit[2]);

  return LSM6DSV16X_OK;
}

/**
 * @brief Enable timestamp in FIFO.
 * 
 * Enables the timestamp feature in the FIFO of the LSM6DSV16X device.
 * 
 * @return Status of the timestamp enabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Enable_Timestamp()
{
  return (LSM6DSV16XStatusTypeDef)timestamp_set(PROPERTY_ENABLE);
}

/**
 * @brief Disable timestamp in FIFO.
 * 
 * Disables the timestamp feature in the FIFO of the LSM6DSV16X device.
 * 
 * @return Status of the timestamp disabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Disable_Timestamp()
{
  return (LSM6DSV16XStatusTypeDef)timestamp_set(PROPERTY_DISABLE);
}

/**
 * @brief Set the timestamp decimation in FIFO.
 * 
 * Sets the decimation factor for the timestamp in the FIFO of the LSM6DSV16X device.
 * 
 * @param decimation Decimation factor to set for the timestamp.
 * 
 * @return Status of the timestamp decimation setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Set_Timestamp_Decimation(uint8_t decimation)
{
  return (LSM6DSV16XStatusTypeDef)fifo_timestamp_batch_set((lsm6dsv16x_fifo_timestamp_batch_t)decimation);
}

/**
 * @brief Get the timestamp from FIFO.
 * 
 * Retrieves the timestamp data from the FIFO of the LSM6DSV16X device.
 * 
 * @param[out] timestamp Pointer to a variable where the timestamp will be stored.
 * 
 * @return Status of the timestamp data retrieval operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_Timestamp(uint32_t *timestamp)
{
  uint32_t raw_data[2]; //first is timestamp second is half full of meta data
  if (FIFO_Get_Data((uint8_t *)raw_data) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  *timestamp = raw_data[0];
  return LSM6DSV16X_OK;
}

/**
 * @brief Enable Wake-Up Detection.
 * 
 * Enables the wake-up detection feature in the LSM6DSV16X device.
 * 
 * @param IntPin Selects the interrupt pin for wake-up event.
 * 
 * @return Status of the wake-up detection enabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Wake_Up_Detection(LSM6DSV16X_SensorIntPin_t IntPin)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;
  lsm6dsv16x_functions_enable_t functions_enable;

  /* Output Data Rate selection */
  if (Set_X_ODR(480) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set wake-up threshold */
  if (Set_Wake_Up_Threshold(63) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set wake-up duration */
  if (Set_Wake_Up_Duration(0) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable wake-up event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV16X_INT1_PIN:
      if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val1.int1_wu = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    case LSM6DSV16X_INT2_PIN:
      if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val2.int2_wu = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Disable Wake-Up Detection.
 * 
 * Disables the wake-up detection feature in the LSM6DSV16X device.
 * 
 * @return Status of the wake-up detection disabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_Wake_Up_Detection()
{
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;

  /* Disable wake-up event on both INT1 and INT2 pins */
  if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  val1.int1_wu = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  val2.int2_wu = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Reset wake-up threshold */
  if (Set_Wake_Up_Threshold(0) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Reset wake-up duration */
  if (Set_Wake_Up_Duration(0) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Set Wake-Up Threshold.
 * 
 * Sets the wake-up threshold value for the LSM6DSV16X device.
 * 
 * @param Threshold Wake-up threshold value to be set.
 * 
 * @return Status of the wake-up threshold setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_Wake_Up_Threshold(uint32_t Threshold)
{
  lsm6dsv16x_act_thresholds_t wake_up_ths;

  if (act_thresholds_get(&wake_up_ths) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  wake_up_ths.threshold = Threshold;

  if (act_thresholds_set(&wake_up_ths) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Set Wake-Up Duration.
 * 
 * Sets the wake-up duration value for the LSM6DSV16X device.
 * 
 * @param Duration Wake-up duration value to be set.
 * 
 * @return Status of the wake-up duration setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_Wake_Up_Duration(uint8_t Duration)
{
  lsm6dsv16x_act_wkup_time_windows_t dur_t;

  if (act_wkup_time_windows_get(&dur_t) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  dur_t.shock = Duration;

  if (act_wkup_time_windows_set(dur_t) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}



/**
 * @brief Enable Free Fall Detection.
 * 
 * Enables the free fall detection feature in the LSM6DSV16X device.
 * 
 * @param IntPin Specifies the interrupt pin for free fall detection.
 * 
 * @return Status of the free fall detection enabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Free_Fall_Detection(LSM6DSV16X_SensorIntPin_t IntPin)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;
  lsm6dsv16x_functions_enable_t functions_enable;

  /* Output Data Rate selection */
  if (Set_X_ODR(480) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set free fall duration */
  if (Set_Free_Fall_Duration(3) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set free fall threshold */
  if (Set_Free_Fall_Threshold(3) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable free fall event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV16X_INT1_PIN:
      if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val1.int1_ff = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    case LSM6DSV16X_INT2_PIN:
      if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val2.int2_ff = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Disable Free Fall Detection.
 * 
 * Disables the free fall detection feature in the LSM6DSV16X device.
 * 
 * @return Status of the free fall detection disabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_Free_Fall_Detection()
{
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;

  /* Disable free fall event on both INT1 and INT2 pins */
  if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  val1.int1_ff = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  val2.int2_ff = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Reset free fall threshold */
  if (Set_Free_Fall_Threshold(0) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Reset free fall duration */
  if (Set_Free_Fall_Duration(0) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}


/**
 * @brief Set Free Fall Threshold.
 * 
 * Sets the threshold for free fall detection in the LSM6DSV16X device.
 * 
 * @param Threshold The threshold value to be set. It should be one of the predefined threshold values:
 *                  - LSM6DSV16X_156_mg
 *                  - LSM6DSV16X_219_mg
 *                  - LSM6DSV16X_250_mg
 *                  - LSM6DSV16X_312_mg
 *                  - LSM6DSV16X_344_mg
 *                  - LSM6DSV16X_406_mg
 *                  - LSM6DSV16X_469_mg
 *                  - LSM6DSV16X_500_mg
 * 
 * @return Status of the free fall threshold setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_Free_Fall_Threshold(uint8_t Threshold)
{
  lsm6dsv16x_ff_thresholds_t val;
  switch (Threshold) {
    case LSM6DSV16X_156_mg:
      val = LSM6DSV16X_156_mg;
      break;

    case LSM6DSV16X_219_mg:
      val = LSM6DSV16X_219_mg;
      break;

    case LSM6DSV16X_250_mg:
      val = LSM6DSV16X_250_mg;
      break;

    case LSM6DSV16X_312_mg:
      val = LSM6DSV16X_312_mg;
      break;

    case LSM6DSV16X_344_mg:
      val = LSM6DSV16X_344_mg;
      break;

    case LSM6DSV16X_406_mg:
      val = LSM6DSV16X_406_mg;
      break;

    case LSM6DSV16X_469_mg:
      val = LSM6DSV16X_469_mg;
      break;

    case LSM6DSV16X_500_mg:
      val = LSM6DSV16X_500_mg;
      break;

    default:
      val = LSM6DSV16X_156_mg;
      break;
  }

  /* Set free fall threshold. */
  if (ff_thresholds_set(val) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Set Free Fall Duration.
 * 
 * Sets the duration for free fall detection in the LSM6DSV16X device.
 * 
 * @param Duration The duration value to be set, in number of samples.
 * 
 * @return Status of the free fall duration setting operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_Free_Fall_Duration(uint8_t Duration)
{
  if (ff_time_windows_set(Duration) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Enable Pedometer.
 * 
 * Enables the pedometer algorithm in the LSM6DSV16X device.
 * 
 * @param IntPin The interrupt pin to which the pedometer interrupt will be routed.
 * 
 * @return Status of the pedometer enabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Pedometer(LSM6DSV16X_SensorIntPin_t IntPin)
{
  lsm6dsv16x_stpcnt_mode_t mode;
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;
  lsm6dsv16x_emb_func_int1_t emb_func_int1;
  lsm6dsv16x_emb_func_int2_t emb_func_int2;

  /* Output Data Rate selection */
  if (Set_X_ODR(30) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (stpcnt_mode_get(&mode) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable pedometer algorithm. */
  mode.step_counter_enable = PROPERTY_ENABLE;
  mode.false_step_rej = PROPERTY_DISABLE;

  /* Turn on embedded features */
  if (stpcnt_mode_set(mode) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable pedometer event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV16X_INT1_PIN:
      /* Enable access to embedded functions registers. */
      if (mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Step detector interrupt driven to INT1 pin */
      if (readRegister(LSM6DSV16X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      emb_func_int1.int1_step_detector = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Disable access to embedded functions registers */
      if (mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val1.int1_emb_func = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    case LSM6DSV16X_INT2_PIN:
      /* Enable access to embedded functions registers. */
      if (mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Step detector interrupt driven to INT1 pin */
      if (readRegister(LSM6DSV16X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      emb_func_int2.int2_step_detector = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Disable access to embedded functions registers */
      if (mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val2.int2_emb_func = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    default:
      return LSM6DSV16X_ERROR;
      break;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Enable Tilt Detection.
 * 
 * Enables tilt detection in the LSM6DSV16X device.
 * 
 * @param IntPin The interrupt pin to which the tilt interrupt will be routed.
 * 
 * @return Status of the tilt detection enabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Tilt_Detection(LSM6DSV16X_SensorIntPin_t IntPin)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;
  lsm6dsv16x_emb_func_en_a_t emb_func_en_a;
  lsm6dsv16x_emb_func_int1_t emb_func_int1;
  lsm6dsv16x_emb_func_int2_t emb_func_int2;

  /* Output Data Rate selection */
  if (Set_X_ODR(30) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  switch (IntPin) {
    case LSM6DSV16X_INT1_PIN:
      /* Enable access to embedded functions registers */
      if (mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Enable tilt detection */
      if (readRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      emb_func_en_a.tilt_en = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Tilt interrupt driven to INT1 pin */
      if (readRegister(LSM6DSV16X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      emb_func_int1.int1_tilt = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Disable access to embedded functions registers */
      if (mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Enable routing the embedded functions interrupt */
      if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val1.int1_emb_func = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    case LSM6DSV16X_INT2_PIN:
      /* Enable access to embedded functions registers */
      if (mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Enable tilt detection */
      if (readRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      emb_func_en_a.tilt_en = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Tilt interrupt driven to INT2 pin */
      if (readRegister(LSM6DSV16X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      emb_func_int2.int2_tilt = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Disable access to embedded functions registers */
      if (mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      /* Enable routing the embedded functions interrupt */
      if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val2.int2_emb_func = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Disable Tilt Detection.
 * 
 * Disables tilt detection in the LSM6DSV16X device.
 * 
 * @return Status of the tilt detection disabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_Tilt_Detection()
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;

  lsm6dsv16x_emb_func_en_a_t emb_func_en_a;
  lsm6dsv16x_emb_func_int1_t emb_func_int1;
  lsm6dsv16x_emb_func_int2_t emb_func_int2;

  /* Disable emb func event on either INT1 or INT2 pin */
  if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  val1.int1_emb_func = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  val2.int2_emb_func = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable access to embedded functions registers. */
  if (mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Disable tilt detection. */
  if (readRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  emb_func_en_a.tilt_en = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Reset interrupt driven to INT1 pin. */
  if (readRegister(LSM6DSV16X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  emb_func_int1.int1_tilt = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Reset interrupt driven to INT2 pin. */
  if (readRegister(LSM6DSV16X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  emb_func_int2.int2_tilt = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Disable access to embedded functions registers. */
  if (mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return ret;
}

/**
 * @brief Enable Double Tap Detection.
 * 
 * Enables double tap detection on the LSM6DSV16X device.
 * 
 * @param IntPin The interrupt pin to which the double tap interrupt will be routed.
 * 
 * @return Status of the double tap detection enabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Double_Tap_Detection(LSM6DSV16X_SensorIntPin_t IntPin)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;
  lsm6dsv16x_functions_enable_t functions_enable;

  lsm6dsv16x_tap_dur_t tap_dur;
  lsm6dsv16x_tap_cfg0_t tap_cfg0;
  lsm6dsv16x_tap_ths_6d_t tap_ths_6d;

  /* Enable tap detection on Z-axis. */
  if (readRegister(LSM6DSV16X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  tap_cfg0.tap_z_en = 0x01U;

  if (writeRegister(LSM6DSV16X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set Z-axis threshold. */
  if (readRegister(LSM6DSV16X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  tap_ths_6d.tap_ths_z = 0x03U;

  if (writeRegister(LSM6DSV16X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set quiet shock and duration. */
  if (readRegister(LSM6DSV16X_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  tap_dur.dur = (uint8_t)0x03U;
  tap_dur.quiet = (uint8_t)0x02U;
  tap_dur.shock = (uint8_t)0x02U;

  if (writeRegister(LSM6DSV16X_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set tap mode. */
  if (tap_mode_set(LSM6DSV16X_BOTH_SINGLE_DOUBLE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Output Data Rate selection */
  if (Set_X_ODR(480) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(8) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable double tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV16X_INT1_PIN:
      if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val1.int1_double_tap = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    case LSM6DSV16X_INT2_PIN:
      if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val2.int2_double_tap = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}


/**
 * @brief Disable Double Tap Detection.
 * 
 * Disables double tap detection on the LSM6DSV16X device.
 * 
 * @return Status of the double tap detection disabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_Double_Tap_Detection()
{
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;

  lsm6dsv16x_tap_dur_t tap_dur;
  lsm6dsv16x_tap_cfg0_t tap_cfg0;
  lsm6dsv16x_tap_ths_6d_t tap_ths_6d;

  /* Disable double tap event on both INT1 and INT2 pins */
  if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  val1.int1_ff = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  val2.int2_ff = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Disable tap detection on Z-axis. */
  if (readRegister(LSM6DSV16X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  tap_cfg0.tap_z_en = 0x0U;

  if (writeRegister(LSM6DSV16X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Reset Z-axis threshold. */
  if (readRegister(LSM6DSV16X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  tap_ths_6d.tap_ths_z = 0x0U;

  if (writeRegister(LSM6DSV16X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Reset quiet shock and duration. */
  if (readRegister(LSM6DSV16X_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  tap_dur.dur = (uint8_t)0x0U;
  tap_dur.quiet = (uint8_t)0x0U;
  tap_dur.shock = (uint8_t)0x0U;

  if (writeRegister(LSM6DSV16X_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set tap mode. */
  if (tap_mode_set(LSM6DSV16X_ONLY_SINGLE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Enable Single Tap Detection.
 * 
 * Enables single tap detection on the LSM6DSV16X device.
 * 
 * @param IntPin The interrupt pin to which the single tap event will be mapped.
 * 
 * @return Status of the single tap detection enabling operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Single_Tap_Detection(LSM6DSV16X_SensorIntPin_t IntPin)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;
  lsm6dsv16x_functions_enable_t functions_enable;

  lsm6dsv16x_tap_dur_t tap_dur;
  lsm6dsv16x_tap_cfg0_t tap_cfg0;
  lsm6dsv16x_tap_ths_6d_t tap_ths_6d;

  /* Output Data Rate selection */
  if (Set_X_ODR(480) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(8) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable tap detection on Z-axis. */
  if (readRegister(LSM6DSV16X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  tap_cfg0.tap_z_en = 0x01U;

  if (writeRegister(LSM6DSV16X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set Z-axis threshold. */
  if (readRegister(LSM6DSV16X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  tap_ths_6d.tap_ths_z = 0x2U;

  if (writeRegister(LSM6DSV16X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set quiet and shock time windows. */
  if (readRegister(LSM6DSV16X_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  tap_dur.quiet = (uint8_t)0x02U;
  tap_dur.shock = (uint8_t)0x01U;

  if (writeRegister(LSM6DSV16X_TAP_DUR, (uint8_t *)&tap_dur, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Set tap mode. */
  if (tap_mode_set(LSM6DSV16X_ONLY_SINGLE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable single tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSV16X_INT1_PIN:
      if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val1.int1_single_tap = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    case LSM6DSV16X_INT2_PIN:
      if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val2.int2_single_tap = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}


/**
 * @brief Get the event status of the LSM6DSV16X sensor.
 * 
 * This function retrieves various event statuses from the LSM6DSV16X sensor
 * and populates the provided LSM6DSV16X_Event_Status_t structure with the obtained information.
 * 
 * @param Status Pointer to the structure where the event statuses will be stored.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_X_Event_Status(LSM6DSV16X_Event_Status_t *Status)
{
  lsm6dsv16x_emb_func_status_t emb_func_status;
  lsm6dsv16x_wake_up_src_t wake_up_src;
  lsm6dsv16x_tap_src_t tap_src;
  lsm6dsv16x_d6d_src_t d6d_src;
  lsm6dsv16x_emb_func_src_t func_src;
  lsm6dsv16x_md1_cfg_t md1_cfg;
  lsm6dsv16x_md2_cfg_t md2_cfg;
  lsm6dsv16x_emb_func_int1_t int1_ctrl;
  lsm6dsv16x_emb_func_int2_t int2_ctrl;

  // Initialize Status structure
  (void)memset((void *)Status, 0x0, sizeof(LSM6DSV16X_Event_Status_t));

  // Read various registers related to event statuses
  if (readRegister(LSM6DSV16X_WAKE_UP_SRC, (uint8_t *)&wake_up_src, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_TAP_SRC, (uint8_t *)&tap_src, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_D6D_SRC, (uint8_t *)&d6d_src, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_EMB_FUNC_SRC, (uint8_t *)&func_src, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_EMB_FUNC_INT1, (uint8_t *)&int1_ctrl, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_EMB_FUNC_INT2, (uint8_t *)&int2_ctrl, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_EMB_FUNC_STATUS, (uint8_t *)&emb_func_status, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK) != 0) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&md1_cfg, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&md2_cfg, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  // Check configuration of interrupts for different event types
  if ((md1_cfg.int1_ff == 1U) || (md2_cfg.int2_ff == 1U)) {
    if (wake_up_src.ff_ia == 1U) {
      Status->FreeFallStatus = 1;
    }
  }

  if ((md1_cfg.int1_wu == 1U) || (md2_cfg.int2_wu == 1U)) {
    if (wake_up_src.wu_ia == 1U) {
      Status->WakeUpStatus = 1;
    }
  }

  if ((md1_cfg.int1_single_tap == 1U) || (md2_cfg.int2_single_tap == 1U)) {
    if (tap_src.single_tap == 1U) {
      Status->TapStatus = 1;
    }
  }

  if ((md1_cfg.int1_double_tap == 1U) || (md2_cfg.int2_double_tap == 1U)) {
    if (tap_src.double_tap == 1U) {
      Status->DoubleTapStatus = 1;
    }
  }

  if ((md1_cfg.int1_6d == 1U) || (md2_cfg.int2_6d == 1U)) {
    if (d6d_src.d6d_ia == 1U) {
      Status->D6DOrientationStatus = 1;
    }
  }

  if (int1_ctrl.int1_step_detector == 1U || int2_ctrl.int2_step_detector == 1U) {
    if (func_src.step_detected == 1U) {
      Status->StepStatus = 1;
    }
  }

  if ((int1_ctrl.int1_tilt == 1U) || (int2_ctrl.int2_tilt == 1U)) {
    if (emb_func_status.is_tilt == 1U) {
      Status->TiltStatus = 1;
    }
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Enables 6D orientation detection on the LSM6DSV16X sensor.
 * 
 * This function configures the sensor to enable 6D orientation detection
 * and sets the corresponding interrupt pin.
 * 
 * @param IntPin Selects the interrupt pin to which the 6D orientation event will be routed.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_6D_Orientation(LSM6DSV16X_SensorIntPin_t IntPin)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;
  lsm6dsv16x_functions_enable_t functions_enable;

  // Set output data rate
  if (Set_X_ODR(480.0f) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  // Set full scale
  if (Set_X_FS(2) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  // Enable 6D orientation detection
  if (Set_6D_Orientation_Threshold(2) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  // Enable 6D orientation event on the selected interrupt pin
  switch (IntPin) {
    case LSM6DSV16X_INT1_PIN:
      if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val1.int1_6d = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    case LSM6DSV16X_INT2_PIN:
      if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      val2.int2_6d = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      if (readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }

      functions_enable.interrupts_enable = PROPERTY_ENABLE;

      if (writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1) != LSM6DSV16X_OK) {
        return LSM6DSV16X_ERROR;
      }
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Disables 6D orientation detection on the LSM6DSV16X sensor.
 * 
 * This function disables 6D orientation detection and clears the corresponding interrupt pins.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_6D_Orientation()
{
  lsm6dsv16x_md1_cfg_t val1;
  lsm6dsv16x_md2_cfg_t val2;

  // Reset threshold
  if (Set_6D_Orientation_Threshold(0) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  // Disable 6D orientation event on both INT1 and INT2 pins
  if (readRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  val1.int1_6d = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_MD1_CFG, (uint8_t *)&val1, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  val2.int2_6d = PROPERTY_DISABLE;

  if (writeRegister(LSM6DSV16X_MD2_CFG, (uint8_t *)&val2, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}


/**
 * @brief Sets the threshold for 6D orientation detection on the LSM6DSV16X sensor.
 * 
 * This function sets the threshold for 6D orientation detection based on the provided
 * threshold value.
 * 
 * @param Threshold The threshold value to set. It should be one of the following:
 *                  - 0: 80 degrees.
 *                  - 1: 70 degrees.
 *                  - 2: 60 degrees.
 *                  - 3: 50 degrees.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_6D_Orientation_Threshold(uint8_t Threshold)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_6d_threshold_t newThreshold = LSM6DSV16X_DEG_80;

  switch (Threshold) {
    case 0:
      newThreshold = LSM6DSV16X_DEG_80;
      break;
    case 1:
      newThreshold = LSM6DSV16X_DEG_70;
      break;
    case 2:
      newThreshold = LSM6DSV16X_DEG_60;
      break;
    case 3:
      newThreshold = LSM6DSV16X_DEG_50;
      break;
    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  if (ret == LSM6DSV16X_ERROR) {
    return LSM6DSV16X_ERROR;
  }

  if (_6d_threshold_set(newThreshold) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the low byte of the X axis from the 6D orientation source register.
 * 
 * This function reads the low byte of the X axis from the 6D orientation source register.
 * 
 * @param XLow Pointer to store the low byte of the X axis.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_6D_Orientation_XL(uint8_t *XLow)
{
  lsm6dsv16x_d6d_src_t data;

  if (readRegister(LSM6DSV16X_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *XLow = data.xl;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the high byte of the X axis from the 6D orientation source register.
 * 
 * This function reads the high byte of the X axis from the 6D orientation source register.
 * 
 * @param XHigh Pointer to store the high byte of the X axis.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_6D_Orientation_XH(uint8_t *XHigh)
{
  lsm6dsv16x_d6d_src_t data;

  if (readRegister(LSM6DSV16X_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *XHigh = data.xh;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the low byte of the Y axis from the 6D orientation source register.
 * 
 * This function reads the low byte of the Y axis from the 6D orientation source register.
 * 
 * @param YLow Pointer to store the low byte of the Y axis.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_6D_Orientation_YL(uint8_t *YLow)
{
  lsm6dsv16x_d6d_src_t data;

  if (readRegister(LSM6DSV16X_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *YLow = data.yl;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the high byte of the Y axis from the 6D orientation source register.
 * 
 * This function reads the high byte of the Y axis from the 6D orientation source register.
 * 
 * @param YHigh Pointer to store the high byte of the Y axis.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_6D_Orientation_YH(uint8_t *YHigh)
{
  lsm6dsv16x_d6d_src_t data;

  if (readRegister(LSM6DSV16X_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *YHigh = data.yh;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the low byte of the Z axis from the 6D orientation source register.
 * 
 * This function reads the low byte of the Z axis from the 6D orientation source register.
 * 
 * @param ZLow Pointer to store the low byte of the Z axis.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_6D_Orientation_ZL(uint8_t *ZLow)
{
  lsm6dsv16x_d6d_src_t data;

  if (readRegister(LSM6DSV16X_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *ZLow = data.zl;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the high byte of the Z axis from the 6D orientation source register.
 * 
 * This function reads the high byte of the Z axis from the 6D orientation source register.
 * 
 * @param ZHigh Pointer to store the high byte of the Z axis.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_6D_Orientation_ZH(uint8_t *ZHigh)
{
  lsm6dsv16x_d6d_src_t data;

  if (readRegister(LSM6DSV16X_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *ZHigh = data.zh;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the number of samples stored in the FIFO.
 * 
 * This function retrieves the number of samples currently stored in the FIFO buffer.
 * 
 * @param NumSamples Pointer to store the number of samples.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_Num_Samples(uint16_t *NumSamples)
{
  lsm6dsv16x_fifo_status_t status;
  if (fifo_status_get(&status) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  *NumSamples = status.fifo_level;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the tag of the next data set stored in the FIFO.
 * 
 * This function retrieves the tag of the next data set stored in the FIFO buffer.
 * 
 * @param Tag Pointer to store the tag value.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_Tag(uint8_t *Tag)
{
  lsm6dsv16x_fifo_data_out_tag_t tag_local;

  if (readRegister(LSM6DSV16X_FIFO_DATA_OUT_TAG, (uint8_t *)&tag_local, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *Tag = (uint8_t)tag_local.tag_sensor;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the rotation vector data from the FIFO.
 * 
 * This function retrieves the rotation vector data from the FIFO buffer.
 * 
 * @param rvec Pointer to an array to store the rotation vector data.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_Rotation_Vector(float *rvec)
{
  lsm6dsv16x_axis3bit16_t data_raw;

  if (FIFO_Get_Data(data_raw.u8bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  sflp2q(rvec, (uint16_t *)&data_raw.i16bit[0]);

  return LSM6DSV16X_OK;
}

/**
 * @brief Get data from the FIFO buffer.
 * 
 * This function retrieves data from the FIFO buffer.
 * 
 * @param Data Pointer to an array to store the retrieved data.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::FIFO_Get_Data(uint8_t *Data)
{
  return (LSM6DSV16XStatusTypeDef) readRegister(LSM6DSV16X_FIFO_DATA_OUT_X_L, Data, 6);
}

/**
 * @brief Set the full-scale range for the X axis.
 * 
 * This function sets the full-scale range for the X axis accelerometer.
 * 
 * @param FullScale Full-scale range to set for the X axis. Valid values are:
 *                  - 2: ±2g
 *                  - 4: ±4g
 *                  - 8: ±8g
 *                  - 16: ±16g
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_X_FS(int32_t FullScale)
{
  lsm6dsv16x_xl_full_scale_t new_fs;

  new_fs = (FullScale <= 2) ? LSM6DSV16X_2g
           : (FullScale <= 4) ? LSM6DSV16X_4g
           : (FullScale <= 8) ? LSM6DSV16X_8g
           :                    LSM6DSV16X_16g;

  if (new_fs == acc_fs) {
    return LSM6DSV16X_OK;
  }
  acc_fs = new_fs;

  if (xl_full_scale_set(acc_fs) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Set the output data rate (ODR) for the X axis accelerometer.
 * 
 * This function sets the output data rate (ODR) for the X axis accelerometer based on the specified mode.
 * 
 * @param Odr Output data rate to set.
 * @param Mode Operating mode for the accelerometer.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_X_ODR(float Odr, LSM6DSV16X_ACC_Operating_Mode_t Mode)
{
  switch (Mode) {
    case LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE: {
        if (xl_mode_set(LSM6DSV16X_XL_HIGH_PERFORMANCE_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 7.68kHz */
        Odr = (Odr <    7.5f) ?    7.5f
              : (Odr > 7680.0f) ? 7680.0f
              :                       Odr;
        break;
      }

    case LSM6DSV16X_ACC_HIGH_ACCURACY_MODE:
      // Not implemented.
      return LSM6DSV16X_ERROR;

    case LSM6DSV16X_ACC_NORMAL_MODE: {
        if (xl_mode_set(LSM6DSV16X_XL_NORMAL_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 1.92kHz */
        Odr = (Odr <    7.5f) ?    7.5f
              : (Odr > 1920.0f) ? 1920.0f
              :                       Odr;
        break;
      }

    case LSM6DSV16X_ACC_LOW_POWER_MODE1: {
        if (xl_mode_set(LSM6DSV16X_XL_LOW_POWER_2_AVG_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
        Odr = (Odr ==   1.875f) ?    Odr
              : (Odr <   15.000f) ?  15.0f
              : (Odr >  240.000f) ? 240.0f
              :                        Odr;
        break;
      }

    case LSM6DSV16X_ACC_LOW_POWER_MODE2: {
        if (xl_mode_set(LSM6DSV16X_XL_LOW_POWER_4_AVG_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
        Odr = (Odr ==   1.875f) ?    Odr
              : (Odr <   15.000f) ?  15.0f
              : (Odr >  240.000f) ? 240.0f
              :                        Odr;
        break;
      }

    case LSM6DSV16X_ACC_LOW_POWER_MODE3: {
        if (xl_mode_set(LSM6DSV16X_XL_LOW_POWER_8_AVG_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 1.875Hz;  15Hz <= Odr <= 240kHz */
        Odr = (Odr ==   1.875f) ?    Odr
              : (Odr <   15.000f) ?  15.0f
              : (Odr >  240.000f) ? 240.0f
              :                        Odr;
        break;
      }

    default:
      return LSM6DSV16X_ERROR;
  }

  if (acc_is_enabled == 1U) {
    return Set_X_ODR_When_Enabled(Odr);
  } else {
    return Set_X_ODR_When_Disabled(Odr);
  }
}

/**
 * @brief Set the output data rate (ODR) for the X axis accelerometer when disabled.
 * 
 * This function sets the output data rate (ODR) for the X axis accelerometer when it is disabled.
 * 
 * @param Odr Output data rate to set.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_X_ODR_When_Disabled(float Odr)
{
  acc_odr = (Odr <=    1.875f) ? LSM6DSV16X_ODR_AT_1Hz875
            : (Odr <=    7.5f) ? LSM6DSV16X_ODR_AT_7Hz5
            : (Odr <=   15.0f) ? LSM6DSV16X_ODR_AT_15Hz
            : (Odr <=   30.0f) ? LSM6DSV16X_ODR_AT_30Hz
            : (Odr <=   60.0f) ? LSM6DSV16X_ODR_AT_60Hz
            : (Odr <=  120.0f) ? LSM6DSV16X_ODR_AT_120Hz
            : (Odr <=  240.0f) ? LSM6DSV16X_ODR_AT_240Hz
            : (Odr <=  480.0f) ? LSM6DSV16X_ODR_AT_480Hz
            : (Odr <=  960.0f) ? LSM6DSV16X_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? LSM6DSV16X_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? LSM6DSV16X_ODR_AT_3840Hz
            :                    LSM6DSV16X_ODR_AT_7680Hz;

  return LSM6DSV16X_OK;
}

/**
 * @brief Set the output data rate (ODR) for the X axis accelerometer when enabled.
 * 
 * This function sets the output data rate (ODR) for the X axis accelerometer when it is enabled.
 * 
 * @param Odr Output data rate to set.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_X_ODR_When_Enabled(float Odr)
{
  lsm6dsv16x_data_rate_t new_odr;

  new_odr = (Odr <=    1.875f) ? LSM6DSV16X_ODR_AT_1Hz875
            : (Odr <=    7.5f) ? LSM6DSV16X_ODR_AT_7Hz5
            : (Odr <=   15.0f) ? LSM6DSV16X_ODR_AT_15Hz
            : (Odr <=   30.0f) ? LSM6DSV16X_ODR_AT_30Hz
            : (Odr <=   60.0f) ? LSM6DSV16X_ODR_AT_60Hz
            : (Odr <=  120.0f) ? LSM6DSV16X_ODR_AT_120Hz
            : (Odr <=  240.0f) ? LSM6DSV16X_ODR_AT_240Hz
            : (Odr <=  480.0f) ? LSM6DSV16X_ODR_AT_480Hz
            : (Odr <=  960.0f) ? LSM6DSV16X_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? LSM6DSV16X_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? LSM6DSV16X_ODR_AT_3840Hz
            :                    LSM6DSV16X_ODR_AT_7680Hz;

  /* Output data rate selection. */
  if (xl_data_rate_set(new_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Enable the X-axis accelerometer.
 * 
 * This function enables the X-axis accelerometer.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_X()
{
  /* Check if the component is already enabled */
  if (acc_is_enabled == 1U) {
    return LSM6DSV16X_OK;
  }

  /* Output data rate selection. */
  if (xl_data_rate_set(acc_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  acc_is_enabled = 1U;

  return LSM6DSV16X_OK;
}

/**
 * @brief Enable the gyroscope.
 * 
 * This function enables the gyroscope.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_G()
{
  /* Check if the component is already enabled */
  if (gyro_is_enabled == 1U) {
    return LSM6DSV16X_OK;
  }

  /* Output data rate selection. */
  if (gy_data_rate_set(gyro_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  gyro_is_enabled = 1U;

  return LSM6DSV16X_OK;
}

/**
 * @brief Disable the LSM6DSV16X sensor.
 * 
 * This function disables both the accelerometer and the gyroscope, resets the output data rate,
 * and resets the initialization flag.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::end()
{
  /* Disable the component */
  if (Disable_X() != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (Disable_G() != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Reset output data rate. */
  acc_odr = LSM6DSV16X_ODR_OFF;
  gyro_odr = LSM6DSV16X_ODR_OFF;

  initialized = 0U;

  return LSM6DSV16X_OK;
}

/**
 * @brief Disable the X-axis accelerometer.
 * 
 * This function disables the X-axis accelerometer.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_X()
{
  /* Check if the component is already disabled */
  if (acc_is_enabled == 0U) {
    return LSM6DSV16X_OK;
  }

  /* Get current output data rate. */
  if (xl_data_rate_get(&acc_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Output data rate selection - power down. */
  if (xl_data_rate_set(LSM6DSV16X_ODR_OFF) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  acc_is_enabled = 0U;

  return LSM6DSV16X_OK;
}

/**
 * @brief Disable the gyroscope.
 * 
 * This function disables the gyroscope.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_G()
{
  /* Check if the component is already disabled */
  if (gyro_is_enabled == 0U) {
    return LSM6DSV16X_OK;
  }

  /* Get current output data rate. */
  if (gy_data_rate_get(&gyro_odr) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Output data rate selection - power down. */
  if (gy_data_rate_set(LSM6DSV16X_ODR_OFF) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  gyro_is_enabled = 0U;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the sensitivity of the gyroscope.
 * 
 * This function retrieves the sensitivity of the gyroscope based on the current full scale selection.
 * 
 * @param[out] Sensitivity Pointer to store the sensitivity value.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_G_Sensitivity(float *Sensitivity)
{
  lsm6dsv16x_gy_full_scale_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (gy_full_scale_get(&full_scale) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *Sensitivity = Convert_G_Sensitivity(full_scale);
  if (*Sensitivity == 0.0f) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Get the output data rate (ODR) of the gyroscope.
 * 
 * This function retrieves the output data rate (ODR) of the gyroscope.
 * 
 * @param[out] Odr Pointer to store the output data rate (ODR) value in Hz.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_G_ODR(float *Odr)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_data_rate_t odr_low_level;

  /* Get current output data rate. */
  if (gy_data_rate_get(&odr_low_level) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  switch (odr_low_level) {
    case LSM6DSV16X_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSV16X_ODR_AT_7Hz5:
      *Odr = 7.5f;
      break;

    case LSM6DSV16X_ODR_AT_15Hz:
      *Odr = 15.0f;
      break;

    case LSM6DSV16X_ODR_AT_30Hz:
      *Odr = 30.0f;
      break;

    case LSM6DSV16X_ODR_AT_60Hz:
      *Odr = 60.0f;
      break;

    case LSM6DSV16X_ODR_AT_120Hz:
      *Odr = 120.0f;
      break;

    case LSM6DSV16X_ODR_AT_240Hz:
      *Odr = 240.0f;
      break;

    case LSM6DSV16X_ODR_AT_480Hz:
      *Odr = 480.0f;
      break;

    case LSM6DSV16X_ODR_AT_960Hz:
      *Odr = 960.0f;
      break;

    case LSM6DSV16X_ODR_AT_1920Hz:
      *Odr = 1920.0f;
      break;

    case LSM6DSV16X_ODR_AT_3840Hz:
      *Odr = 3840.0f;
      break;

    case LSM6DSV16X_ODR_AT_7680Hz:
      *Odr = 7680.0f;
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Set the output data rate (ODR) of the gyroscope.
 * 
 * This function sets the output data rate (ODR) of the gyroscope according to the specified mode.
 * 
 * @param[in] Odr Desired output data rate (ODR) value in Hz.
 * @param[in] Mode Operating mode of the gyroscope.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_G_ODR(float Odr, LSM6DSV16X_GYRO_Operating_Mode_t Mode)
{
  switch (Mode) {
    case LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE: {
        if (gy_mode_set(LSM6DSV16X_GY_HIGH_PERFORMANCE_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 7.68kHz */
        Odr = (Odr <    7.5f) ?    7.5f
              : (Odr > 7680.0f) ? 7680.0f
              :                       Odr;
        break;
      }

    case LSM6DSV16X_GYRO_HIGH_ACCURACY_MODE:
      // TODO: Not implemented.
      // NOTE: According to datasheet, section `6.5 High-accuracy ODR mode`:
      // "... the other sensor also has to be configured in high-accuracy ODR (HAODR) mode."
      return LSM6DSV16X_ERROR;

    case LSM6DSV16X_GYRO_SLEEP_MODE:
      // TODO: Not implemented.
      // NOTE: Unknown ODR validity for this mode
      return LSM6DSV16X_ERROR;

    case LSM6DSV16X_GYRO_LOW_POWER_MODE: {
        if (gy_mode_set(LSM6DSV16X_GY_LOW_POWER_MD) != LSM6DSV16X_OK) {
          return LSM6DSV16X_ERROR;
        }

        /* Valid ODR: 7.5Hz <= Odr <= 240kHz */
        Odr = (Odr <   7.5f) ?   7.5f
              : (Odr > 240.0f) ? 240.0f
              :                     Odr;
        break;
      }

    default:
      return LSM6DSV16X_ERROR;
  }

  if (gyro_is_enabled == 1U) {
    return Set_G_ODR_When_Enabled(Odr);
  } else {
    return Set_G_ODR_When_Disabled(Odr);
  }
}

/**
 * @brief Get the full-scale range (FS) of the gyroscope.
 * 
 * This function retrieves the current full-scale range (FS) of the gyroscope.
 * 
 * @param[out] FullScale Pointer to store the full-scale range (FS) value in dps.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_G_FS(int32_t *FullScale)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_gy_full_scale_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (gy_full_scale_get(&fs_low_level) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  switch (fs_low_level) {
    case LSM6DSV16X_125dps:
      *FullScale =  125;
      break;

    case LSM6DSV16X_250dps:
      *FullScale =  250;
      break;

    case LSM6DSV16X_500dps:
      *FullScale =  500;
      break;

    case LSM6DSV16X_1000dps:
      *FullScale = 1000;
      break;

    case LSM6DSV16X_2000dps:
      *FullScale = 2000;
      break;

    case LSM6DSV16X_4000dps:
      *FullScale = 4000;
      break;

    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Set the full-scale range (FS) of the gyroscope.
 * 
 * This function sets the full-scale range (FS) of the gyroscope to the specified value.
 * 
 * @param[in] FullScale Desired full-scale range (FS) value in dps.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_G_FS(int32_t FullScale)
{
  lsm6dsv16x_gy_full_scale_t new_fs;

  new_fs = (FullScale <= 125)  ? LSM6DSV16X_125dps
           : (FullScale <= 250)  ? LSM6DSV16X_250dps
           : (FullScale <= 500)  ? LSM6DSV16X_500dps
           : (FullScale <= 1000) ? LSM6DSV16X_1000dps
           : (FullScale <= 2000) ? LSM6DSV16X_2000dps
           :                       LSM6DSV16X_4000dps;

  if (new_fs == gyro_fs) {
    return LSM6DSV16X_OK;
  }
  gyro_fs = new_fs;

  return (LSM6DSV16XStatusTypeDef) gy_full_scale_set(gyro_fs);
}

/**
 * @brief Get the raw angular rate values from the gyroscope.
 * 
 * This function retrieves the raw angular rate values from the gyroscope without any scaling applied.
 * 
 * @param[out] Value Pointer to store the raw angular rate values [X, Y, Z].
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_G_AxesRaw(int16_t *Value)
{
  lsm6dsv16x_axis3bit16_t data_raw;

  /* Read raw data values. */
  if (angular_rate_raw_get(data_raw.i16bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Format the data. */
  Value[0] = data_raw.i16bit[0];
  Value[1] = data_raw.i16bit[1];
  Value[2] = data_raw.i16bit[2];

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the angular rate values from the gyroscope.
 * 
 * This function retrieves the angular rate values from the gyroscope, scaled according to the selected full-scale range (FS).
 * 
 * @param[out] AngularRate Pointer to store the angular rate values [X, Y, Z] in dps (degrees per second).
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_G_Axes(int32_t *AngularRate)
{
  lsm6dsv16x_axis3bit16_t data_raw;
  float sensitivity = Convert_G_Sensitivity(gyro_fs);

  /* Read raw data values. */
  if (angular_rate_raw_get(data_raw.i16bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /* Get LSM6DSV16X actual sensitivity. */
  if (sensitivity == 0.0f) {
    return LSM6DSV16X_ERROR;
  }

  /* Calculate the data. */
  AngularRate[0] = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  AngularRate[1] = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  AngularRate[2] = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSV16X_OK;
}

/**
 * @brief Get the gyroscope data ready (DRDY) status.
 * 
 * This function retrieves the gyroscope data ready (DRDY) status.
 * 
 * @param[out] Status Pointer to store the gyroscope DRDY status.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_G_DRDY_Status(uint8_t *Status)
{
  lsm6dsv16x_all_sources_t val;

  if (all_sources_get(&val) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *Status = val.drdy_gy;
  return LSM6DSV16X_OK;
}

/**
 * @brief Set the gyroscope power mode.
 * 
 * This function sets the power mode of the gyroscope.
 * 
 * @param[in] PowerMode Power mode to set for the gyroscope.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_G_Power_Mode(uint8_t PowerMode)
{
  if (gy_mode_set((lsm6dsv16x_gy_mode_t)PowerMode) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Set the gyroscope filter mode.
 * 
 * This function sets the filter mode of the gyroscope.
 * 
 * @param[in] LowHighPassFlag Flag to select low-pass or high-pass filter mode.
 * @param[in] FilterMode Filter mode to set for the gyroscope.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_G_Filter_Mode(uint8_t LowHighPassFlag, uint8_t FilterMode)
{
  if (LowHighPassFlag == 0) {
    /* Set gyroscope low_pass 1 filter-mode */
    /* Enable low-pass filter */
    if (gy_lp1_set(1) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
    if (filt_gy_lp1_bandwidth_set((lsm6dsv16x_filt_gy_lp1_bandwidth_t)FilterMode) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
  } else {
    /* Set gyroscope high_pass filter-mode */
    /* Enable high-pass filter */
    if (gy_lp1_set(0) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
    if (filt_gy_lp1_bandwidth_set((lsm6dsv16x_filt_gy_lp1_bandwidth_t)FilterMode) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Get the temperature output data rate (ODR).
 * 
 * This function retrieves the output data rate (ODR) for the temperature sensor.
 * 
 * @param[out] Odr Pointer to store the temperature output data rate (ODR) in Hz.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_Temp_ODR(float *Odr)
{
  lsm6dsv16x_fifo_ctrl4_t ctrl4;

  if (readRegister(LSM6DSV16X_FIFO_CTRL4, (uint8_t *)&ctrl4, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  switch (ctrl4.odr_t_batch) {
    case LSM6DSV16X_TEMP_NOT_BATCHED:
      *Odr = 0;
      break;
    case LSM6DSV16X_TEMP_BATCHED_AT_1Hz875:
      *Odr = 1.875f;
      break;
    case LSM6DSV16X_TEMP_BATCHED_AT_15Hz:
      *Odr = 15;
      break;
    case LSM6DSV16X_TEMP_BATCHED_AT_60Hz:
      *Odr = 60;
      break;
    default:
      break;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Set the temperature output data rate (ODR).
 * 
 * This function sets the output data rate (ODR) for the temperature sensor.
 * 
 * @param[in] Odr Temperature output data rate (ODR) to set in Hz.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_Temp_ODR(float Odr)
{
  lsm6dsv16x_fifo_ctrl4_t ctrl4;

  if (readRegister(LSM6DSV16X_FIFO_CTRL4, (uint8_t *)&ctrl4, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (Odr == 0.0F) {
    ctrl4.odr_t_batch = LSM6DSV16X_TEMP_NOT_BATCHED;
  } else if (Odr <= 1.875F) {
    ctrl4.odr_t_batch = LSM6DSV16X_TEMP_BATCHED_AT_1Hz875;
  } else if (Odr <= 15.0F) {
    ctrl4.odr_t_batch = LSM6DSV16X_TEMP_BATCHED_AT_15Hz;
  } else {
    ctrl4.odr_t_batch = LSM6DSV16X_TEMP_BATCHED_AT_60Hz;
  }

  return (LSM6DSV16XStatusTypeDef)writeRegister(
           LSM6DSV16X_FIFO_CTRL4,
           (uint8_t *)&ctrl4,
           1);
}

/**
 * @brief Get the raw temperature value.
 * 
 * This function retrieves the raw temperature value from the sensor.
 * 
 * @param[out] value Pointer to store the raw temperature value.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_Temp_Raw(int16_t *value)
{
  return (LSM6DSV16XStatusTypeDef)temperature_raw_get(value);
}

/**
 * @brief Test the IMU functionality.
 * 
 * This function tests the functionality of the IMU by performing X-axis and
 * gyroscope tests.
 * 
 * @param[in] XTestType Type of X-axis test to perform.
 * @param[in] GTestType Type of gyroscope test to perform.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the IMU test is successful.
 *         - LSM6DSV16X_ERROR if the IMU test fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Test_IMU(uint8_t XTestType, uint8_t GTestType)
{
  uint8_t whoamI;

  if (ReadID(&whoamI) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (whoamI != LSM6DSV16X_ID) {
    return LSM6DSV16X_ERROR;
  }

  if (Test_X_IMU(XTestType) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (Test_G_IMU(GTestType) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Read the device ID.
 * 
 * This function reads the device ID of the LSM6DSV16X sensor.
 * 
 * @param[out] val Pointer to store the device ID.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::ReadID(uint8_t *val)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  if (!readRegister(LSM6DSV16X_WHO_AM_I, (uint8_t *)val, 1)) {
    ret = LSM6DSV16X_ERROR;
  }
  return ret;
}

/**
 * @brief Perform X-axis accelerometer self-test.
 * 
 * This function performs the X-axis accelerometer self-test.
 * 
 * @param[in] TestType Type of self-test to perform.
 * 
 * @return Status of the self-test.
 *         - LSM6DSV16X_OK if the self-test passes.
 *         - LSM6DSV16X_ERROR if the self-test fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Test_X_IMU(uint8_t TestType)
{
  int16_t data_raw[3];
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];

  if (Device_Reset(LSM6DSV16X_RESET_CTRL_REGS) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (block_data_update_set(PROPERTY_ENABLE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /*
   * Accelerometer Self Test
   */
  if (xl_data_rate_set(LSM6DSV16X_ODR_AT_60Hz) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (xl_full_scale_set(LSM6DSV16X_4g) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  thread_sleep_for(100); // Wait for Accelerometer to stabilize;
  memset(val_st_off, 0x00, 3 * sizeof(float));
  memset(val_st_on, 0x00, 3 * sizeof(float));

  /* Ignore First Data */
  if (Get_X_AxesRaw_When_Aval(data_raw) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  for (uint8_t i = 0; i < 5; i++) {
    if (Get_X_AxesRaw_When_Aval(data_raw) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }

    /* Average the data in each axis */
    for (uint8_t j = 0; j < 3; j++) {
      val_st_off[j] += from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (uint8_t i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  if (xl_self_test_set((lsm6dsv16x_xl_self_test_t)TestType) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  thread_sleep_for(100);

  /* Ignore First Data */
  if (Get_X_AxesRaw_When_Aval(data_raw) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  for (uint8_t i = 0; i < 5; i++) {
    if (Get_X_AxesRaw_When_Aval(data_raw) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }

    /* Average the data in each axis */
    for (uint8_t j = 0; j < 3; j++) {
      val_st_on[j] += from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (uint8_t i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (uint8_t i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  for (uint8_t i = 0; i < 3; i++) {
    if ((LSM6DSV16X_MIN_ST_LIMIT_mg > test_val[i]) || (test_val[i] > LSM6DSV16X_MAX_ST_LIMIT_mg)) {
      return LSM6DSV16X_ERROR;
    }
  }

  if (xl_self_test_set(LSM6DSV16X_XL_ST_DISABLE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (xl_data_rate_set(LSM6DSV16X_ODR_OFF) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}


/**
 * @brief Get the raw accelerometer data when available.
 * 
 * This function retrieves the raw accelerometer data when available.
 * 
 * @param[out] Value Pointer to store the raw accelerometer data.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_X_AxesRaw_When_Aval(int16_t *Value)
{
  lsm6dsv16x_data_ready_t drdy;
  do {
    if (flag_data_ready_get(&drdy) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
  } while (!drdy.drdy_xl);

  if (Get_X_AxesRaw(Value) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Perform gyroscope self-test.
 * 
 * This function performs the gyroscope self-test.
 * 
 * @param[in] TestType Type of self-test to perform.
 * 
 * @return Status of the self-test.
 *         - LSM6DSV16X_OK if the self-test passes.
 *         - LSM6DSV16X_ERROR if the self-test fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Test_G_IMU(uint8_t TestType)
{
  int16_t data_raw[3];
  float test_val[3];

  if (Device_Reset(LSM6DSV16X_RESET_CTRL_REGS) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (block_data_update_set(PROPERTY_ENABLE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  /*
   * Gyroscope Self Test
   */
  if (gy_data_rate_set(LSM6DSV16X_ODR_AT_240Hz) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (gy_full_scale_set(LSM6DSV16X_2000dps) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  thread_sleep_for(100);

  /* Ignore First Data */
  if (Get_G_AxesRaw_When_Aval(data_raw) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  float val_st_off[3] = {0};
  float val_st_on[3] = {0};

  for (uint8_t i = 0; i < 5; i++) {
    if (Get_G_AxesRaw_When_Aval(data_raw) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }

    /* Average the data in each axis */
    for (uint8_t j = 0; j < 3; j++) {
      val_st_off[j] += from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (uint8_t i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  if (gy_self_test_set((lsm6dsv16x_gy_self_test_t)TestType) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  thread_sleep_for(100);

  /* Ignore First Data */
  if (Get_G_AxesRaw_When_Aval(data_raw) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  for (uint8_t i = 0; i < 5; i++) {
    if (Get_G_AxesRaw_When_Aval(data_raw) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }

    /* Average the data in each axis */
    for (uint8_t j = 0; j < 3; j++) {
      val_st_on[j] += from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (uint8_t i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (uint8_t i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (uint8_t i = 0; i < 3; i++) {
    if ((LSM6DSV16X_MIN_ST_LIMIT_mdps > test_val[i]) || (test_val[i] > LSM6DSV16X_MAX_ST_LIMIT_mdps)) {
      return LSM6DSV16X_ERROR;
    }
  }

  if (gy_self_test_set(LSM6DSV16X_GY_ST_DISABLE) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (xl_data_rate_set(LSM6DSV16X_ODR_OFF) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Enable QVAR feature.
 * 
 * This function enables the QVAR feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::QVAR_Enable()
{
  lsm6dsv16x_ctrl7_t ctrl7;

  if (readRegister(LSM6DSV16X_CTRL7, (uint8_t *)&ctrl7, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  ctrl7.ah_qvar_en = 1;
  ctrl7.int2_drdy_ah_qvar = 1;

  if (writeRegister(LSM6DSV16X_CTRL7, (uint8_t *)&ctrl7, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Disable QVAR feature.
 * 
 * This function disables the QVAR feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::QVAR_Disable()
{
  lsm6dsv16x_ctrl7_t ctrl7;

  if (readRegister(LSM6DSV16X_CTRL7, (uint8_t *)&ctrl7, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  ctrl7.ah_qvar_en = 0;
  ctrl7.int2_drdy_ah_qvar = 0;

  if (writeRegister(LSM6DSV16X_CTRL7, (uint8_t *)&ctrl7, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Get QVAR data.
 * 
 * This function retrieves QVAR data.
 * 
 * @param[out] Data Pointer to store the QVAR data.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::QVAR_GetData(float *Data)
{
  lsm6dsv16x_axis1bit16_t data_raw;
  (void)memset(data_raw.u8bit, 0x00, sizeof(int16_t));

  if (ah_qvar_raw_get(&data_raw.i16bit) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *Data = ((float)data_raw.i16bit) / LSM6DSV16X_QVAR_GAIN;
  return LSM6DSV16X_OK;
}

/**
 * @brief Get QVAR impedance.
 * 
 * This function retrieves QVAR impedance.
 * 
 * @param[out] val Pointer to store the QVAR impedance.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::QVAR_GetImpedance(uint16_t *val)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_ah_qvar_zin_t imp;

  if (ah_qvar_zin_get(&imp) != LSM6DSV16X_OK) {
    ret = LSM6DSV16X_ERROR;
  }
  switch (imp) {
    case LSM6DSV16X_2400MOhm:
      *val = 2400;
      break;
    case LSM6DSV16X_730MOhm:
      *val = 730;
      break;
    case LSM6DSV16X_300MOhm:
      *val = 300;
      break;
    case LSM6DSV16X_255MOhm:
      *val = 255;
      break;
    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief Set QVAR impedance.
 * 
 * This function sets QVAR impedance.
 * 
 * @param[in] val QVAR impedance to set.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::QVAR_SetImpedance(uint16_t val)
{
  LSM6DSV16XStatusTypeDef ret = LSM6DSV16X_OK;
  lsm6dsv16x_ah_qvar_zin_t imp;
  switch (val) {
    case 2400:
      imp = LSM6DSV16X_2400MOhm;
      break;
    case 730:
      imp = LSM6DSV16X_730MOhm;
      break;
    case 300:
      imp = LSM6DSV16X_300MOhm;
      break;
    case 255:
      imp = LSM6DSV16X_255MOhm;
      break;
    default:
      ret = LSM6DSV16X_ERROR;
      break;
  }
  if (ret != LSM6DSV16X_ERROR) {
    if (ah_qvar_zin_set(imp) != LSM6DSV16X_OK) {
      ret = LSM6DSV16X_ERROR;
    }
  }
  return ret;
}

/**
 * @brief Get QVAR status.
 * 
 * This function retrieves QVAR status.
 * 
 * @param[out] val Pointer to store the QVAR status.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::QVAR_GetStatus(uint8_t *val)
{
  lsm6dsv16x_status_reg_t status;

  if (readRegister(LSM6DSV16X_STATUS_REG, (uint8_t *)&status, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  *val = status.ah_qvarda;

  return LSM6DSV16X_OK;
}

/**
 * @brief Get MLC status.
 * 
 * This function retrieves MLC (Machine Learning Core) status.
 * 
 * @param[out] status Pointer to store the MLC status.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_MLC_Status(lsm6dsv16x_mlc_status_mainpage_t *status)
{
  if (readRegister(LSM6DSV16X_MLC_STATUS_MAINPAGE, (uint8_t *)status, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Get MLC output.
 * 
 * This function retrieves MLC output.
 * 
 * @param[out] output Pointer to store the MLC output.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Get_MLC_Output(lsm6dsv16x_mlc_out_t *output)
{
  if (mlc_out_get(output) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  return LSM6DSV16X_OK;
}

/**
 * @brief Enable Rotation Vector feature.
 * 
 * This function enables the Rotation Vector feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Rotation_Vector()
{
  lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;

  if (fifo_sflp_batch_get(&fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }
  /* Enable Rotation Vector SFLP feature */
  fifo_sflp.game_rotation = 1;

  if (fifo_sflp_batch_set(fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable SFLP low power */
  if (sflp_game_rotation_set(PROPERTY_ENABLE)) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Disable Rotation Vector feature.
 * 
 * This function disables the Rotation Vector feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_Rotation_Vector()
{
  lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;

  if (fifo_sflp_batch_get(&fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }
  /* Disable Rotation Vector SFLP feature */
  fifo_sflp.game_rotation = 0;

  if (fifo_sflp_batch_set(fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }

  /* Check if all SFLP features are disabled */
  if (!fifo_sflp.game_rotation && !fifo_sflp.gravity && !fifo_sflp.gbias) {
    /* Disable SFLP */
    if (sflp_game_rotation_set(PROPERTY_DISABLE)) {
      return LSM6DSV16X_ERROR;
    }
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Enable Gravity Vector feature.
 * 
 * This function enables the Gravity Vector feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Gravity_Vector()
{
  lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;

  if (fifo_sflp_batch_get(&fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }
  /* Enable Gravity Vector SFLP feature */
  fifo_sflp.gravity = 1;

  if (fifo_sflp_batch_set(fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable SFLP low power */
  if (sflp_game_rotation_set(PROPERTY_ENABLE)) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Disable Gravity Vector feature.
 * 
 * This function disables the Gravity Vector feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_Gravity_Vector()
{
  lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;

  if (fifo_sflp_batch_get(&fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }
  /* Disable Gravity Vector SFLP feature */
  fifo_sflp.gravity = 0;

  if (fifo_sflp_batch_set(fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }

  /* Check if all SFLP features are disabled */
  if (!fifo_sflp.game_rotation && !fifo_sflp.gravity && !fifo_sflp.gbias) {
    /* Disable SFLP */
    if (sflp_game_rotation_set(PROPERTY_DISABLE)) {
      return LSM6DSV16X_ERROR;
    }
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Enable Gyroscope Bias feature.
 * 
 * This function enables the Gyroscope Bias feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Gyroscope_Bias()
{
  lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;

  if (fifo_sflp_batch_get(&fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }
  /* Enable Gyroscope Bias SFLP feature */
  fifo_sflp.gbias = 1;

  if (fifo_sflp_batch_set(fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }

  /* Enable SFLP low power */
  if (sflp_game_rotation_set(PROPERTY_ENABLE)) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Disable Gyroscope Bias feature.
 * 
 * This function disables the Gyroscope Bias feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_Gyroscope_Bias()
{
  lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;

  if (fifo_sflp_batch_get(&fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }
  /* Disable Gyroscope Bias SFLP feature */
  fifo_sflp.gbias = 0;

  if (fifo_sflp_batch_set(fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }

  /* Check if all SFLP features are disabled */
  if (!fifo_sflp.game_rotation && !fifo_sflp.gravity && !fifo_sflp.gbias) {
    /* Disable SFLP */
    if (sflp_game_rotation_set(PROPERTY_DISABLE)) {
      return LSM6DSV16X_ERROR;
    }
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Set SFLP Batch.
 * 
 * This function sets the SFLP Batch configuration.
 * 
 * @param GameRotation Flag to enable or disable Game Rotation.
 * @param Gravity Flag to enable or disable Gravity.
 * @param gBias Flag to enable or disable Gyroscope Bias.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_SFLP_Batch(bool GameRotation, bool Gravity, bool gBias)
{
  lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
  fifo_sflp.game_rotation = GameRotation;
  fifo_sflp.gravity = Gravity;
  fifo_sflp.gbias = gBias;
  if (fifo_sflp_batch_set(fifo_sflp)) {
    return LSM6DSV16X_ERROR;
  }

  /* Check if all SFLP features are disabled */
  if (!fifo_sflp.game_rotation && !fifo_sflp.gravity && !fifo_sflp.gbias) {
    /* Disable SFLP */
    if (sflp_game_rotation_set(PROPERTY_DISABLE)) {
      return LSM6DSV16X_ERROR;
    }
  } else {
    /* Enable SFLP low power */
    if (sflp_game_rotation_set(PROPERTY_ENABLE)) {
      return LSM6DSV16X_ERROR;
    }
  }
  return LSM6DSV16X_OK;
}


/**
 * @brief Set the SFLP (Sensor Filtered Low Power) data rate.
 * 
 * This function sets the SFLP data rate based on the given parameter.
 * 
 * @param odr The desired output data rate (in Hz).
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_SFLP_ODR(float odr)
{
  lsm6dsv16x_sflp_data_rate_t rate = odr <= 15    ? LSM6DSV16X_SFLP_15Hz
                                     : odr <= 30  ? LSM6DSV16X_SFLP_30Hz
                                     : odr <= 60  ? LSM6DSV16X_SFLP_60Hz
                                     : odr <= 120 ? LSM6DSV16X_SFLP_120Hz
                                     : odr <= 240 ? LSM6DSV16X_SFLP_240Hz
                                     : LSM6DSV16X_SFLP_480Hz;

  return (LSM6DSV16XStatusTypeDef)sflp_data_rate_set(rate);
}

/**
 * @brief Set the Gyroscope Bias for SFLP (Sensor Filtered Low Power) mode.
 * 
 * This function sets the Gyroscope Bias values for SFLP mode.
 * 
 * @param x The Gyroscope Bias value for the X-axis.
 * @param y The Gyroscope Bias value for the Y-axis.
 * @param z The Gyroscope Bias value for the Z-axis.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Set_SFLP_GBIAS(float x, float y, float z)
{
  lsm6dsv16x_sflp_gbias_t val = {0, 0, 0};
  val.gbias_x = x;
  val.gbias_y = y;
  val.gbias_z = z;
  if (sflp_game_gbias_set(&val) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Reset SFLP (Sensor Filtered Low Power).
 * 
 * This function resets the SFLP mode.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Reset_SFLP(void)
{
  lsm6dsv16x_emb_func_init_a_t emb_func_init_a;

  if (mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (readRegister(LSM6DSV16X_EMB_FUNC_INIT_A, (uint8_t *)&emb_func_init_a, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  emb_func_init_a.sflp_game_init = 1;

  if (writeRegister(LSM6DSV16X_EMB_FUNC_INIT_A, (uint8_t *)&emb_func_init_a, 1) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  if (mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }
  return LSM6DSV16X_OK;
}

/**
 * @brief Enable Block Data Update.
 * 
 * This function enables the Block Data Update feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Block_Data_Update()
{
  return (LSM6DSV16XStatusTypeDef)block_data_update_set(PROPERTY_ENABLE);
}

/**
 * @brief Disable Block Data Update.
 * 
 * This function disables the Block Data Update feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_Block_Data_Update()
{
  return (LSM6DSV16XStatusTypeDef)block_data_update_set(PROPERTY_DISABLE);
}

/**
 * @brief Enable Auto Increment.
 * 
 * This function enables the Auto Increment feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Enable_Auto_Increment()
{
  return (LSM6DSV16XStatusTypeDef)auto_increment_set(PROPERTY_ENABLE);
}

/**
 * @brief Disable Auto Increment.
 * 
 * This function disables the Auto Increment feature.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Disable_Auto_Increment()
{
  return (LSM6DSV16XStatusTypeDef)auto_increment_set(PROPERTY_DISABLE);
}

/**
 * @brief Reset the LSM6DSV16X device.
 * 
 * This function resets the LSM6DSV16X device according to the specified reset flags and ensures
 * that the device is ready after the reset.
 * 
 * @param flags The reset flags specifying which components of the device to reset.
 * 
 * @return Status of the operation.
 *         - LSM6DSV16X_OK if the operation is successful.
 *         - LSM6DSV16X_ERROR if the operation fails.
 */
LSM6DSV16XStatusTypeDef LSM6DSV16X::Device_Reset(LSM6DSV16X_Reset_t flags)
{
  if (reset_set((lsm6dsv16x_reset_t)flags) != LSM6DSV16X_OK) {
    return LSM6DSV16X_ERROR;
  }

  lsm6dsv16x_reset_t rst;
  do {
    if (reset_get(&rst) != LSM6DSV16X_OK) {
      return LSM6DSV16X_ERROR;
    }
  } while (rst != LSM6DSV16X_READY);
  
  return LSM6DSV16X_OK;
}

/*********************/
/* PRIVATE FUNCTIONS */
/*********************/
LSM6DSV16XStatusTypeDef LSM6DSV16X::npy_halfbits_to_floatbits(uint16_t h, uint32_t *f)
{
  uint16_t h_exp, h_sig;
  uint32_t f_sgn, f_exp, f_sig;

  h_exp = (h & 0x7c00u);
  f_sgn = ((uint32_t)h & 0x8000u) << 16;
  switch (h_exp) {
    case 0x0000u: /* 0 or subnormal */
      h_sig = (h & 0x03ffu);
      /* Signed zero */
      if (h_sig == 0) {
        *f = f_sgn;
        return LSM6DSV16X_OK;
      }
      /* Subnormal */
      h_sig <<= 1;
      while ((h_sig & 0x0400u) == 0) {
        h_sig <<= 1;
        h_exp++;
      }
      f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
      f_sig = ((uint32_t)(h_sig & 0x03ffu)) << 13;
      *f = f_sgn + f_exp + f_sig;
      return LSM6DSV16X_OK;
    case 0x7c00u: /* inf or NaN */
      /* All-ones exponent and a copy of the significand */
      *f = f_sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
      return LSM6DSV16X_OK;
    default: /* normalized */
      /* Just need to adjust the exponent and shift */
      *f = f_sgn + (((uint32_t)(h & 0x7fffu) + 0x1c000u) << 13);
      return LSM6DSV16X_OK;
  }
}

LSM6DSV16XStatusTypeDef LSM6DSV16X::npy_half_to_float(uint16_t h, float *f)
{
  union {
    float ret;
    uint32_t retbits;
  } conv;
  npy_halfbits_to_floatbits(h, &conv.retbits);
  *f = conv.ret;
  return LSM6DSV16X_OK;
}

LSM6DSV16XStatusTypeDef LSM6DSV16X::sflp2q(float quat[4], uint16_t sflp[3])
{
  float sumsq = 0;

  npy_half_to_float(sflp[0], &quat[0]);
  npy_half_to_float(sflp[1], &quat[1]);
  npy_half_to_float(sflp[2], &quat[2]);

  for (uint8_t i = 0; i < 3; i++) {
    sumsq += quat[i] * quat[i];
  }

  if (sumsq > 1.0f) {
    float n = sqrtf(sumsq);
    quat[0] /= n;
    quat[1] /= n;
    quat[2] /= n;
    sumsq = 1.0f;
  }

  quat[3] = sqrtf(1.0f - sumsq);
  return LSM6DSV16X_OK;
}


bool LSM6DSV16X::readRegister(uint8_t reg, uint8_t *value, uint16_t len) {
    if (i2c->write(lsm6ds01tis_8bit_address, (const char*)&reg, 1) != 0)
        return 1;
    if (i2c->read(lsm6ds01tis_8bit_address, (char*) value, len) != 0)
        return 1;

    bool ret = 0;
    return ret;
}

bool LSM6DSV16X::writeRegister(uint8_t reg, const uint8_t *value, uint16_t len) {
    uint8_t data[len + 1];
    data[0] = reg; // inserisci il byte di registro nella prima posizione

    // copia i dati da value a data a partire dalla seconda posizione
    for (uint16_t i = 0; i < len; ++i) {
        data[i + 1] = value[i];
    }

    // scrivi il registro con i dati al bus I2C
    if (i2c->write(lsm6ds01tis_8bit_address, (const char*)data, len + 1) != 0)
        return 1;
    return 0;
}



int32_t LSM6DSV16X::auto_increment_set(uint8_t val)
{
  lsm6dsv16x_ctrl3_t ctrl3;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL3, (uint8_t *)&ctrl3, 1);
  if (ret == 0) {
    ctrl3.if_inc = val;
    ret = writeRegister(LSM6DSV16X_CTRL3, (uint8_t *)&ctrl3, 1);
  }

  return ret;
}


int32_t LSM6DSV16X::block_data_update_set(uint8_t val)
{
  lsm6dsv16x_ctrl3_t ctrl3;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL3, (uint8_t *)&ctrl3, 1);

  if (ret == 0) {
    ctrl3.bdu = val;
    ret = writeRegister(LSM6DSV16X_CTRL3, (uint8_t *)&ctrl3, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::fifo_mode_set(lsm6dsv16x_fifo_mode_t val)
{
  lsm6dsv16x_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  if (ret == 0) {
    fifo_ctrl4.fifo_mode = (uint8_t)val & 0x07U;
    ret = writeRegister(LSM6DSV16X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::xl_data_rate_set(lsm6dsv16x_data_rate_t val)
{
  lsm6dsv16x_ctrl1_t ctrl1;
  lsm6dsv16x_haodr_cfg_t haodr;
  uint8_t sel;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL1, (uint8_t *)&ctrl1, 1);
  if (ret != 0) {
    return ret;
  }

  ctrl1.odr_xl = (uint8_t)val & 0x0Fu;
  ret = writeRegister(LSM6DSV16X_CTRL1, (uint8_t *)&ctrl1, 1);
  if (ret != 0) {
    return ret;
  }

  sel = ((uint8_t)val >> 4) & 0xFU;
  if (sel != 0U) {
    ret += readRegister(LSM6DSV16X_HAODR_CFG, (uint8_t *)&haodr, 1);
    haodr.haodr_sel = sel;
    ret += writeRegister(LSM6DSV16X_HAODR_CFG, (uint8_t *)&haodr, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::xl_full_scale_set(lsm6dsv16x_xl_full_scale_t val)
{
  lsm6dsv16x_ctrl8_t ctrl8;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL8, (uint8_t *)&ctrl8, 1);

  if (ret == 0) {
    ctrl8.fs_xl = (uint8_t)val & 0x3U;
    ret = writeRegister(LSM6DSV16X_CTRL8, (uint8_t *)&ctrl8, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::gy_data_rate_set(lsm6dsv16x_data_rate_t val)
{
  lsm6dsv16x_ctrl2_t ctrl2;
  lsm6dsv16x_haodr_cfg_t haodr;
  uint8_t sel;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL2, (uint8_t *)&ctrl2, 1);
  ctrl2.odr_g = (uint8_t)val & 0x0Fu;
  ret += writeRegister(LSM6DSV16X_CTRL2, (uint8_t *)&ctrl2, 1);
  if (ret != 0) {
    return ret;
  }

  sel = ((uint8_t)val >> 4) & 0xFU;
  if (sel != 0U) {
    ret += readRegister(LSM6DSV16X_HAODR_CFG, (uint8_t *)&haodr, 1);
    haodr.haodr_sel = sel;
    ret += writeRegister(LSM6DSV16X_HAODR_CFG, (uint8_t *)&haodr, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::gy_full_scale_set(lsm6dsv16x_gy_full_scale_t val)
{
  lsm6dsv16x_ctrl6_t ctrl6;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL6, (uint8_t *)&ctrl6, 1);

  if (ret == 0) {
    ctrl6.fs_g = (uint8_t)val & 0xfu;
    ret = writeRegister(LSM6DSV16X_CTRL6, (uint8_t *)&ctrl6, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::xl_data_rate_get(lsm6dsv16x_data_rate_t *val)
{
  lsm6dsv16x_ctrl1_t ctrl1;
  lsm6dsv16x_haodr_cfg_t haodr;
  uint8_t sel;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += readRegister(LSM6DSV16X_HAODR_CFG, (uint8_t *)&haodr, 1);
  if (ret != 0) {
    return ret;
  }

  sel = haodr.haodr_sel;

  switch (ctrl1.odr_xl) {
    case LSM6DSV16X_ODR_OFF:
      *val = LSM6DSV16X_ODR_OFF;
      break;

    case LSM6DSV16X_ODR_AT_1Hz875:
      *val = LSM6DSV16X_ODR_AT_1Hz875;
      break;

    case LSM6DSV16X_ODR_AT_7Hz5:
      *val = LSM6DSV16X_ODR_AT_7Hz5;
      break;

    case LSM6DSV16X_ODR_AT_15Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_15Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_15Hz625;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_12Hz5;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_30Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_30Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_31Hz25;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_25Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_60Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_60Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_62Hz5;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_50Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_120Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_120Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_125Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_100Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_240Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_240Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_250Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_200Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_480Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_480Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_500Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_400Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_960Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_960Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_1000Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_800Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_1920Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_1920Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_2000Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_1600Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_3840Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_3840Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_4000Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_3200Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_7680Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_7680Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_8000Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_6400Hz;
          break;
      }
      break;

    default:
      *val = LSM6DSV16X_ODR_OFF;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::gy_data_rate_get(lsm6dsv16x_data_rate_t *val)
{
  lsm6dsv16x_ctrl2_t ctrl2;
  lsm6dsv16x_haodr_cfg_t haodr;
  uint8_t sel;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL2, (uint8_t *)&ctrl2, 1);
  ret += readRegister(LSM6DSV16X_HAODR_CFG, (uint8_t *)&haodr, 1);
  if (ret != 0) {
    return ret;
  }

  sel = haodr.haodr_sel;

  switch (ctrl2.odr_g) {
    case LSM6DSV16X_ODR_OFF:
      *val = LSM6DSV16X_ODR_OFF;
      break;

    case LSM6DSV16X_ODR_AT_1Hz875:
      *val = LSM6DSV16X_ODR_AT_1Hz875;
      break;

    case LSM6DSV16X_ODR_AT_7Hz5:
      *val = LSM6DSV16X_ODR_AT_7Hz5;
      break;

    case LSM6DSV16X_ODR_AT_15Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_15Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_15Hz625;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_12Hz5;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_30Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_30Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_31Hz25;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_25Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_60Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_60Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_62Hz5;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_50Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_120Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_120Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_125Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_100Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_240Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_240Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_250Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_200Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_480Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_480Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_500Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_400Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_960Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_960Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_1000Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_800Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_1920Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_1920Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_2000Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_1600Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_3840Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_3840Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_4000Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_3200Hz;
          break;
      }
      break;

    case LSM6DSV16X_ODR_AT_7680Hz:
      switch (sel) {
        default:
        case 0:
          *val = LSM6DSV16X_ODR_AT_7680Hz;
          break;
        case 1:
          *val = LSM6DSV16X_ODR_HA01_AT_8000Hz;
          break;
        case 2:
          *val = LSM6DSV16X_ODR_HA02_AT_6400Hz;
          break;
      }
      break;

    default:
      *val = LSM6DSV16X_ODR_OFF;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::gy_mode_set(lsm6dsv16x_gy_mode_t val)
{
  lsm6dsv16x_ctrl2_t ctrl2;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL2, (uint8_t *)&ctrl2, 1);
  if (ret == 0) {
    ctrl2.op_mode_g = (uint8_t)val & 0x07U;
    ret = writeRegister(LSM6DSV16X_CTRL2, (uint8_t *)&ctrl2, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::gy_mode_get(lsm6dsv16x_gy_mode_t *val)
{
  lsm6dsv16x_ctrl2_t ctrl2;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL2, (uint8_t *)&ctrl2, 1);
  if (ret != 0) {
    return ret;
  }

  switch (ctrl2.op_mode_g) {
    case LSM6DSV16X_GY_HIGH_PERFORMANCE_MD:
      *val = LSM6DSV16X_GY_HIGH_PERFORMANCE_MD;
      break;

    case LSM6DSV16X_GY_HIGH_ACCURACY_ODR_MD:
      *val = LSM6DSV16X_GY_HIGH_ACCURACY_ODR_MD;
      break;

    case LSM6DSV16X_GY_SLEEP_MD:
      *val = LSM6DSV16X_GY_SLEEP_MD;
      break;

    case LSM6DSV16X_GY_LOW_POWER_MD:
      *val = LSM6DSV16X_GY_LOW_POWER_MD;
      break;

    default:
      *val = LSM6DSV16X_GY_HIGH_PERFORMANCE_MD;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::angular_rate_raw_get(int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = readRegister(LSM6DSV16X_OUTX_L_G, &buff[0], 6);
  if (ret != 0) {
    return ret;
  }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

int32_t LSM6DSV16X::all_sources_get(lsm6dsv16x_all_sources_t *val)
{
  lsm6dsv16x_emb_func_status_mainpage_t emb_func_status_mainpage;
  lsm6dsv16x_emb_func_exec_status_t emb_func_exec_status;
  lsm6dsv16x_fsm_status_mainpage_t fsm_status_mainpage;
  lsm6dsv16x_mlc_status_mainpage_t mlc_status_mainpage;
  lsm6dsv16x_functions_enable_t functions_enable;
  lsm6dsv16x_emb_func_src_t emb_func_src;
  lsm6dsv16x_fifo_status2_t fifo_status2;
  lsm6dsv16x_all_int_src_t all_int_src;
  lsm6dsv16x_wake_up_src_t wake_up_src;
  lsm6dsv16x_status_reg_t status_reg;
  lsm6dsv16x_d6d_src_t d6d_src;
  lsm6dsv16x_tap_src_t tap_src;
  lsm6dsv16x_ui_status_reg_ois_t status_reg_ois;
  lsm6dsv16x_status_master_t status_shub;
  uint8_t buff[8];
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  functions_enable.dis_rst_lir_all_int = PROPERTY_ENABLE;
  ret += writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret != 0) {
    return ret;
  }

  ret = readRegister(LSM6DSV16X_FIFO_STATUS1, (uint8_t *)&buff, 4);
  if (ret != 0) {
    return ret;
  }

  bytecpy((uint8_t *)&fifo_status2, &buff[1]);
  bytecpy((uint8_t *)&all_int_src, &buff[2]);
  bytecpy((uint8_t *)&status_reg, &buff[3]);

  val->fifo_ovr = fifo_status2.fifo_ovr_ia;
  val->fifo_bdr = fifo_status2.counter_bdr_ia;
  val->fifo_full = fifo_status2.fifo_full_ia;
  val->fifo_th = fifo_status2.fifo_wtm_ia;

  val->free_fall = all_int_src.ff_ia;
  val->wake_up = all_int_src.wu_ia;
  val->six_d = all_int_src.d6d_ia;

  val->drdy_xl = status_reg.xlda;
  val->drdy_gy = status_reg.gda;
  val->drdy_temp = status_reg.tda;
  val->drdy_ah_qvar = status_reg.ah_qvarda;
  val->drdy_eis = status_reg.gda_eis;
  val->drdy_ois = status_reg.ois_drdy;
  val->timestamp = status_reg.timestamp_endcount;

  ret = readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  functions_enable.dis_rst_lir_all_int = PROPERTY_DISABLE;
  ret += writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret != 0) {
    return ret;
  }

  ret = readRegister(LSM6DSV16X_UI_STATUS_REG_OIS, (uint8_t *)&buff, 8);
  if (ret != 0) {
    return ret;
  }

  bytecpy((uint8_t *)&status_reg_ois, &buff[0]);
  bytecpy((uint8_t *)&wake_up_src, &buff[1]);
  bytecpy((uint8_t *)&tap_src, &buff[2]);
  bytecpy((uint8_t *)&d6d_src, &buff[3]);
  bytecpy((uint8_t *)&emb_func_status_mainpage, &buff[5]);
  bytecpy((uint8_t *)&fsm_status_mainpage, &buff[6]);
  bytecpy((uint8_t *)&mlc_status_mainpage, &buff[7]);

  val->gy_settling = status_reg_ois.gyro_settling;
  val->sleep_change = wake_up_src.sleep_change_ia;
  val->wake_up_x = wake_up_src.x_wu;
  val->wake_up_y = wake_up_src.y_wu;
  val->wake_up_z = wake_up_src.z_wu;
  val->sleep_state = wake_up_src.sleep_state;

  val->tap_x = tap_src.x_tap;
  val->tap_y = tap_src.y_tap;
  val->tap_z = tap_src.z_tap;
  val->tap_sign = tap_src.tap_sign;
  val->double_tap = tap_src.double_tap;
  val->single_tap = tap_src.single_tap;

  val->six_d_zl = d6d_src.zl;
  val->six_d_zh = d6d_src.zh;
  val->six_d_yl = d6d_src.yl;
  val->six_d_yh = d6d_src.yh;
  val->six_d_xl = d6d_src.xl;
  val->six_d_xh = d6d_src.xh;

  val->step_detector = emb_func_status_mainpage.is_step_det;
  val->tilt = emb_func_status_mainpage.is_tilt;
  val->sig_mot = emb_func_status_mainpage.is_sigmot;
  val->fsm_lc = emb_func_status_mainpage.is_fsm_lc;

  val->fsm1 = fsm_status_mainpage.is_fsm1;
  val->fsm2 = fsm_status_mainpage.is_fsm2;
  val->fsm3 = fsm_status_mainpage.is_fsm3;
  val->fsm4 = fsm_status_mainpage.is_fsm4;
  val->fsm5 = fsm_status_mainpage.is_fsm5;
  val->fsm6 = fsm_status_mainpage.is_fsm6;
  val->fsm7 = fsm_status_mainpage.is_fsm7;
  val->fsm8 = fsm_status_mainpage.is_fsm8;

  val->mlc1 = mlc_status_mainpage.is_mlc1;
  val->mlc2 = mlc_status_mainpage.is_mlc2;
  val->mlc3 = mlc_status_mainpage.is_mlc3;
  val->mlc4 = mlc_status_mainpage.is_mlc4;


  /* embedded func */
  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  ret += readRegister(LSM6DSV16X_EMB_FUNC_EXEC_STATUS, (uint8_t *)&emb_func_exec_status, 1);
  ret += readRegister(LSM6DSV16X_EMB_FUNC_SRC, (uint8_t *)&emb_func_src, 1);
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  val->emb_func_stand_by = emb_func_exec_status.emb_func_endop;
  val->emb_func_time_exceed = emb_func_exec_status.emb_func_exec_ovr;
  val->step_count_inc = emb_func_src.stepcounter_bit_set;
  val->step_count_overflow = emb_func_src.step_overflow;
  val->step_on_delta_time = emb_func_src.step_count_delta_ia;

  val->step_detector = emb_func_src.step_detected;

  /* sensor hub */
  ret = readRegister(LSM6DSV16X_STATUS_MASTER_MAINPAGE, (uint8_t *)&status_shub, 1);
  if (ret != 0) {
    return ret;
  }

  val->sh_endop = status_shub.sens_hub_endop;
  val->sh_wr_once = status_shub.wr_once_done;
  val->sh_slave3_nack = status_shub.slave3_nack;
  val->sh_slave2_nack = status_shub.slave2_nack;
  val->sh_slave1_nack = status_shub.slave1_nack;
  val->sh_slave0_nack = status_shub.slave0_nack;

  return ret;
}

int32_t LSM6DSV16X::mem_bank_set(lsm6dsv16x_mem_bank_t val)
{
  lsm6dsv16x_func_cfg_access_t func_cfg_access;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0) {
    return ret;
  }

  func_cfg_access.shub_reg_access = ((uint8_t)val & 0x02U) >> 1;
  func_cfg_access.emb_func_reg_access = (uint8_t)val & 0x01U;
  ret = writeRegister(LSM6DSV16X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);

  return ret;
}


int32_t LSM6DSV16X::mem_bank_get(lsm6dsv16x_mem_bank_t *val)
{
  lsm6dsv16x_func_cfg_access_t func_cfg_access;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0) {
    return ret;
  }

  switch ((func_cfg_access.shub_reg_access << 1) + func_cfg_access.emb_func_reg_access) {
    case LSM6DSV16X_MAIN_MEM_BANK:
      *val = LSM6DSV16X_MAIN_MEM_BANK;
      break;

    case LSM6DSV16X_EMBED_FUNC_MEM_BANK:
      *val = LSM6DSV16X_EMBED_FUNC_MEM_BANK;
      break;

    case LSM6DSV16X_SENSOR_HUB_MEM_BANK:
      *val = LSM6DSV16X_SENSOR_HUB_MEM_BANK;
      break;

    default:
      *val = LSM6DSV16X_MAIN_MEM_BANK;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::gy_lp1_set(uint8_t val)
{
  lsm6dsv16x_ctrl7_t ctrl7;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret == 0) {
    ctrl7.lpf1_g_en = val;
    ret = writeRegister(LSM6DSV16X_CTRL7, (uint8_t *)&ctrl7, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::filt_gy_lp1_bandwidth_set(lsm6dsv16x_filt_gy_lp1_bandwidth_t val)
{
  lsm6dsv16x_ctrl6_t ctrl6;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL6, (uint8_t *)&ctrl6, 1);
  if (ret == 0) {
    ctrl6.lpf1_g_bw = (uint8_t)val & 0x0Fu;
    ret = writeRegister(LSM6DSV16X_CTRL6, (uint8_t *)&ctrl6, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::temperature_raw_get(int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = readRegister(LSM6DSV16X_OUT_TEMP_L, &buff[0], 2);
  if (ret != 0) {
    return ret;
  }

  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}




int32_t LSM6DSV16X::flag_data_ready_get(lsm6dsv16x_data_ready_t *val)
{
  lsm6dsv16x_status_reg_t status;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_STATUS_REG, (uint8_t *)&status, 1);
  if (ret != 0) {
    return ret;
  }

  val->drdy_xl = status.xlda;
  val->drdy_gy = status.gda;
  val->drdy_temp = status.tda;

  return ret;
}

int32_t LSM6DSV16X::gy_self_test_set(lsm6dsv16x_gy_self_test_t val)
{
  lsm6dsv16x_ctrl10_t ctrl10;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL10, (uint8_t *)&ctrl10, 1);

  if (ret == 0) {
    ctrl10.st_g = (uint8_t)val & 0x3U;
    ret = writeRegister(LSM6DSV16X_CTRL10, (uint8_t *)&ctrl10, 1);
  }

  return ret;
}


int32_t LSM6DSV16X::gy_self_test_get(lsm6dsv16x_gy_self_test_t *val)
{
  lsm6dsv16x_ctrl10_t ctrl10;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL10, (uint8_t *)&ctrl10, 1);
  if (ret != 0) {
    return ret;
  }

  switch (ctrl10.st_g) {
    case LSM6DSV16X_GY_ST_DISABLE:
      *val = LSM6DSV16X_GY_ST_DISABLE;
      break;

    case LSM6DSV16X_GY_ST_POSITIVE:
      *val = LSM6DSV16X_GY_ST_POSITIVE;
      break;

    case LSM6DSV16X_GY_ST_NEGATIVE:
      *val = LSM6DSV16X_GY_ST_NEGATIVE;
      break;

    default:
      *val = LSM6DSV16X_GY_ST_DISABLE;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::ah_qvar_raw_get(int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = readRegister(LSM6DSV16X_AH_QVAR_OUT_L, &buff[0], 2);
  if (ret != 0) {
    return ret;
  }

  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}

int32_t LSM6DSV16X::ah_qvar_zin_get(lsm6dsv16x_ah_qvar_zin_t *val)
{
  lsm6dsv16x_ctrl7_t ctrl7;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret != 0) {
    return ret;
  }

  switch (ctrl7.ah_qvar_c_zin) {
    case LSM6DSV16X_2400MOhm:
      *val = LSM6DSV16X_2400MOhm;
      break;

    case LSM6DSV16X_730MOhm:
      *val = LSM6DSV16X_730MOhm;
      break;

    case LSM6DSV16X_300MOhm:
      *val = LSM6DSV16X_300MOhm;
      break;

    case LSM6DSV16X_255MOhm:
      *val = LSM6DSV16X_255MOhm;
      break;

    default:
      *val = LSM6DSV16X_2400MOhm;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::ah_qvar_zin_set(lsm6dsv16x_ah_qvar_zin_t val)
{
  lsm6dsv16x_ctrl7_t ctrl7;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret == 0) {
    ctrl7.ah_qvar_c_zin = (uint8_t)val & 0x03U;
    ret = writeRegister(LSM6DSV16X_CTRL7, (uint8_t *)&ctrl7, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::mlc_out_get(lsm6dsv16x_mlc_out_t *val)
{
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret == 0) {
    ret = readRegister(LSM6DSV16X_MLC1_SRC, (uint8_t *)val, 4);
  }
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}

int32_t LSM6DSV16X::fifo_sflp_batch_set(lsm6dsv16x_fifo_sflp_raw_t val)
{
  lsm6dsv16x_emb_func_fifo_en_a_t emb_func_fifo_en_a;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret == 0) {
    ret = readRegister(LSM6DSV16X_EMB_FUNC_FIFO_EN_A, (uint8_t *)&emb_func_fifo_en_a, 1);
    emb_func_fifo_en_a.sflp_game_fifo_en = val.game_rotation;
    emb_func_fifo_en_a.sflp_gravity_fifo_en = val.gravity;
    emb_func_fifo_en_a.sflp_gbias_fifo_en = val.gbias;
    ret += writeRegister(LSM6DSV16X_EMB_FUNC_FIFO_EN_A,
                                (uint8_t *)&emb_func_fifo_en_a, 1);
  }

  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}


int32_t LSM6DSV16X::fifo_sflp_batch_get(lsm6dsv16x_fifo_sflp_raw_t *val)
{
  lsm6dsv16x_emb_func_fifo_en_a_t emb_func_fifo_en_a;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret == 0) {
    ret = readRegister(LSM6DSV16X_EMB_FUNC_FIFO_EN_A, (uint8_t *)&emb_func_fifo_en_a, 1);

    val->game_rotation = emb_func_fifo_en_a.sflp_game_fifo_en;
    val->gravity = emb_func_fifo_en_a.sflp_gravity_fifo_en;
    val->gbias = emb_func_fifo_en_a.sflp_gbias_fifo_en;
  }

  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}

int32_t LSM6DSV16X::sflp_game_rotation_set(uint8_t val)
{
  lsm6dsv16x_emb_func_en_a_t emb_func_en_a;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  ret = readRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  if (ret != 0) {
    goto exit;
  }

  emb_func_en_a.sflp_game_en = val;
  ret += writeRegister(LSM6DSV16X_EMB_FUNC_EN_A,
                              (uint8_t *)&emb_func_en_a, 1);

exit:
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}


int32_t LSM6DSV16X::sflp_game_rotation_get(uint8_t *val)
{
  lsm6dsv16x_emb_func_en_a_t emb_func_en_a;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  ret = readRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  *val = emb_func_en_a.sflp_game_en;

  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}

int32_t LSM6DSV16X::sflp_data_rate_set(lsm6dsv16x_sflp_data_rate_t val)
{
  lsm6dsv16x_sflp_odr_t sflp_odr;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  ret = readRegister(LSM6DSV16X_SFLP_ODR, (uint8_t *)&sflp_odr, 1);
  if (ret != 0) {
    goto exit;
  }

  sflp_odr.sflp_game_odr = (uint8_t)val & 0x07U;
  ret += writeRegister(LSM6DSV16X_SFLP_ODR, (uint8_t *)&sflp_odr, 1);

exit:
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}

int32_t LSM6DSV16X::sflp_game_gbias_set(lsm6dsv16x_sflp_gbias_t *val)
{
  lsm6dsv16x_sflp_data_rate_t sflp_odr;
  lsm6dsv16x_emb_func_exec_status_t emb_func_sts;
  lsm6dsv16x_data_ready_t drdy;
  lsm6dsv16x_xl_full_scale_t xl_fs;
  lsm6dsv16x_ctrl10_t ctrl10;
  uint8_t master_config;
  uint8_t emb_func_en_saved[2];
  uint8_t conf_saved[2];
  uint8_t reg_zero[2] = {0x0, 0x0};
  uint16_t gbias_hf[3];
  float_t k = 0.005f;
  int16_t xl_data[3];
  int32_t data_tmp;
  uint8_t *data_ptr = (uint8_t *)&data_tmp;
  uint8_t i, j;
  int32_t ret;

  ret = sflp_data_rate_get(&sflp_odr);
  if (ret != 0) {
    return ret;
  }

  /* Calculate k factor */
  switch (sflp_odr) {
    default:
    case LSM6DSV16X_SFLP_15Hz:
      k = 0.04f;
      break;
    case LSM6DSV16X_SFLP_30Hz:
      k = 0.02f;
      break;
    case LSM6DSV16X_SFLP_60Hz:
      k = 0.01f;
      break;
    case LSM6DSV16X_SFLP_120Hz:
      k = 0.005f;
      break;
    case LSM6DSV16X_SFLP_240Hz:
      k = 0.0025f;
      break;
    case LSM6DSV16X_SFLP_480Hz:
      k = 0.00125f;
      break;
  }

  /* compute gbias as half precision float in order to be put in embedded advanced feature register */
  gbias_hf[0] = npy_float_to_half(val->gbias_x * (3.14159265358979323846f / 180.0f) / k);
  gbias_hf[1] = npy_float_to_half(val->gbias_y * (3.14159265358979323846f / 180.0f) / k);
  gbias_hf[2] = npy_float_to_half(val->gbias_z * (3.14159265358979323846f / 180.0f) / k);

  /* Save sensor configuration and set high-performance mode (if the sensor is in power-down mode, turn it on) */
  ret += readRegister(LSM6DSV16X_CTRL1, conf_saved, 2);
  ret += xl_mode_set(LSM6DSV16X_XL_HIGH_PERFORMANCE_MD);
  ret += gy_mode_set(LSM6DSV16X_GY_HIGH_PERFORMANCE_MD);
  if (((uint8_t)conf_saved[0] & 0x0FU) == (uint8_t)LSM6DSV16X_ODR_OFF) {
    ret += xl_data_rate_set(LSM6DSV16X_ODR_AT_120Hz);
  }

  /* Make sure to turn the sensor-hub master off */
  ret += sh_master_get(&master_config);
  ret += sh_master_set(0);

  /* disable algos */
  ret += mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  ret += readRegister(LSM6DSV16X_EMB_FUNC_EN_A, emb_func_en_saved, 2);
  ret += writeRegister(LSM6DSV16X_EMB_FUNC_EN_A, reg_zero, 2);
  do {
    ret += readRegister(LSM6DSV16X_EMB_FUNC_EXEC_STATUS,
                               (uint8_t *)&emb_func_sts, 1);
  } while (emb_func_sts.emb_func_endop != 1U);
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  // enable gbias setting
  ret += readRegister(LSM6DSV16X_CTRL10, (uint8_t *)&ctrl10, 1);
  ctrl10.emb_func_debug = 1;
  ret += writeRegister(LSM6DSV16X_CTRL10, (uint8_t *)&ctrl10, 1);

  /* enable algos */
  ret += mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  emb_func_en_saved[0] |= 0x02U; /* force SFLP GAME en */
  ret += writeRegister(LSM6DSV16X_EMB_FUNC_EN_A, emb_func_en_saved,
                              2);
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  ret += xl_full_scale_get(&xl_fs);

  /* Read XL data */
  do {
    ret += flag_data_ready_get(&drdy);
  } while (drdy.drdy_xl != 1U);
  ret += acceleration_raw_get(xl_data);

  /* force sflp initialization */
  ret += mem_bank_set(LSM6DSV16X_SENSOR_HUB_MEM_BANK);
  for (i = 0; i < 3U; i++) {
    j = 0;
    data_tmp = (int32_t)xl_data[i];
    data_tmp <<= xl_fs; // shift based on current fs
    ret += writeRegister(LSM6DSV16X_SENSOR_HUB_1 + 3U * i,
                                &data_ptr[j++], 1);
    ret += writeRegister(LSM6DSV16X_SENSOR_HUB_2 + 3U * i,
                                &data_ptr[j++], 1);
    ret += writeRegister(LSM6DSV16X_SENSOR_HUB_3 + 3U * i, &data_ptr[j],
                                1);
  }
  for (i = 0; i < 3U; i++) {
    j = 0;
    data_tmp = 0;
    ret += writeRegister(LSM6DSV16X_SENSOR_HUB_10 + 3U * i,
                                &data_ptr[j++], 1);
    ret += writeRegister(LSM6DSV16X_SENSOR_HUB_11 + 3U * i,
                                &data_ptr[j++], 1);
    ret += writeRegister(LSM6DSV16X_SENSOR_HUB_12 + 3U * i, &data_ptr[j],
                                1);
  }
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  // wait end_op (and at least 30 us)
  wait_us(40);

  ret += mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  do {
    ret += readRegister(LSM6DSV16X_EMB_FUNC_EXEC_STATUS,
                               (uint8_t *)&emb_func_sts, 1);
  } while (emb_func_sts.emb_func_endop != 1U);
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  /* write gbias in embedded advanced features registers */
  ret += ln_pg_write(LSM6DSV16X_SFLP_GAME_GBIASX_L,
                                (uint8_t *)gbias_hf, 6);

  /* reload previous sensor configuration */
  ret += writeRegister(LSM6DSV16X_CTRL1, conf_saved, 2);

  // disable gbias setting
  ctrl10.emb_func_debug = 0;
  ret += writeRegister(LSM6DSV16X_CTRL10, (uint8_t *)&ctrl10, 1);

  /* reload previous master configuration */
  ret += sh_master_set(master_config);

  return ret;
}

int32_t LSM6DSV16X::sflp_data_rate_get(lsm6dsv16x_sflp_data_rate_t *val)
{
  lsm6dsv16x_sflp_odr_t sflp_odr;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  ret += readRegister(LSM6DSV16X_SFLP_ODR, (uint8_t *)&sflp_odr, 1);
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  switch (sflp_odr.sflp_game_odr) {
    case LSM6DSV16X_SFLP_15Hz:
      *val = LSM6DSV16X_SFLP_15Hz;
      break;

    case LSM6DSV16X_SFLP_30Hz:
      *val = LSM6DSV16X_SFLP_30Hz;
      break;

    case LSM6DSV16X_SFLP_60Hz:
      *val = LSM6DSV16X_SFLP_60Hz;
      break;

    case LSM6DSV16X_SFLP_120Hz:
      *val = LSM6DSV16X_SFLP_120Hz;
      break;

    case LSM6DSV16X_SFLP_240Hz:
      *val = LSM6DSV16X_SFLP_240Hz;
      break;

    case LSM6DSV16X_SFLP_480Hz:
      *val = LSM6DSV16X_SFLP_480Hz;
      break;

    default:
      *val = LSM6DSV16X_SFLP_15Hz;
      break;
  }

  return ret;
}


uint16_t LSM6DSV16X::npy_float_to_half(float_t f)
{
  union {
    float_t f;
    uint32_t fbits;
  } conv;
  conv.f = f;
  return npy_floatbits_to_halfbits(conv.fbits);
}

uint16_t LSM6DSV16X::npy_floatbits_to_halfbits(uint32_t f)
{
  uint32_t f_exp, f_sig;
  uint16_t h_sgn, h_exp, h_sig;

  h_sgn = (uint16_t)((f & 0x80000000u) >> 16);
  f_exp = (f & 0x7f800000u);

  /* Exponent overflow/NaN converts to signed inf/NaN */
  if (f_exp >= 0x47800000u) {
    if (f_exp == 0x7f800000u) {
      /* Inf or NaN */
      f_sig = (f & 0x007fffffu);
      if (f_sig != 0U) {
        /* NaN - propagate the flag in the significand... */
        uint16_t ret = (uint16_t)(0x7c00u + (f_sig >> 13));
        /* ...but make sure it stays a NaN */
        if (ret == 0x7c00u) {
          ret++;
        }
        return h_sgn + ret;
      } else {
        /* signed inf */
        return (uint16_t)(h_sgn + 0x7c00u);
      }
    } else {
      /* overflow to signed inf */
#if NPY_HALF_GENERATE_OVERFLOW
      npy_set_floatstatus_overflow();
#endif
      return (uint16_t)(h_sgn + 0x7c00u);
    }
  }

  /* Exponent underflow converts to a subnormal half or signed zero */
  if (f_exp <= 0x38000000u) {
    /*
     * Signed zeros, subnormal floats, and floats with small
     * exponents all convert to signed zero half-floats.
     */
    if (f_exp < 0x33000000u) {
#if NPY_HALF_GENERATE_UNDERFLOW
      /* If f != 0, it underflowed to 0 */
      if ((f & 0x7fffffff) != 0) {
        npy_set_floatstatus_underflow();
      }
#endif
      return h_sgn;
    }
    /* Make the subnormal significand */
    f_exp >>= 23;
    f_sig = (0x00800000u + (f & 0x007fffffu));
#if NPY_HALF_GENERATE_UNDERFLOW
    /* If it's not exactly represented, it underflowed */
    if ((f_sig & (((uint32_t)1 << (126 - f_exp)) - 1)) != 0) {
      npy_set_floatstatus_underflow();
    }
#endif
    /*
     * Usually the significand is shifted by 13. For subnormals an
     * additional shift needs to occur. This shift is one for the largest
     * exponent giving a subnormal `f_exp = 0x38000000 >> 23 = 112`, which
     * offsets the new first bit. At most the shift can be 1+10 bits.
     */
    f_sig >>= (113U - f_exp);
    /* Handle rounding by adding 1 to the bit beyond half precision */
#if NPY_HALF_ROUND_TIES_TO_EVEN
    /*
     * If the last bit in the half significand is 0 (already even), and
     * the remaining bit pattern is 1000...0, then we do not add one
     * to the bit after the half significand. However, the (113 - f_exp)
     * shift can lose up to 11 bits, so the || checks them in the original.
     * In all other cases, we can just add one.
     */
    if (((f_sig & 0x00003fffu) != 0x00001000u) || (f & 0x000007ffu)) {
      f_sig += 0x00001000u;
    }
#else
    f_sig += 0x00001000u;
#endif
    h_sig = (uint16_t)(f_sig >> 13);
    /*
     * If the rounding causes a bit to spill into h_exp, it will
     * increment h_exp from zero to one and h_sig will be zero.
     * This is the correct result.
     */
    return (uint16_t)(h_sgn + h_sig);
  }

  /* Regular case with no overflow or underflow */
  h_exp = (uint16_t)((f_exp - 0x38000000u) >> 13);
  /* Handle rounding by adding 1 to the bit beyond half precision */
  f_sig = (f & 0x007fffffu);
#if NPY_HALF_ROUND_TIES_TO_EVEN
  /*
   * If the last bit in the half significand is 0 (already even), and
   * the remaining bit pattern is 1000...0, then we do not add one
   * to the bit after the half significand.  In all other cases, we do.
   */
  if ((f_sig & 0x00003fffu) != 0x00001000u) {
    f_sig += 0x00001000u;
  }
#else
  f_sig += 0x00001000u;
#endif
  h_sig = (uint16_t)(f_sig >> 13);
  /*
   * If the rounding causes a bit to spill into h_exp, it will
   * increment h_exp by one and h_sig will be zero.  This is the
   * correct result.  h_exp may increment to 15, at greatest, in
   * which case the result overflows to a signed inf.
   */
#if NPY_HALF_GENERATE_OVERFLOW
  h_sig += h_exp;
  if (h_sig == 0x7c00u) {
    npy_set_floatstatus_overflow();
  }
  return h_sgn + h_sig;
#else
  return h_sgn + h_exp + h_sig;
#endif
}

int32_t LSM6DSV16X::xl_mode_set(lsm6dsv16x_xl_mode_t val)
{
  lsm6dsv16x_ctrl1_t ctrl1;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL1, (uint8_t *)&ctrl1, 1);

  if (ret == 0) {
    ctrl1.op_mode_xl = (uint8_t)val & 0x07U;
    ret = writeRegister(LSM6DSV16X_CTRL1, (uint8_t *)&ctrl1, 1);
  }

  return ret;
}


int32_t LSM6DSV16X::xl_mode_get(lsm6dsv16x_xl_mode_t *val)
{
  lsm6dsv16x_ctrl1_t ctrl1;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL1, (uint8_t *)&ctrl1, 1);
  if (ret != 0) {
    return ret;
  }

  switch (ctrl1.op_mode_xl) {
    case LSM6DSV16X_XL_HIGH_PERFORMANCE_MD:
      *val = LSM6DSV16X_XL_HIGH_PERFORMANCE_MD;
      break;

    case LSM6DSV16X_XL_HIGH_ACCURACY_ODR_MD:
      *val = LSM6DSV16X_XL_HIGH_ACCURACY_ODR_MD;
      break;

    case LSM6DSV16X_XL_ODR_TRIGGERED_MD:
      *val = LSM6DSV16X_XL_ODR_TRIGGERED_MD;
      break;

    case LSM6DSV16X_XL_LOW_POWER_2_AVG_MD:
      *val = LSM6DSV16X_XL_LOW_POWER_2_AVG_MD;
      break;

    case LSM6DSV16X_XL_LOW_POWER_4_AVG_MD:
      *val = LSM6DSV16X_XL_LOW_POWER_4_AVG_MD;
      break;

    case LSM6DSV16X_XL_LOW_POWER_8_AVG_MD:
      *val = LSM6DSV16X_XL_LOW_POWER_8_AVG_MD;
      break;

    case LSM6DSV16X_XL_NORMAL_MD:
      *val = LSM6DSV16X_XL_NORMAL_MD;
      break;

    default:
      *val = LSM6DSV16X_XL_HIGH_PERFORMANCE_MD;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::sh_master_set(uint8_t val)
{
  lsm6dsv16x_master_config_t master_config;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_SENSOR_HUB_MEM_BANK);
  ret += readRegister(LSM6DSV16X_MASTER_CONFIG, (uint8_t *)&master_config, 1);
  if (ret != 0) {
    goto exit;
  }

  master_config.master_on = val;
  ret = writeRegister(LSM6DSV16X_MASTER_CONFIG, (uint8_t *)&master_config, 1);

exit:
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}


int32_t LSM6DSV16X::sh_master_get(uint8_t *val)
{
  lsm6dsv16x_master_config_t master_config;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_SENSOR_HUB_MEM_BANK);
  ret += readRegister(LSM6DSV16X_MASTER_CONFIG, (uint8_t *)&master_config, 1);

  *val = master_config.master_on;

  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}

int32_t LSM6DSV16X::xl_full_scale_get(lsm6dsv16x_xl_full_scale_t *val)
{
  lsm6dsv16x_ctrl8_t ctrl8;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL8, (uint8_t *)&ctrl8, 1);
  if (ret != 0) {
    return ret;
  }

  switch (ctrl8.fs_xl) {
    case LSM6DSV16X_2g:
      *val = LSM6DSV16X_2g;
      break;

    case LSM6DSV16X_4g:
      *val = LSM6DSV16X_4g;
      break;

    case LSM6DSV16X_8g:
      *val = LSM6DSV16X_8g;
      break;

    case LSM6DSV16X_16g:
      *val = LSM6DSV16X_16g;
      break;

    default:
      *val = LSM6DSV16X_2g;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::acceleration_raw_get(int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = readRegister(LSM6DSV16X_OUTX_L_A, &buff[0], 6);
  if (ret != 0) {
    return ret;
  }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}


int32_t LSM6DSV16X::dual_acceleration_raw_get(int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = readRegister(LSM6DSV16X_UI_OUTX_L_A_OIS_DUALC, &buff[0], 6);
  if (ret != 0) {
    return ret;
  }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

int32_t LSM6DSV16X::ln_pg_write(uint16_t address, uint8_t *buf, uint8_t len)
{
  lsm6dsv16x_page_address_t  page_address;
  lsm6dsv16x_page_sel_t page_sel;
  lsm6dsv16x_page_rw_t page_rw;
  uint8_t msb;
  uint8_t lsb;
  int32_t ret;
  uint8_t i ;

  msb = ((uint8_t)(address >> 8) & 0x0FU);
  lsb = (uint8_t)address & 0xFFU;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  /* set page write */
  ret += readRegister(LSM6DSV16X_PAGE_RW, (uint8_t *)&page_rw, 1);
  page_rw.page_read = PROPERTY_DISABLE;
  page_rw.page_write = PROPERTY_ENABLE;
  ret += writeRegister(LSM6DSV16X_PAGE_RW, (uint8_t *)&page_rw, 1);
  if (ret != 0) {
    goto exit;
  }

  /* select page */
  ret += readRegister(LSM6DSV16X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  page_sel.page_sel = msb;
  page_sel.not_used0 = 1; // Default value
  ret += writeRegister(LSM6DSV16X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0) {
    goto exit;
  }

  /* set page addr */
  page_address.page_addr = lsb;
  ret += writeRegister(LSM6DSV16X_PAGE_ADDRESS,
                              (uint8_t *)&page_address, 1);
  if (ret != 0) {
    goto exit;
  }

  for (i = 0; ((i < len) && (ret == 0)); i++) {
    ret += writeRegister(LSM6DSV16X_PAGE_VALUE, &buf[i], 1);
    if (ret != 0) {
      goto exit;
    }

    lsb++;

    /* Check if page wrap */
    if (((lsb & 0xFFU) == 0x00U) && (ret == 0)) {
      msb++;
      ret += readRegister(LSM6DSV16X_PAGE_SEL, (uint8_t *)&page_sel, 1);
      if (ret != 0) {
        goto exit;
      }

      page_sel.page_sel = msb;
      page_sel.not_used0 = 1; // Default value
      ret += writeRegister(LSM6DSV16X_PAGE_SEL, (uint8_t *)&page_sel, 1);
      if (ret != 0) {
        goto exit;
      }
    }
  }

  page_sel.page_sel = 0;
  page_sel.not_used0 = 1;// Default value
  ret += writeRegister(LSM6DSV16X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0) {
    goto exit;
  }

  /* unset page write */
  ret += readRegister(LSM6DSV16X_PAGE_RW, (uint8_t *)&page_rw, 1);
  page_rw.page_read = PROPERTY_DISABLE;
  page_rw.page_write = PROPERTY_DISABLE;
  ret += writeRegister(LSM6DSV16X_PAGE_RW, (uint8_t *)&page_rw, 1);

exit:
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}

int32_t LSM6DSV16X::reset_set(lsm6dsv16x_reset_t val)
{
  lsm6dsv16x_func_cfg_access_t func_cfg_access;
  lsm6dsv16x_ctrl3_t ctrl3;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL3, (uint8_t *)&ctrl3, 1);
  ret += readRegister(LSM6DSV16X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0) {
    return ret;
  }

  ctrl3.boot = ((uint8_t)val & 0x04U) >> 2;
  ctrl3.sw_reset = ((uint8_t)val & 0x02U) >> 1;
  func_cfg_access.sw_por = (uint8_t)val & 0x01U;

  ret = writeRegister(LSM6DSV16X_CTRL3, (uint8_t *)&ctrl3, 1);
  ret += writeRegister(LSM6DSV16X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);

  return ret;
}


int32_t LSM6DSV16X::reset_get(lsm6dsv16x_reset_t *val)
{
  lsm6dsv16x_func_cfg_access_t func_cfg_access;
  lsm6dsv16x_ctrl3_t ctrl3;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL3, (uint8_t *)&ctrl3, 1);
  ret += readRegister(LSM6DSV16X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0) {
    return ret;
  }

  switch ((ctrl3.sw_reset << 2) + (ctrl3.boot << 1) + func_cfg_access.sw_por) {
    case LSM6DSV16X_READY:
      *val = LSM6DSV16X_READY;
      break;

    case LSM6DSV16X_GLOBAL_RST:
      *val = LSM6DSV16X_GLOBAL_RST;
      break;

    case LSM6DSV16X_RESTORE_CAL_PARAM:
      *val = LSM6DSV16X_RESTORE_CAL_PARAM;
      break;

    case LSM6DSV16X_RESTORE_CTRL_REGS:
      *val = LSM6DSV16X_RESTORE_CTRL_REGS;
      break;

    default:
      *val = LSM6DSV16X_GLOBAL_RST;
      break;
  }

  return ret;
}

float_t LSM6DSV16X::from_sflp_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.061f;
}

float_t LSM6DSV16X::from_fs2_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.061f;
}

float_t LSM6DSV16X::from_fs4_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.122f;
}

float_t LSM6DSV16X::from_fs8_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.244f;
}

float_t LSM6DSV16X::from_fs16_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.488f;
}

float_t LSM6DSV16X::from_fs125_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 4.375f;
}

float_t LSM6DSV16X::from_fs250_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 8.750f;
}

float_t LSM6DSV16X::from_fs500_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 17.50f;
}

float_t LSM6DSV16X::from_fs1000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 35.0f;
}

float_t LSM6DSV16X::from_fs2000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 70.0f;
}

float_t LSM6DSV16X::from_fs4000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 140.0f;
}

float_t LSM6DSV16X::from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}

float_t LSM6DSV16X::from_lsb_to_nsec(uint32_t lsb)
{
  return ((float_t)lsb * 21750.0f);
}

float_t LSM6DSV16X::from_lsb_to_mv(int16_t lsb)
{
  return ((float_t)lsb) / 78.0f;
}

void LSM6DSV16X::bytecpy(uint8_t *target, uint8_t *source)
{
  if ((target != NULL) && (source != NULL)) {
    *target = *source;
  }
}

int32_t LSM6DSV16X::xl_self_test_set(lsm6dsv16x_xl_self_test_t val)
{
  lsm6dsv16x_ctrl10_t ctrl10;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL10, (uint8_t *)&ctrl10, 1);

  if (ret == 0) {
    ctrl10.st_xl = (uint8_t)val & 0x3U;
    ret = writeRegister(LSM6DSV16X_CTRL10, (uint8_t *)&ctrl10, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::gy_full_scale_get(lsm6dsv16x_gy_full_scale_t *val)
{
  lsm6dsv16x_ctrl6_t ctrl6;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_CTRL6, (uint8_t *)&ctrl6, 1);
  if (ret != 0) {
    return ret;
  }

  switch (ctrl6.fs_g) {
    case LSM6DSV16X_125dps:
      *val = LSM6DSV16X_125dps;
      break;

    case LSM6DSV16X_250dps:
      *val = LSM6DSV16X_250dps;
      break;

    case LSM6DSV16X_500dps:
      *val = LSM6DSV16X_500dps;
      break;

    case LSM6DSV16X_1000dps:
      *val = LSM6DSV16X_1000dps;
      break;

    case LSM6DSV16X_2000dps:
      *val = LSM6DSV16X_2000dps;
      break;

    case LSM6DSV16X_4000dps:
      *val = LSM6DSV16X_4000dps;
      break;

    default:
      *val = LSM6DSV16X_125dps;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::fifo_status_get(lsm6dsv16x_fifo_status_t *val)
{
  uint8_t buff[2];
  lsm6dsv16x_fifo_status2_t status;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FIFO_STATUS1, (uint8_t *)&buff[0], 2);
  if (ret != 0) {
    return ret;
  }

  bytecpy((uint8_t *)&status, &buff[1]);

  val->fifo_bdr = status.counter_bdr_ia;
  val->fifo_ovr = status.fifo_ovr_ia;
  val->fifo_full = status.fifo_full_ia;
  val->fifo_th = status.fifo_wtm_ia;

  val->fifo_level = (uint16_t)buff[1] & 0x01U;
  val->fifo_level = (val->fifo_level * 256U) + buff[0];

  return ret;
}

int32_t LSM6DSV16X::_6d_threshold_set(lsm6dsv16x_6d_threshold_t val)
{
  lsm6dsv16x_tap_ths_6d_t tap_ths_6d;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  if (ret == 0) {
    tap_ths_6d.sixd_ths = (uint8_t)val & 0x03U;
    ret = writeRegister(LSM6DSV16X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  }

  return ret;
}

float LSM6DSV16X::Convert_X_Sensitivity(lsm6dsv16x_xl_full_scale_t full_scale)
{
  float Sensitivity = 0.0f;
  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale) {
    case LSM6DSV16X_2g:
      Sensitivity = LSM6DSV16X_ACC_SENSITIVITY_FS_2G;
      break;

    case LSM6DSV16X_4g:
      Sensitivity = LSM6DSV16X_ACC_SENSITIVITY_FS_4G;
      break;

    case LSM6DSV16X_8g:
      Sensitivity = LSM6DSV16X_ACC_SENSITIVITY_FS_8G;
      break;

    case LSM6DSV16X_16g:
      Sensitivity = LSM6DSV16X_ACC_SENSITIVITY_FS_16G;
      break;
  }
  return Sensitivity;
}

int32_t LSM6DSV16X::tap_mode_set(lsm6dsv16x_tap_mode_t val)
{
  lsm6dsv16x_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  if (ret == 0) {
    wake_up_ths.single_double_tap = (uint8_t)val & 0x01U;
    ret = writeRegister(LSM6DSV16X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::tap_mode_get(lsm6dsv16x_tap_mode_t *val)
{
  lsm6dsv16x_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  if (ret != 0) {
    return ret;
  }

  switch (wake_up_ths.single_double_tap) {
    case LSM6DSV16X_ONLY_SINGLE:
      *val = LSM6DSV16X_ONLY_SINGLE;
      break;

    case LSM6DSV16X_BOTH_SINGLE_DOUBLE:
      *val = LSM6DSV16X_BOTH_SINGLE_DOUBLE;
      break;

    default:
      *val = LSM6DSV16X_ONLY_SINGLE;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::stpcnt_mode_get(lsm6dsv16x_stpcnt_mode_t *val)
{
  lsm6dsv16x_emb_func_en_a_t emb_func_en_a;
  lsm6dsv16x_pedo_cmd_reg_t pedo_cmd_reg;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  ret += readRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  ret = ln_pg_read(LSM6DSV16X_EMB_ADV_PG_1 + LSM6DSV16X_PEDO_CMD_REG, (uint8_t *)&pedo_cmd_reg, 1);
  if (ret != 0) {
    return ret;
  }

  val->false_step_rej = pedo_cmd_reg.fp_rejection_en;
  val->step_counter_enable = emb_func_en_a.pedo_en;

  return ret;
}


int32_t LSM6DSV16X::stpcnt_steps_get(uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  ret += readRegister(LSM6DSV16X_STEP_COUNTER_L, &buff[0], 2);
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  *val = buff[1];
  *val = (*val * 256U) + buff[0];

  return ret;
}


int32_t LSM6DSV16X::stpcnt_rst_step_set(uint8_t val)
{
  lsm6dsv16x_emb_func_src_t emb_func_src;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  ret = readRegister(LSM6DSV16X_EMB_FUNC_SRC, (uint8_t *)&emb_func_src, 1);
  if (ret != 0) {
    goto exit;
  }

  emb_func_src.pedo_rst_step = val;
  ret = writeRegister(LSM6DSV16X_EMB_FUNC_SRC, (uint8_t *)&emb_func_src, 1);

exit:
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}


int32_t LSM6DSV16X::stpcnt_rst_step_get(uint8_t *val)
{
  lsm6dsv16x_emb_func_src_t emb_func_src;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  ret = readRegister(LSM6DSV16X_EMB_FUNC_SRC, (uint8_t *)&emb_func_src, 1);
  *val = emb_func_src.pedo_rst_step;

  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}

int32_t LSM6DSV16X::stpcnt_debounce_set(uint8_t val)
{
  lsm6dsv16x_pedo_deb_steps_conf_t pedo_deb_steps_conf;
  int32_t ret;

  ret = ln_pg_read(LSM6DSV16X_EMB_ADV_PG_1 + LSM6DSV16X_PEDO_DEB_STEPS_CONF, (uint8_t *)&pedo_deb_steps_conf, 1);
  if (ret == 0) {
    pedo_deb_steps_conf.deb_step = val;
    ret = ln_pg_write(LSM6DSV16X_EMB_ADV_PG_1 + LSM6DSV16X_PEDO_DEB_STEPS_CONF, (uint8_t *)&pedo_deb_steps_conf, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::stpcnt_debounce_get(uint8_t *val)
{
  lsm6dsv16x_pedo_deb_steps_conf_t pedo_deb_steps_conf;
  int32_t ret;

  ret = ln_pg_read(LSM6DSV16X_EMB_ADV_PG_1 + LSM6DSV16X_PEDO_DEB_STEPS_CONF, (uint8_t *)&pedo_deb_steps_conf, 1);
  *val = pedo_deb_steps_conf.deb_step;

  return ret;
}


int32_t LSM6DSV16X::stpcnt_period_set(uint16_t val)
{
  uint8_t buff[2];
  int32_t ret;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));
  ret = ln_pg_write(LSM6DSV16X_EMB_ADV_PG_1 + LSM6DSV16X_PEDO_SC_DELTAT_L, (uint8_t *)&buff[0], 2);

  return ret;
}


int32_t LSM6DSV16X::stpcnt_period_get(uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = ln_pg_read(LSM6DSV16X_EMB_ADV_PG_1 + LSM6DSV16X_PEDO_SC_DELTAT_L, &buff[0], 2);
  if (ret != 0) {
    return ret;
  }

  *val = buff[1];
  *val = (*val * 256U) + buff[0];

  return ret;
}

int32_t LSM6DSV16X::ln_pg_read(uint16_t address, uint8_t *buf,
                              uint8_t len)
{
  lsm6dsv16x_page_address_t  page_address;
  lsm6dsv16x_page_sel_t page_sel;
  lsm6dsv16x_page_rw_t page_rw;
  uint8_t msb;
  uint8_t lsb;
  int32_t ret;
  uint8_t i ;

  msb = ((uint8_t)(address >> 8) & 0x0FU);
  lsb = (uint8_t)address & 0xFFU;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  /* set page write */
  ret += readRegister(LSM6DSV16X_PAGE_RW, (uint8_t *)&page_rw, 1);
  page_rw.page_read = PROPERTY_ENABLE;
  page_rw.page_write = PROPERTY_DISABLE;
  ret += writeRegister(LSM6DSV16X_PAGE_RW, (uint8_t *)&page_rw, 1);
  if (ret != 0) {
    goto exit;
  }

  /* select page */
  ret += readRegister(LSM6DSV16X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  page_sel.page_sel = msb;
  page_sel.not_used0 = 1; // Default value
  ret += writeRegister(LSM6DSV16X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0) {
    goto exit;
  }

  /* set page addr */
  page_address.page_addr = lsb;
  ret += writeRegister(LSM6DSV16X_PAGE_ADDRESS,
                              (uint8_t *)&page_address, 1);
  if (ret != 0) {
    goto exit;
  }

  for (i = 0; ((i < len) && (ret == 0)); i++) {
    ret += readRegister(LSM6DSV16X_PAGE_VALUE, &buf[i], 1);
    if (ret != 0) {
      goto exit;
    }

    lsb++;

    /* Check if page wrap */
    if (((lsb & 0xFFU) == 0x00U) && (ret == 0)) {
      msb++;
      ret += readRegister(LSM6DSV16X_PAGE_SEL, (uint8_t *)&page_sel, 1);
      if (ret != 0) {
        goto exit;
      }

      page_sel.page_sel = msb;
      page_sel.not_used0 = 1; // Default value
      ret += writeRegister(LSM6DSV16X_PAGE_SEL, (uint8_t *)&page_sel, 1);
      if (ret != 0) {
        goto exit;
      }
    }
  }

  page_sel.page_sel = 0;
  page_sel.not_used0 = 1;// Default value
  ret += writeRegister(LSM6DSV16X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0) {
    goto exit;
  }

  /* unset page write */
  ret += readRegister(LSM6DSV16X_PAGE_RW, (uint8_t *)&page_rw, 1);
  page_rw.page_read = PROPERTY_DISABLE;
  page_rw.page_write = PROPERTY_DISABLE;
  ret += writeRegister(LSM6DSV16X_PAGE_RW, (uint8_t *)&page_rw, 1);

exit:
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  return ret;
}

int32_t LSM6DSV16X::stpcnt_mode_set(lsm6dsv16x_stpcnt_mode_t val)
{
  lsm6dsv16x_emb_func_en_a_t emb_func_en_a;
  lsm6dsv16x_emb_func_en_b_t emb_func_en_b;
  lsm6dsv16x_pedo_cmd_reg_t pedo_cmd_reg;
  int32_t ret;

  ret = mem_bank_set(LSM6DSV16X_EMBED_FUNC_MEM_BANK);
  if (ret != 0) {
    return ret;
  }

  ret = readRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  ret += readRegister(LSM6DSV16X_EMB_FUNC_EN_B, (uint8_t *)&emb_func_en_b, 1);
  if (ret != 0) {
    goto exit;
  }

  if ((val.false_step_rej == PROPERTY_ENABLE)
      && ((emb_func_en_a.mlc_before_fsm_en & emb_func_en_b.mlc_en) ==
          PROPERTY_DISABLE)) {
    emb_func_en_a.mlc_before_fsm_en = PROPERTY_ENABLE;
  }

  emb_func_en_a.pedo_en = val.step_counter_enable;
  ret += writeRegister(LSM6DSV16X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);

exit:
  ret += mem_bank_set(LSM6DSV16X_MAIN_MEM_BANK);

  if (ret == 0) {
    ret = ln_pg_read(LSM6DSV16X_EMB_ADV_PG_1 + LSM6DSV16X_PEDO_CMD_REG, (uint8_t *)&pedo_cmd_reg, 1);
    pedo_cmd_reg.fp_rejection_en = val.false_step_rej;
    ret += ln_pg_write(LSM6DSV16X_EMB_ADV_PG_1 + LSM6DSV16X_PEDO_CMD_REG, (uint8_t *)&pedo_cmd_reg, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::ff_thresholds_set(lsm6dsv16x_ff_thresholds_t val)
{
  lsm6dsv16x_free_fall_t free_fall;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FREE_FALL, (uint8_t *)&free_fall, 1);
  if (ret == 0) {
    free_fall.ff_ths = (uint8_t)val & 0x7U;
    ret = writeRegister(LSM6DSV16X_FREE_FALL, (uint8_t *)&free_fall, 1);
  }

  return ret;
}


int32_t LSM6DSV16X::ff_thresholds_get(lsm6dsv16x_ff_thresholds_t *val)
{
  lsm6dsv16x_free_fall_t free_fall;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FREE_FALL, (uint8_t *)&free_fall, 1);
  if (ret != 0) {
    return ret;
  }

  switch (free_fall.ff_ths) {
    case LSM6DSV16X_156_mg:
      *val = LSM6DSV16X_156_mg;
      break;

    case LSM6DSV16X_219_mg:
      *val = LSM6DSV16X_219_mg;
      break;

    case LSM6DSV16X_250_mg:
      *val = LSM6DSV16X_250_mg;
      break;

    case LSM6DSV16X_312_mg:
      *val = LSM6DSV16X_312_mg;
      break;

    case LSM6DSV16X_344_mg:
      *val = LSM6DSV16X_344_mg;
      break;

    case LSM6DSV16X_406_mg:
      *val = LSM6DSV16X_406_mg;
      break;

    case LSM6DSV16X_469_mg:
      *val = LSM6DSV16X_469_mg;
      break;

    case LSM6DSV16X_500_mg:
      *val = LSM6DSV16X_500_mg;
      break;

    default:
      *val = LSM6DSV16X_156_mg;
      break;
  }

  return ret;
}

int32_t LSM6DSV16X::ff_time_windows_set(uint8_t val)
{
  lsm6dsv16x_wake_up_dur_t wake_up_dur;
  lsm6dsv16x_free_fall_t free_fall;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  wake_up_dur.ff_dur = ((uint8_t)val & 0x20U) >> 5;
  ret += writeRegister(LSM6DSV16X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret != 0) {
    return ret;
  }

  ret = readRegister(LSM6DSV16X_FREE_FALL, (uint8_t *)&free_fall, 1);
  free_fall.ff_dur = (uint8_t)val & 0x1FU;
  ret += writeRegister(LSM6DSV16X_FREE_FALL, (uint8_t *)&free_fall, 1);

  return ret;
}


int32_t LSM6DSV16X::ff_time_windows_get(uint8_t *val)
{
  lsm6dsv16x_wake_up_dur_t wake_up_dur;
  lsm6dsv16x_free_fall_t free_fall;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  ret += writeRegister(LSM6DSV16X_FREE_FALL, (uint8_t *)&free_fall, 1);

  *val = (wake_up_dur.ff_dur << 5) + free_fall.ff_dur;

  return ret;
}

int32_t LSM6DSV16X::act_thresholds_get(lsm6dsv16x_act_thresholds_t *val)
{
  lsm6dsv16x_inactivity_dur_t inactivity_dur;
  lsm6dsv16x_inactivity_ths_t inactivity_ths;
  lsm6dsv16x_wake_up_ths_t wake_up_ths;
  lsm6dsv16x_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  ret += readRegister(LSM6DSV16X_INACTIVITY_THS, (uint8_t *)&inactivity_ths, 1);
  ret += readRegister(LSM6DSV16X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  ret += readRegister(LSM6DSV16X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret != 0) {
    return ret;
  }

  val->inactivity_cfg.wu_inact_ths_w = inactivity_dur.wu_inact_ths_w;
  val->inactivity_cfg.xl_inact_odr = inactivity_dur.xl_inact_odr;
  val->inactivity_cfg.inact_dur = inactivity_dur.inact_dur;

  val->inactivity_ths = inactivity_ths.inact_ths;
  val->threshold = wake_up_ths.wk_ths;
  val->duration = wake_up_dur.wake_dur;

  return ret;
}

int32_t LSM6DSV16X::act_wkup_time_windows_set(lsm6dsv16x_act_wkup_time_windows_t val)
{
  lsm6dsv16x_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret == 0) {
    wake_up_dur.wake_dur = val.shock;
    wake_up_dur.sleep_dur = val.quiet;
    ret = writeRegister(LSM6DSV16X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::act_wkup_time_windows_get(lsm6dsv16x_act_wkup_time_windows_t *val)
{
  lsm6dsv16x_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret != 0) {
    return ret;
  }

  val->shock = wake_up_dur.wake_dur;
  val->quiet = wake_up_dur.sleep_dur;

  return ret;
}

int32_t LSM6DSV16X::act_thresholds_set(lsm6dsv16x_act_thresholds_t *val)
{
  lsm6dsv16x_inactivity_ths_t inactivity_ths;
  lsm6dsv16x_inactivity_dur_t inactivity_dur;
  lsm6dsv16x_wake_up_ths_t wake_up_ths;
  lsm6dsv16x_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  ret += readRegister(LSM6DSV16X_INACTIVITY_THS, (uint8_t *)&inactivity_ths, 1);
  ret += readRegister(LSM6DSV16X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  ret += readRegister(LSM6DSV16X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret != 0) {
    return ret;
  }

  inactivity_dur.wu_inact_ths_w = val->inactivity_cfg.wu_inact_ths_w;
  inactivity_dur.xl_inact_odr = val->inactivity_cfg.xl_inact_odr;
  inactivity_dur.inact_dur = val->inactivity_cfg.inact_dur;

  inactivity_ths.inact_ths = val->inactivity_ths;
  wake_up_ths.wk_ths = val->threshold;
  wake_up_dur.wake_dur = val->duration;

  ret += writeRegister(LSM6DSV16X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  ret += writeRegister(LSM6DSV16X_INACTIVITY_THS, (uint8_t *)&inactivity_ths, 1);
  ret += writeRegister(LSM6DSV16X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  ret += writeRegister(LSM6DSV16X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);

  return ret;
}

int32_t LSM6DSV16X::fifo_watermark_set(uint8_t val)
{
  lsm6dsv16x_fifo_ctrl1_t fifo_ctrl1;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FIFO_CTRL1, (uint8_t *)&fifo_ctrl1, 1);

  if (ret == 0) {
    fifo_ctrl1.wtm = val;
    ret = writeRegister(LSM6DSV16X_FIFO_CTRL1, (uint8_t *)&fifo_ctrl1, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::fifo_stop_on_wtm_set(uint8_t val)
{
  lsm6dsv16x_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret == 0) {
    fifo_ctrl2.stop_on_wtm = val;
    ret = writeRegister(LSM6DSV16X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::fifo_xl_batch_set(lsm6dsv16x_fifo_xl_batch_t val)
{
  lsm6dsv16x_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  if (ret == 0) {
    fifo_ctrl3.bdr_xl = (uint8_t)val & 0xFu;
    ret = writeRegister(LSM6DSV16X_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::fifo_gy_batch_set(lsm6dsv16x_fifo_gy_batch_t val)
{
  lsm6dsv16x_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  if (ret == 0) {
    fifo_ctrl3.bdr_gy = (uint8_t)val & 0x0Fu;
    ret = writeRegister(LSM6DSV16X_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::timestamp_set(uint8_t val)
{
  lsm6dsv16x_functions_enable_t functions_enable;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret == 0) {
    functions_enable.timestamp_en = val;
    ret = writeRegister(LSM6DSV16X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  }

  return ret;
}

int32_t LSM6DSV16X::fifo_timestamp_batch_set(lsm6dsv16x_fifo_timestamp_batch_t val)
{
  lsm6dsv16x_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = readRegister(LSM6DSV16X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  if (ret == 0) {
    fifo_ctrl4.dec_ts_batch = (uint8_t)val & 0x03U;
    ret = writeRegister(LSM6DSV16X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  }

  return ret;
}
