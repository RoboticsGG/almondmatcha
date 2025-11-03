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

#include "STTS22H.h"
#include <cstdint>
#include "mbed.h"



bool STTS22H::readRegister(uint8_t reg, uint8_t *value, uint16_t len) {
    if (i2c->write(stts22h_8bit_address, (const char*)&reg, 1) != 0)
        return 1;
    if (i2c->read(stts22h_8bit_address, (char*) value, len) != 0)
        return 1;

    bool ret = 0;
    return ret;
}

bool STTS22H::writeRegister(uint8_t reg, const uint8_t *value, uint16_t len) {
    uint8_t data[len + 1];
    data[0] = reg; // insert the byte of register in the first position

    // copies the values in data
    for (uint16_t i = 0; i < len; ++i) {
        data[i + 1] = value[i];
    }

    // write data into the register
    if (i2c->write(stts22h_8bit_address, (const char*)data, len + 1) != 0)
        return 1;
    return 0;
}

/**
 * @brief Constructor for the LSM6DSV16X class.
 * 
 * Initializes the LSM6DSV16X object with the specified I2C pins.
 * 
 * @param sda PinName of the data line for I2C communication.
 * @param scl PinName of the clock line for I2C communication.
 */
STTS22H::STTS22H(PinName sda, PinName scl) {
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
STTS22H::STTS22H(PinName mosi, PinName miso, PinName sck, PinName cs) {
    // Inizializzazione del sensore tramite SPI
    initialize();
}

/**
 * @brief Destructor for the LSM6DSV16X class.
 * 
 * Cleans up resources allocated for I2C and SPI communication.
 */
STTS22H::~STTS22H() {
    delete i2c; // Deallocate memory for I2C communication
    delete spi; // Deallocate memory for SPI communication
    delete cs_pin; // Deallocate memory for the Chip Select pin
}

/**
 * @brief Initialize the LSM6DSV16X device.
 * 
 * Configures communication settings based on the selected interface (I2C or SPI).
 */
void STTS22H::initialize() {
    if (i2c) {
        i2c->frequency(400000); // Set I2C frequency to 400kHz
    } else if (spi) {
        cs_pin->write(0); // Set Chip Select pin low to enable communication
        cs_pin->write(1); // Set Chip Select pin high to disable communication
    }
}


STTS22HStatusTypeDef STTS22H::begin(){
    /* Set default ODR */
  temp_odr = 1.0f;

  /* Enable BDU */
  if(block_data_update_set(PROPERTY_ENABLE) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  /* Enable Automatic Address Increment */
  if(auto_increment_set(PROPERTY_ENABLE) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  /* Put the component in standby mode. */
  if (temp_data_rate_set(STTS22H_POWER_DOWN) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }
  
  temp_is_enabled = 0U;

  return STTS22H_OK;
}


/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::end()
{
  /* Disable temperature sensor */
  if (Disable() != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}



/**
 * @brief  Get WHO_AM_I value
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::ReadID(uint8_t *Id)
{
  uint8_t buf;

  if (dev_id_get(&buf) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  *Id = buf;

  return STTS22H_OK;
}

/**
 * @brief  Enable the STTS22H temperature sensor
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::Enable()
{
  /* Check if the component is already enabled */
  if (temp_is_enabled == 1U)
  {
    return STTS22H_OK;
  }

  /* Power on the component and set the odr. */
  if (SetOutputDataRate(temp_odr) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  temp_is_enabled = 1U;

  return STTS22H_OK;
}

/**
 * @brief  Disable the STTS22H temperature sensor
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::Disable()
{
  /* Check if the component is already disabled */
  if (temp_is_enabled == 0U)
  {
    return STTS22H_OK;
  }

  /* Save the current odr. */
  if (GetOutputDataRate(&temp_odr) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }
  
  /* Put the component in standby mode. */
  if (temp_data_rate_set(STTS22H_POWER_DOWN) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  temp_is_enabled = 0U;

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H temperature sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::GetOutputDataRate(float *Odr)
{
  STTS22HStatusTypeDef ret = STTS22H_OK;
  stts22h_odr_temp_t odr_low_level;

  if (temp_data_rate_get(&odr_low_level) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  switch (odr_low_level)
  {
    case STTS22H_POWER_DOWN:
    case STTS22H_ONE_SHOT:
      *Odr = 0.0f;
      break;

    case STTS22H_1Hz:
      *Odr = 1.0f;
      break;

    case STTS22H_25Hz:
      *Odr = 25.0f;
      break;

    case STTS22H_50Hz:
      *Odr = 50.0f;
      break;

    case STTS22H_100Hz:
      *Odr = 100.0f;
      break;

    case STTS22H_200Hz:
      *Odr = 200.0f;
      break;

    default:
      ret = STTS22H_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the STTS22H temperature sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::SetOutputDataRate(float Odr)
{
  stts22h_odr_temp_t new_odr;

  new_odr = (Odr <= 1.0f   ) ? STTS22H_1Hz
          : (Odr <= 25.0f  ) ? STTS22H_25Hz
          : (Odr <= 50.0f  ) ? STTS22H_50Hz
          : (Odr <= 100.0f ) ? STTS22H_100Hz
          :                    STTS22H_200Hz;

  if (temp_data_rate_set(new_odr) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H temperature value
 * @param  Value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::GetTemperature(float *Value)
{
  int16_t raw_value = 0;

  /* Get the temperature */
  if (temperature_raw_get(&raw_value) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  *Value = from_lsb_to_celsius(raw_value);

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H temperature data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::Get_DRDY_Status(uint8_t *Status)
{
  uint8_t val;

  if (temp_flag_data_ready_get( &val) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  if(val)
  {
    *Status = 1;
  } else
  {
    *Status = 0;
  }

  return STTS22H_OK;
}

/**
 * @brief  Set the STTS22H low temperature threshold value
 * @param  Value the high temperature threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::SetLowTemperatureThreshold(float Value)
{
  uint8_t raw_value;

  raw_value = (uint8_t)((Value / 0.64f) + 63.0f);

  /* Set the temperature threshold */
  if (temp_trshld_low_set(raw_value) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Set the STTS22H high temperature threshold value
 * @param  Value the high temperature threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::SetHighTemperatureThreshold(float Value)
{
  uint8_t raw_value;

  raw_value = (uint8_t)((Value / 0.64f) + 63.0f);

  /* Set the temperature threshold */
  if (temp_trshld_high_set(raw_value) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H temperature limits status
 * @param  HighLimit indicates that high temperature limit has been exceeded
 * @param  LowhLimit indicates that low temperature limit has been exceeded
 * @param  ThermLimit indicates that therm temperature limit has been exceeded
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::GetTemperatureLimitStatus(uint8_t *HighLimit, uint8_t *LowLimit)
{
  stts22h_temp_trlhd_src_t status;

  /* Read status register */
  if (temp_trshld_src_get(&status) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  if(HighLimit)
  {
    *HighLimit = status.over_thh;
  }

  if(LowLimit)
  {
    *LowLimit = status.under_thl;
  }

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H register value
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (readRegister(Reg, Data, 1) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Set the STTS22H register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (writeRegister(Reg, &Data, 1) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Set the STTS22H One Shot Mode
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::Set_One_Shot()
{
  /* Start One Shot Measurement */
  if(temp_data_rate_set(STTS22H_ONE_SHOT) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H One Shot Status
 * @param  Status pointer to the one shot status (1 means measurements available, 0 means measurements not available yet)
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22H::Get_One_Shot_Status(uint8_t *Status)
{
  stts22h_dev_status_t status;

  /* Get Busy flag */
  if(dev_status_get(&status) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  if(status.busy)
  {
    *Status = 0;
  }
  else
  {
    *Status = 1;
  }

  return STTS22H_OK;
}


//***************************//
//          PRIVATE
//***************************//
float_t from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb /100.0f);
}


/**
  * @brief  Block data update.[set]
  *
  * @param  val    Change the values of bdu in reg CTRL.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::block_data_update_set(uint8_t val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = readRegister(STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  if(ret == 0){
    ctrl.bdu = val;
    ret = writeRegister(STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  }
  return ret;
}


float_t STTS22H::from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb /100.0f);
}


/**
  * @brief  Temperature sensor data rate selection..[set]
  *
  * @param  val    Change the values of "one_shot" in reg STTS22H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::temp_data_rate_set(stts22h_odr_temp_t val)
{
  stts22h_software_reset_t software_reset;
  stts22h_ctrl_t ctrl;
  int32_t ret;
 
  ret = readRegister(STTS22H_CTRL, (uint8_t*)&ctrl, 1);

  if ( ret == 0 ) {
    ret = readRegister(STTS22H_SOFTWARE_RESET, (uint8_t*)&software_reset, 1);
  }
  
  if ( ( val == STTS22H_ONE_SHOT ) && ( ret == 0 ) ) {
    software_reset.sw_reset = PROPERTY_ENABLE;
    ret = writeRegister(STTS22H_SOFTWARE_RESET, (uint8_t*)&software_reset, 1);
    if ( ret == 0 ) {
      software_reset.sw_reset = PROPERTY_DISABLE;
      ret = writeRegister(STTS22H_SOFTWARE_RESET, (uint8_t*)&software_reset, 1);
    }
  }

  if ( ( ( val == STTS22H_25Hz )  || ( val == STTS22H_50Hz  )   ||  
         ( val == STTS22H_100Hz ) || ( val == STTS22H_200Hz ) ) && 
       ( ctrl.freerun == PROPERTY_DISABLE ) && ( ret == 0 ) ) {
    software_reset.sw_reset = PROPERTY_ENABLE;
    ret = writeRegister(STTS22H_SOFTWARE_RESET, (uint8_t*)&software_reset, 1);
    if ( ret == 0 ) {
      software_reset.sw_reset = PROPERTY_DISABLE;
      ret = writeRegister(STTS22H_SOFTWARE_RESET, (uint8_t*)&software_reset, 1);
    }
  }

  if ( ( val == STTS22H_1Hz ) && ( ret == 0 ) ) {
    software_reset.sw_reset = PROPERTY_ENABLE;
    software_reset.low_odr_enable = PROPERTY_ENABLE;
    ret = writeRegister(STTS22H_SOFTWARE_RESET, (uint8_t*)&software_reset, 1);   
    if ( ret == 0 ) {
      software_reset.sw_reset = PROPERTY_DISABLE;
      software_reset.low_odr_enable = PROPERTY_ENABLE;
      ret = writeRegister(STTS22H_SOFTWARE_RESET, (uint8_t*)&software_reset, 1);
    }
  }

  if ( ret == 0 ) {
    ctrl.one_shot = (uint8_t)val & 0x01U;
    ctrl.freerun = ((uint8_t)val & 0x02U) >> 1;
    ctrl.low_odr_start = ((uint8_t)val & 0x04U) >> 2;
    ctrl.avg = ((uint8_t)val & 0x30U) >> 4;
    ret = writeRegister(STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Temperature sensor data rate selection..[get]
  *
  * @param  val    Get the values of one_shot in reg CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::temp_data_rate_get(stts22h_odr_temp_t *val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = readRegister(STTS22H_CTRL,
                            (uint8_t*)&ctrl, 1);
  switch ( ctrl.one_shot | (ctrl.freerun << 1) | (ctrl.low_odr_start << 2) |
           (ctrl.avg << 4)){
    case STTS22H_POWER_DOWN:
      *val = STTS22H_POWER_DOWN;
      break;
    case STTS22H_ONE_SHOT:
      *val = STTS22H_ONE_SHOT;
      break;
    case STTS22H_1Hz:
      *val = STTS22H_1Hz;
      break;
    case STTS22H_25Hz:
      *val = STTS22H_25Hz;
      break;
    case STTS22H_50Hz:
      *val = STTS22H_50Hz;
      break;
    case STTS22H_100Hz:
      *val = STTS22H_100Hz;
      break;
    case STTS22H_200Hz:
      *val = STTS22H_200Hz;
      break;
    default:
      *val = STTS22H_POWER_DOWN;
      break;
  }
  return ret;
}


/**
  * @brief  Block data update.[get]
  *
  * @param  val    Get the values of bdu in reg CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::block_data_update_get(uint8_t *val)
{
  int32_t ret;
  ret = readRegister(STTS22H_CTRL, (uint8_t*)val, 1);
  return ret;
}

/**
  * @brief    New data available from temperature sensor..[get]
  *
  * @param  val    Return an option of "stts22h_uint8_t".(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::temp_flag_data_ready_get(uint8_t *val)
{
  stts22h_status_t status;
  int32_t ret;
  ret = readRegister(STTS22H_STATUS, (uint8_t*)&status, 1);
  if (status.busy == PROPERTY_DISABLE){
    *val = PROPERTY_ENABLE;
  }
  else{
    *val = PROPERTY_DISABLE;   
  }
  return ret;
}


/**
  * @brief   Temperature data output register(r). L and H registers
  *          together express a 16-bit word in two’s complement..[get]
  *
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::temperature_raw_get(int16_t *buff)
{  
  uint16_t temperature;
  uint8_t temperature_low;
  int32_t ret;

  ret = readRegister(STTS22H_TEMP_L_OUT,
                          &temperature_low, 1);
  if (ret == 0) {
    ret = readRegister(STTS22H_TEMP_H_OUT,
                           (uint8_t*)&temperature, 1);

    temperature  = (temperature << 8) + temperature_low;
    *buff = (int16_t)temperature;
  }  
  
  return ret;
}


/**
  * @brief  Device Who am I..[get]
  *
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::dev_id_get(uint8_t *buff)
{
  int32_t ret;
  ret = readRegister(STTS22H_WHOAMI, buff, 1);
  return ret;
}
/**
  * @brief   Device status register.[get]
  *
  * @param  val    In one-shot mode this bit is high when the
  *                conversion is in progress..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::dev_status_get(stts22h_dev_status_t *val)
{
  stts22h_status_t status;
  int32_t ret;

  ret = readRegister(STTS22H_STATUS, (uint8_t*)&status, 1);
  val->busy = status.busy;

  return ret;
}


/**
  * @brief  SMBus mode..[set]
  *
  * @param  val    Change the values of "time_out_dis" in reg STTS22H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::smbus_interface_set(stts22h_smbus_md_t val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = readRegister(STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  if(ret == 0){
    ctrl.time_out_dis = (uint8_t)val;
    ret = writeRegister(STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  }
  return ret;
}

/**
  * @brief  SMBus mode..[get]
  *
  * @param  val    Get the values of time_out_dis in reg CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::smbus_interface_get(stts22h_smbus_md_t *val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = readRegister(STTS22H_CTRL,
                            (uint8_t*)&ctrl, 1);
  switch (ctrl.time_out_dis){
    case STTS22H_SMBUS_TIMEOUT_ENABLE:
      *val = STTS22H_SMBUS_TIMEOUT_ENABLE;
      break;
    case STTS22H_SMBUS_TIMEOUT_DISABLE:
      *val = STTS22H_SMBUS_TIMEOUT_DISABLE;
      break;
    default:
      *val = STTS22H_SMBUS_TIMEOUT_ENABLE;
      break;
  }
  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple
  *         byte access with a serial interface.[set]
  *
  * @param  val    Change the values of "if_add_inc" in reg STTS22H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::auto_increment_set(uint8_t val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = readRegister(STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  if(ret == 0){
    ctrl.if_add_inc = (uint8_t)val;
    ret = writeRegister(STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  }
  return ret;
}

/**
  * @brief   Register address automatically incremented during a multiple
  *          byte access with a serial interface.[get]
  *
  * @param  val    Get the values of if_add_inc in reg CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::auto_increment_get(uint8_t *val)
{
  int32_t ret;
  ret = readRegister(STTS22H_CTRL, (uint8_t*)&val, 1);
  return ret;
}


/**
  * @brief  Over temperature interrupt value. ( degC / 0.64 ) + 63.[set]
  *
  * @param  val    Change the values of thl in reg TEMP_H_LIMIT.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::temp_trshld_high_set(uint8_t val)
{
  stts22h_temp_h_limit_t temp_h_limit;
  int32_t ret;

  ret = readRegister(STTS22H_TEMP_H_LIMIT,
                         (uint8_t*)&temp_h_limit, 1);
  if(ret == 0){
    temp_h_limit.thl = val;
    ret = writeRegister(STTS22H_TEMP_H_LIMIT,
                            (uint8_t*)&temp_h_limit, 1);
  }
  return ret;
}

/**
  * @brief  Over temperature interrupt value. ( degC / 0.64 ) + 63.[get]
  *
  * @param  val    Get the values of thl in reg TEMP_H_LIMIT.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::temp_trshld_high_get(uint8_t *val)
{
  stts22h_temp_h_limit_t temp_h_limit;
  int32_t ret;

  ret = readRegister(STTS22H_TEMP_H_LIMIT,
                         (uint8_t*)&temp_h_limit, 1);
  *val = temp_h_limit.thl;

  return ret;
}

/**
  * @brief   Under temperature interrupt value. ( degC / 0.64 ) + 63.[set]
  *
  * @param  val    Change the values of tll in reg TEMP_L_LIMIT.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::temp_trshld_low_set(uint8_t val)
{
  stts22h_temp_l_limit_t temp_l_limit;
  int32_t ret;

  ret = readRegister(STTS22H_TEMP_L_LIMIT,
                         (uint8_t*)&temp_l_limit, 1);
  if(ret == 0){
    temp_l_limit.tll = val;
    ret = writeRegister(STTS22H_TEMP_L_LIMIT,
                            (uint8_t*)&temp_l_limit, 1);
  }
  return ret;
}

/**
  * @brief   Under temperature interrupt value. ( degC / 0.64 ) + 63.[get]
  *
  * @param  val    Get the values of tll in reg TEMP_L_LIMIT.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::temp_trshld_low_get(uint8_t *val)
{
  stts22h_temp_l_limit_t temp_l_limit;
  int32_t ret;

  ret = readRegister(STTS22H_TEMP_L_LIMIT,
                            (uint8_t*)&temp_l_limit, 1);
  *val = temp_l_limit.tll;

  return ret;
}

/**
  * @brief   Temperature interrupt on threshold source.[get]
  *
  * @param  val     Low limit temperature exceeded..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t STTS22H::temp_trshld_src_get(stts22h_temp_trlhd_src_t *val)
{
  stts22h_status_t status;
  int32_t ret;

  ret = readRegister(STTS22H_STATUS, (uint8_t*)&status, 1);
  val->under_thl = status.under_thl;
  val->over_thh = status.over_thh;

  return ret;
}





