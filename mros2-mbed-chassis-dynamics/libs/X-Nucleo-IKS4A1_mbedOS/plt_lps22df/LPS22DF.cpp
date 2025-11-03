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

#include "mbed.h"
#include "LPS22DF.h"
#include "LPS22DF_registers.h"



bool LPS22DF::readRegister(uint8_t reg, uint8_t *value, uint16_t len) {
    if (i2c->write(stts22h_8bit_address, (const char*)&reg, 1) != 0)
        return 1;
    if (i2c->read(stts22h_8bit_address, (char*) value, len) != 0)
        return 1;

    bool ret = 0;
    return ret;
}

bool LPS22DF::writeRegister(uint8_t reg, const uint8_t *value, uint16_t len) {
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
LPS22DF::LPS22DF(PinName sda, PinName scl) {
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
LPS22DF::LPS22DF(PinName mosi, PinName miso, PinName sck, PinName cs) {
    // Inizializzazione del sensore tramite SPI
    initialize();
}

/**
 * @brief Destructor for the LSM6DSV16X class.
 * 
 * Cleans up resources allocated for I2C and SPI communication.
 */
LPS22DF::~LPS22DF() {
    delete i2c; // Deallocate memory for I2C communication
    delete spi; // Deallocate memory for SPI communication
    delete cs_pin; // Deallocate memory for the Chip Select pin
}

/**
 * @brief Initialize the LSM6DSV16X device.
 * 
 * Configures communication settings based on the selected interface (I2C or SPI).
 */
void LPS22DF::initialize() {
    if (i2c) {
        i2c->frequency(400000); // Set I2C frequency to 400kHz
    } else if (spi) {
        cs_pin->write(0); // Set Chip Select pin low to enable communication
        cs_pin->write(1); // Set Chip Select pin high to disable communication
    }
}


/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::begin()
{
  lps22df_md_t md;
  lps22df_bus_mode_t bus_mode;

  if (spi) {
    // Configure CS pin
    cs_pin->write(1);
  }

  /* Set bdu and if_inc recommended for driver usage */
  if (lps22df_init_set(LPS22DF_DRV_RDY) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  /* Select bus interface */
  if (bus_type == LPS22DF_SPI_3WIRES_BUS) { /* SPI 3-Wires */
    bus_mode.interface = lps22df_bus_mode_t::LPS22DF_SPI_3W;
  } else if (bus_type == LPS22DF_SPI_4WIRES_BUS) { /* SPI 3-Wires */
    bus_mode.interface = lps22df_bus_mode_t::LPS22DF_SPI_4W;
  } else {
    bus_mode.interface = lps22df_bus_mode_t::LPS22DF_SEL_BY_HW;
  }

  bus_mode.filter = lps22df_bus_mode_t::LPS22DF_AUTO;
  if (lps22df_bus_mode_set(&bus_mode) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  /* Set Output Data Rate in Power Down */
  md.odr = lps22df_md_t::LPS22DF_ONE_SHOT;
  md.avg = lps22df_md_t::LPS22DF_4_AVG;
  md.lpf = lps22df_md_t::LPS22DF_LPF_ODR_DIV_4;

  /* Power down the device */
  if (lps22df_mode_set(&md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  if (lps22df_mode_get(&last_md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  last_md.odr = lps22df_md_t::LPS22DF_25Hz;
  enabled = 0L;

  initialized = 1L;

  return LPS22DF_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::end()
{
  if (initialized == 1U && Disable() != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  /* Reset CS configuration */
  if (spi) {
    // Configure CS pin
     cs_pin->write(0);
  }

  initialized = 0L;

  return LPS22DF_OK;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::ReadID(uint8_t *Id)
{
  if (lps22df_id_get((lps22df_id_t *)Id) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  return LPS22DF_OK;
}

/**
 * @brief  Enable the LPS22DF pressure sensor
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::Enable()
{
  /* Check if the component is already enabled */
  if (enabled == 1U) {
    return LPS22DF_OK;
  }

  /* Output data rate selection. */
  if (lps22df_mode_set(&last_md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  enabled = 1U;

  return LPS22DF_OK;
}

/**
 * @brief  Disable the LPS22DF pressure sensor
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::Disable()
{
  /* Check if the component is already disabled */
  if (enabled == 0U) {
    return LPS22DF_OK;
  }

  lps22df_md_t val;

  /* Get current output data rate. */
  if (lps22df_mode_get(&val) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  memcpy(&last_md, &val, sizeof(lps22df_md_t));

  val.odr = lps22df_md_t::LPS22DF_ONE_SHOT;

  /* Output data rate selection - power down. */
  if (lps22df_mode_set(&val) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  enabled = 0L;

  return LPS22DF_OK;
}

/**
 * @brief  Get output data rate
 * @param  Odr the output data rate value
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::GetOutputDataRate(float *Odr)
{
  LPS22DFStatusTypeDef ret = LPS22DF_OK;
  lps22df_md_t val;

  if (lps22df_mode_get(&val) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  switch (val.odr) {
    case lps22df_md_t::LPS22DF_ONE_SHOT:
      *Odr = 0.0f;
      break;

    case lps22df_md_t::LPS22DF_1Hz:
      *Odr = 1.0f;
      break;

    case lps22df_md_t::LPS22DF_4Hz:
      *Odr = 4.0f;
      break;

    case lps22df_md_t::LPS22DF_10Hz:
      *Odr = 10.0f;
      break;

    case lps22df_md_t::LPS22DF_25Hz:
      *Odr = 25.0f;
      break;

    case lps22df_md_t::LPS22DF_50Hz:
      *Odr = 50.0f;
      break;

    case lps22df_md_t::LPS22DF_75Hz:
      *Odr = 75.0f;
      break;

    case lps22df_md_t::LPS22DF_100Hz:
      *Odr = 100.0f;
      break;

    case lps22df_md_t::LPS22DF_200Hz:
      *Odr = 200.0f;
      break;

    default:
      ret = LPS22DF_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LPS22DF pressure sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::SetOutputDataRate(float Odr)
{
  /* Check if the component is enabled */
  if (enabled == 1U) {
    return SetOutputDataRate_When_Enabled(Odr);
  } else {
    return SetOutputDataRate_When_Disabled(Odr);
  }
}

/**
 * @brief  Set output data rate when enabled
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::SetOutputDataRate_When_Enabled(float Odr)
{
  lps22df_md_t new_val;

  if (lps22df_mode_get(&new_val) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  new_val.odr = (Odr <=   1.0f) ? lps22df_md_t::LPS22DF_1Hz
                : (Odr <=   4.0f) ? lps22df_md_t::LPS22DF_4Hz
                : (Odr <=  10.0f) ? lps22df_md_t::LPS22DF_10Hz
                : (Odr <=  25.0f) ? lps22df_md_t::LPS22DF_25Hz
                : (Odr <=  50.0f) ? lps22df_md_t::LPS22DF_50Hz
                : (Odr <=  75.0f) ? lps22df_md_t::LPS22DF_75Hz
                : (Odr <= 100.0f) ? lps22df_md_t::LPS22DF_100Hz
                :                   lps22df_md_t::LPS22DF_200Hz;

  if (lps22df_mode_set(&new_val) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  if (lps22df_mode_get(&last_md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  return LPS22DF_OK;
}

/**
 * @brief  Set output data rate when disabled
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::SetOutputDataRate_When_Disabled(float Odr)
{
  last_md.odr = (Odr <=   1.0f) ? lps22df_md_t::LPS22DF_1Hz
                : (Odr <=   4.0f) ? lps22df_md_t::LPS22DF_4Hz
                : (Odr <=  10.0f) ? lps22df_md_t::LPS22DF_10Hz
                : (Odr <=  25.0f) ? lps22df_md_t::LPS22DF_25Hz
                : (Odr <=  50.0f) ? lps22df_md_t::LPS22DF_50Hz
                : (Odr <=  75.0f) ? lps22df_md_t::LPS22DF_75Hz
                : (Odr <= 100.0f) ? lps22df_md_t::LPS22DF_100Hz
                :                   lps22df_md_t::LPS22DF_200Hz;

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF pressure value
 * @param  Value pointer where the pressure value is written
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::GetPressure(float *Value)
{
  lps22df_data_t data;

  if (lps22df_data_get(&data) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  *Value = data.pressure.hpa;

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF pressure data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::Get_PRESS_DRDY_Status(uint8_t *Status)
{
  lps22df_status_t reg;

  if (readRegister(LPS22DF_STATUS, (uint8_t *) &reg, 1) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  *Status = reg.p_da;

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF temperature value
 * @param  Value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::GetTemperature(float *Value)
{
  lps22df_data_t data;

  if (lps22df_data_get(&data) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  *Value = data.heat.deg_c;

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF temperature data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::Get_TEMP_DRDY_Status(uint8_t *Status)
{
  lps22df_status_t reg;

  if (readRegister(LPS22DF_STATUS, (uint8_t *) &reg, 1) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  *Status = reg.t_da;

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (readRegister(Reg, Data, 1) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  return LPS22DF_OK;
}

/**
 * @brief  Set the LPS22DF register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (writeRegister(Reg, &Data, 1) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  return LPS22DF_OK;
}

/**
 * @brief  Set the LPS22DF One Shot Mode
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::Set_One_Shot()
{
  lps22df_md_t md;

  if (lps22df_mode_get(&md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  /* Start One Shot Measurement */
  if (lps22df_trigger_sw(&md) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  return LPS22DF_OK;
}

/**
 * @brief  Get the LPS22DF One Shot Status
 * @param  Status pointer to the one shot status (1 means measurements available, 0 means measurements not available yet)
 * @retval 0 in case of success, an error code otherwise
 */
LPS22DFStatusTypeDef LPS22DF::Get_One_Shot_Status(uint8_t *Status)
{
  uint8_t p_da;
  uint8_t t_da;

  /* Get DataReady for pressure */
  if (Get_PRESS_DRDY_Status(&p_da) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  /* Get DataReady for temperature */
  if (Get_TEMP_DRDY_Status(&t_da) != LPS22DF_OK) {
    return LPS22DF_ERROR;
  }

  if (p_da && t_da) {
    *Status = 1L;
  } else {
    *Status = 0L;
  }

  return LPS22DF_OK;
}



/************************/
/*        PRIVATE       */
/************************/
static void bytecpy(uint8_t *target, uint8_t *source)
{
  if ((target != NULL) && (source != NULL)) {
    *target = *source;
  }
}


float_t LPS22DF::lps22df_from_lsb_to_hPa(int32_t lsb)
{
  return ((float_t)lsb / 1048576.0f);   /* 4096.0f * 256 */
}

float_t LPS22DF::lps22df_from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb / 100.0f);
}


/**
  * @brief  Device "Who am I".[get]
  *
  * @param  val   ID values.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_id_get(lps22df_id_t *val)
{
  uint8_t reg;
  int32_t ret;

  ret = readRegister(LPS22DF_WHO_AM_I, &reg, 1);
  val->whoami = reg;

  return ret;
}

/**
  * @brief  Configures the bus operating mode.[set]
  *
  * @param  val   configures the bus operating mode.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_bus_mode_set(lps22df_bus_mode_t *val)
{
  lps22df_i3c_if_ctrl_add_t i3c_if_ctrl_add;
  lps22df_if_ctrl_t if_ctrl;
  int32_t ret;

  ret = readRegister(LPS22DF_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  if (ret == 0) {
    if_ctrl.int_en_i3c = ((uint8_t)val->interface & 0x04U) >> 2;
    if_ctrl.i2c_i3c_dis = ((uint8_t)val->interface & 0x02U) >> 1;
    if_ctrl.sim = ((uint8_t)val->interface & 0x01U);
    ret = writeRegister(LPS22DF_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  }
  if (ret == 0) {
    ret = readRegister(LPS22DF_I3C_IF_CTRL_ADD,
                           (uint8_t *)&i3c_if_ctrl_add, 1);
  }
  if (ret == 0) {
    i3c_if_ctrl_add.asf_on = (uint8_t)val->filter & 0x01U;
    i3c_if_ctrl_add.i3c_bus_avb_sel = (uint8_t)val->i3c_ibi_time & 0x03U;
    ret = writeRegister(LPS22DF_I3C_IF_CTRL_ADD,
                            (uint8_t *)&i3c_if_ctrl_add, 1);
  }
  return ret;
}

/**
  * @brief  Configures the bus operating mode.[set]
  *
  * @param  val   configures the bus operating mode.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_bus_mode_get(lps22df_bus_mode_t *val)
{
  lps22df_i3c_if_ctrl_add_t i3c_if_ctrl_add;
  lps22df_if_ctrl_t if_ctrl;
  int32_t ret;

  ret = readRegister(LPS22DF_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  if (ret == 0) {
    ret = readRegister(LPS22DF_I3C_IF_CTRL_ADD,
                           (uint8_t *)&i3c_if_ctrl_add, 1);

    switch ((if_ctrl.int_en_i3c << 2) | (if_ctrl.i2c_i3c_dis << 1) |
            if_ctrl.sim) {
      case lps22df_bus_mode_t::LPS22DF_SEL_BY_HW:
        val->interface = lps22df_bus_mode_t::LPS22DF_SEL_BY_HW;
        break;
      case lps22df_bus_mode_t::LPS22DF_SPI_3W:
        val->interface = lps22df_bus_mode_t::LPS22DF_SPI_3W;
        break;
      case lps22df_bus_mode_t::LPS22DF_SPI_4W:
        val->interface = lps22df_bus_mode_t::LPS22DF_SPI_4W;
        break;
      case lps22df_bus_mode_t::LPS22DF_INT_PIN_ON_I3C:
        val->interface = lps22df_bus_mode_t::LPS22DF_INT_PIN_ON_I3C;
        break;
      default:
        val->interface = lps22df_bus_mode_t::LPS22DF_SEL_BY_HW;
        break;
    }

    switch (i3c_if_ctrl_add.asf_on) {
      case lps22df_bus_mode_t::LPS22DF_AUTO:
        val->filter = lps22df_bus_mode_t::LPS22DF_AUTO;
        break;
      case lps22df_bus_mode_t::LPS22DF_ALWAYS_ON:
        val->filter = lps22df_bus_mode_t::LPS22DF_ALWAYS_ON;
        break;
      default:
        val->filter = lps22df_bus_mode_t::LPS22DF_AUTO;
        break;
    }

    switch (i3c_if_ctrl_add.i3c_bus_avb_sel) {
      case lps22df_bus_mode_t::LPS22DF_IBI_50us:
        val->i3c_ibi_time = lps22df_bus_mode_t::LPS22DF_IBI_50us;
        break;
      case lps22df_bus_mode_t::LPS22DF_IBI_2us:
        val->i3c_ibi_time = lps22df_bus_mode_t::LPS22DF_IBI_2us;
        break;
      case lps22df_bus_mode_t::LPS22DF_IBI_1ms:
        val->i3c_ibi_time = lps22df_bus_mode_t::LPS22DF_IBI_1ms;
        break;
      case lps22df_bus_mode_t::LPS22DF_IBI_25ms:
        val->i3c_ibi_time = lps22df_bus_mode_t::LPS22DF_IBI_25ms;
        break;
      default:
        val->i3c_ibi_time = lps22df_bus_mode_t::LPS22DF_IBI_50us;
        break;
    }
  }
  return ret;
}

/**
  * @brief  Configures the bus operating mode.[get]
  *
  * @param  val   configures the bus operating mode.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_init_set(lps22df_init_t val)
{
  lps22df_ctrl_reg2_t ctrl_reg2;
  lps22df_ctrl_reg3_t ctrl_reg3;
  uint8_t reg[2];
  int32_t ret;

  ret = readRegister(LPS22DF_CTRL_REG2, reg, 2);
  if (ret == 0) {
    bytecpy((uint8_t *)&ctrl_reg2, &reg[0]);
    bytecpy((uint8_t *)&ctrl_reg3, &reg[1]);

    switch (val) {
      case LPS22DF_BOOT:
        ctrl_reg2.boot = PROPERTY_ENABLE;
        ret = writeRegister(LPS22DF_CTRL_REG2,
                                (uint8_t *)&ctrl_reg2, 1);
        break;
      case LPS22DF_RESET:
        ctrl_reg2.swreset = PROPERTY_ENABLE;
        ret = writeRegister(LPS22DF_CTRL_REG2,
                                (uint8_t *)&ctrl_reg2, 1);
        break;
      case LPS22DF_DRV_RDY:
        ctrl_reg2.bdu = PROPERTY_ENABLE;
        ctrl_reg3.if_add_inc = PROPERTY_ENABLE;
        bytecpy(&reg[0], (uint8_t *)&ctrl_reg2);
        bytecpy(&reg[1], (uint8_t *)&ctrl_reg3);
        ret = writeRegister(LPS22DF_CTRL_REG2, reg, 2);
        break;
      default:
        ctrl_reg2.swreset = PROPERTY_ENABLE;
        ret = writeRegister(LPS22DF_CTRL_REG2,
                                (uint8_t *)&ctrl_reg2, 1);
        break;
    }
  }
  return ret;
}

/**
  * @brief  Get the status of the device.[get]
  *
  * @param  val   the status of the device.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_status_get(lps22df_stat_t *val)
{
  lps22df_interrupt_cfg_t interrupt_cfg;
  lps22df_int_source_t int_source;
  lps22df_ctrl_reg2_t ctrl_reg2;
  lps22df_status_t status;
  int32_t ret;

  ret = readRegister(LPS22DF_CTRL_REG2,
                         (uint8_t *)&ctrl_reg2, 1);
  if (ret == 0) {
    ret = readRegister(LPS22DF_INT_SOURCE, (uint8_t *)&int_source, 1);
  }
  if (ret == 0) {
    ret = readRegister(LPS22DF_STATUS, (uint8_t *)&status, 1);
  }
  if (ret == 0) {
    ret = readRegister(LPS22DF_INTERRUPT_CFG,
                           (uint8_t *)&interrupt_cfg, 1);
  }
  val->sw_reset  = ctrl_reg2.swreset;
  val->boot      = int_source.boot_on;
  val->drdy_pres = status.p_da;
  val->drdy_temp = status.t_da;
  val->ovr_pres  = status.p_or;
  val->ovr_temp  = status.t_or;
  val->end_meas  = ~ctrl_reg2.oneshot;
  val->ref_done = ~interrupt_cfg.autozero;

  return ret;
}

/**
  * @brief  Electrical pin configuration.[set]
  *
  * @param  val   the electrical settings for the configurable pins.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_pin_conf_set(lps22df_pin_conf_t *val)
{
  lps22df_ctrl_reg3_t ctrl_reg3;
  lps22df_if_ctrl_t if_ctrl;
  int32_t ret;

  ret = readRegister(LPS22DF_IF_CTRL, (uint8_t *)&if_ctrl, 1);

  if (ret == 0) {
    if_ctrl.int_pd_dis = ~val->int_pull_down;
    if_ctrl.sda_pu_en = val->sda_pull_up;
    if_ctrl.sdo_pu_en = val->sdo_pull_up;
    ret = writeRegister(LPS22DF_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  }
  if (ret == 0) {
    ret = readRegister(LPS22DF_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
  }
  if (ret == 0) {
    ctrl_reg3.pp_od = ~val->int_push_pull;
    ret = writeRegister(LPS22DF_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
  }

  return ret;
}

/**
  * @brief  Electrical pin configuration.[get]
  *
  * @param  val   the electrical settings for the configurable pins.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_pin_conf_get(lps22df_pin_conf_t *val)
{
  lps22df_ctrl_reg3_t ctrl_reg3;
  lps22df_if_ctrl_t if_ctrl;
  int32_t ret;

  ret = readRegister(LPS22DF_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  if (ret == 0) {
    ret = readRegister(LPS22DF_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
  }

  val->int_pull_down = ~if_ctrl.int_pd_dis;
  val->sda_pull_up  = if_ctrl.sda_pu_en;
  val->sdo_pull_up  = if_ctrl.sdo_pu_en;
  val->int_push_pull  = ~ctrl_reg3.pp_od;

  return ret;
}

/**
  * @brief  Get the status of all the interrupt sources.[get]
  *
  * @param  val   the status of all the interrupt sources.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_all_sources_get(lps22df_all_sources_t *val)
{
  lps22df_fifo_status2_t fifo_status2;
  lps22df_int_source_t int_source;
  lps22df_status_t status;
  int32_t ret;

  ret = readRegister(LPS22DF_STATUS, (uint8_t *)&status, 1);
  if (ret == 0) {
    ret = readRegister(LPS22DF_INT_SOURCE,
                           (uint8_t *)&int_source, 1);
  }
  if (ret == 0) {
    ret = readRegister(LPS22DF_FIFO_STATUS2,
                           (uint8_t *)&fifo_status2, 1);
  }

  val->drdy_pres        = status.p_da;
  val->drdy_temp        = status.t_da;
  val->over_pres        = int_source.ph;
  val->under_pres       = int_source.pl;
  val->thrsld_pres      = int_source.ia;
  val->fifo_full        = fifo_status2.fifo_full_ia;
  val->fifo_ovr         = fifo_status2.fifo_ovr_ia;
  val->fifo_th          = fifo_status2.fifo_wtm_ia;

  return ret;
}


/**
  * @brief  Sensor conversion parameters selection.[set]
  *
  * @param  val   set the sensor conversion parameters.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_mode_set(lps22df_md_t *val)
{
  lps22df_ctrl_reg1_t ctrl_reg1;
  lps22df_ctrl_reg2_t ctrl_reg2;
  uint8_t reg[2];
  int32_t ret;

  ret = readRegister(LPS22DF_CTRL_REG1, reg, 2);

  if (ret == 0) {
    bytecpy((uint8_t *)&ctrl_reg1, &reg[0]);
    bytecpy((uint8_t *)&ctrl_reg2, &reg[1]);

    ctrl_reg1.odr = (uint8_t)val->odr;
    ctrl_reg1.avg = (uint8_t)val->avg;
    ctrl_reg2.en_lpfp = (uint8_t)val->lpf & 0x01U;
    ctrl_reg2.lfpf_cfg = ((uint8_t)val->lpf & 0x02U) >> 2;

    bytecpy(&reg[0], (uint8_t *)&ctrl_reg1);
    bytecpy(&reg[1], (uint8_t *)&ctrl_reg2);
    ret = writeRegister(LPS22DF_CTRL_REG1, reg, 2);
  }

  return ret;
}

/**
  * @brief  Sensor conversion parameters selection.[get]
  *
  * @param  val   get the sensor conversion parameters.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_mode_get(lps22df_md_t *val)
{
  lps22df_ctrl_reg1_t ctrl_reg1;
  lps22df_ctrl_reg2_t ctrl_reg2;
  uint8_t reg[2];
  int32_t ret;

  ret = readRegister(LPS22DF_CTRL_REG1, reg, 2);

  if (ret == 0) {
    bytecpy((uint8_t *)&ctrl_reg1, &reg[0]);
    bytecpy((uint8_t *)&ctrl_reg2, &reg[1]);

    switch (ctrl_reg1.odr) {
      case lps22df_md_t::LPS22DF_ONE_SHOT:
        val->odr = lps22df_md_t::LPS22DF_ONE_SHOT;
        break;
      case lps22df_md_t::LPS22DF_1Hz:
        val->odr = lps22df_md_t::LPS22DF_1Hz;
        break;
      case lps22df_md_t::LPS22DF_4Hz:
        val->odr = lps22df_md_t::LPS22DF_4Hz;
        break;
      case lps22df_md_t::LPS22DF_10Hz:
        val->odr = lps22df_md_t::LPS22DF_10Hz;
        break;
      case lps22df_md_t::LPS22DF_25Hz:
        val->odr = lps22df_md_t::LPS22DF_25Hz;
        break;
      case lps22df_md_t::LPS22DF_50Hz:
        val->odr = lps22df_md_t::LPS22DF_50Hz;
        break;
      case lps22df_md_t::LPS22DF_75Hz:
        val->odr = lps22df_md_t::LPS22DF_75Hz;
        break;
      case lps22df_md_t::LPS22DF_100Hz:
        val->odr = lps22df_md_t::LPS22DF_100Hz;
        break;
      case lps22df_md_t::LPS22DF_200Hz:
        val->odr = lps22df_md_t::LPS22DF_200Hz;
        break;
      default:
        val->odr = lps22df_md_t::LPS22DF_ONE_SHOT;
        break;
    }

    switch (ctrl_reg1.avg) {
      case lps22df_md_t::LPS22DF_4_AVG:
        val->avg = lps22df_md_t::LPS22DF_4_AVG;
        break;
      case lps22df_md_t::LPS22DF_8_AVG:
        val->avg = lps22df_md_t::LPS22DF_8_AVG;
        break;
      case lps22df_md_t::LPS22DF_16_AVG:
        val->avg = lps22df_md_t::LPS22DF_16_AVG;
        break;
      case lps22df_md_t::LPS22DF_32_AVG:
        val->avg = lps22df_md_t::LPS22DF_32_AVG;
        break;
      case lps22df_md_t::LPS22DF_64_AVG:
        val->avg = lps22df_md_t::LPS22DF_64_AVG;
        break;
      case lps22df_md_t::LPS22DF_128_AVG:
        val->avg = lps22df_md_t::LPS22DF_128_AVG;
        break;
      case lps22df_md_t::LPS22DF_256_AVG:
        val->avg = lps22df_md_t::LPS22DF_256_AVG;
        break;
      case lps22df_md_t::LPS22DF_512_AVG:
        val->avg = lps22df_md_t::LPS22DF_512_AVG;
        break;
      default:
        val->avg = lps22df_md_t::LPS22DF_4_AVG;
        break;
    }

    switch ((ctrl_reg2.lfpf_cfg << 2) | ctrl_reg2.en_lpfp) {
      case lps22df_md_t::LPS22DF_LPF_DISABLE:
        val->lpf = lps22df_md_t::LPS22DF_LPF_DISABLE;
        break;
      case lps22df_md_t::LPS22DF_LPF_ODR_DIV_4:
        val->lpf = lps22df_md_t::LPS22DF_LPF_ODR_DIV_4;
        break;
      case lps22df_md_t::LPS22DF_LPF_ODR_DIV_9:
        val->lpf = lps22df_md_t::LPS22DF_LPF_ODR_DIV_9;
        break;
      default:
        val->lpf = lps22df_md_t::LPS22DF_LPF_DISABLE;
        break;
    }
  }
  return ret;
}

/**
  * @brief  Software trigger for One-Shot.[get]
  *
  * @param  md    the sensor conversion parameters.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_trigger_sw(lps22df_md_t *md)
{
  lps22df_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = readRegister(LPS22DF_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
  if (md->odr == lps22df_md_t::LPS22DF_ONE_SHOT) {
    ctrl_reg2.oneshot = PROPERTY_ENABLE;
    if (ret == 0) {
      ret = writeRegister(LPS22DF_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
    }
  }
  return ret;
}

/**
  * @brief  Software trigger for One-Shot.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  data  data retrieved from the sensor.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_data_get(lps22df_data_t *data)
{
  uint8_t buff[5];
  int32_t ret;

  ret = readRegister(LPS22DF_PRESS_OUT_XL, buff, 5);

  /* pressure conversion */
  data->pressure.raw = (int32_t)buff[2];
  data->pressure.raw = (data->pressure.raw * 256) + (int32_t) buff[1];
  data->pressure.raw = (data->pressure.raw * 256) + (int32_t) buff[0];
  data->pressure.raw = data->pressure.raw * 256;

  data->pressure.hpa = lps22df_from_lsb_to_hPa(data->pressure.raw);


  /* temperature conversion */
  data->heat.raw = (int16_t)buff[4];
  data->heat.raw = (data->heat.raw * 256) + (int16_t) buff[3];
  data->heat.deg_c = lps22df_from_lsb_to_celsius(data->heat.raw);

  return ret;
}


/**
  * @brief  FIFO operation mode selection.[set]
  *
  * @param  val   set the FIFO operation mode.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_fifo_mode_set(lps22df_fifo_md_t *val)
{
  lps22df_fifo_ctrl_t fifo_ctrl;
  lps22df_fifo_wtm_t fifo_wtm;
  uint8_t reg[2];
  int32_t ret;

  ret = readRegister(LPS22DF_FIFO_CTRL, reg, 2);
  if (ret == 0) {
    bytecpy((uint8_t *)&fifo_ctrl, &reg[0]);
    bytecpy((uint8_t *)&fifo_wtm, &reg[1]);

    fifo_ctrl.f_mode = (uint8_t)val->operation & 0x03U;
    fifo_ctrl.trig_modes = ((uint8_t)val->operation & 0x04U) >> 2;

    if (val->watermark != 0x00U) {
      fifo_ctrl.stop_on_wtm = PROPERTY_ENABLE;
    } else {
      fifo_ctrl.stop_on_wtm = PROPERTY_DISABLE;
    }

    fifo_wtm.wtm = val->watermark;

    bytecpy(&reg[0], (uint8_t *)&fifo_ctrl);
    bytecpy(&reg[1], (uint8_t *)&fifo_wtm);

    ret = writeRegister(LPS22DF_FIFO_CTRL, reg, 2);
  }
  return ret;
}

/**
  * @brief  FIFO operation mode selection.[get]
  *
  * @param  val   get the FIFO operation mode.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_fifo_mode_get(lps22df_fifo_md_t *val)
{
  lps22df_fifo_ctrl_t fifo_ctrl;
  lps22df_fifo_wtm_t fifo_wtm;
  uint8_t reg[2];
  int32_t ret;

  ret = readRegister(LPS22DF_FIFO_CTRL, reg, 2);

  bytecpy((uint8_t *)&fifo_ctrl, &reg[0]);
  bytecpy((uint8_t *)&fifo_wtm, &reg[1]);

  switch ((fifo_ctrl.trig_modes << 2) | fifo_ctrl.f_mode) {
    case lps22df_fifo_md_t::LPS22DF_BYPASS:
      val->operation = lps22df_fifo_md_t::LPS22DF_BYPASS;
      break;
    case lps22df_fifo_md_t::LPS22DF_FIFO:
      val->operation = lps22df_fifo_md_t::LPS22DF_FIFO;
      break;
    case lps22df_fifo_md_t::LPS22DF_STREAM:
      val->operation = lps22df_fifo_md_t::LPS22DF_STREAM;
      break;
    case lps22df_fifo_md_t::LPS22DF_STREAM_TO_FIFO:
      val->operation = lps22df_fifo_md_t::LPS22DF_STREAM_TO_FIFO;
      break;
    case lps22df_fifo_md_t::LPS22DF_BYPASS_TO_STREAM:
      val->operation = lps22df_fifo_md_t::LPS22DF_BYPASS_TO_STREAM;
      break;
    case lps22df_fifo_md_t::LPS22DF_BYPASS_TO_FIFO:
      val->operation = lps22df_fifo_md_t::LPS22DF_BYPASS_TO_FIFO;
      break;
    default:
      val->operation = lps22df_fifo_md_t::LPS22DF_BYPASS;
      break;
  }

  val->watermark = fifo_wtm.wtm;

  return ret;
}

/**
  * @brief  Get the number of samples stored in FIFO.[get]
  *
  * @param  val   number of samples stored in FIFO.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_fifo_level_get(uint8_t *val)
{
  lps22df_fifo_status1_t fifo_status1;
  int32_t ret;

  ret = readRegister(LPS22DF_FIFO_STATUS1,
                         (uint8_t *)&fifo_status1, 1);

  *val = fifo_status1.fss;

  return ret;
}

/**
  * @brief  Software trigger for One-Shot.[get]
  *
  * @param  samp  number of samples stored in FIFO.(ptr)
  * @param  data  data retrieved from FIFO.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_fifo_data_get(uint8_t samp, lps22df_fifo_data_t *data)
{
  uint8_t fifo_data[3];
  uint8_t i;
  int32_t ret = 0;

  for (i = 0U; i < samp; i++) {
    ret = readRegister(LPS22DF_FIFO_DATA_OUT_PRESS_XL, fifo_data, 3);
    data[i].raw = (int32_t)fifo_data[2];
    data[i].raw = (data[i].raw * 256) + (int32_t)fifo_data[1];
    data[i].raw = (data[i].raw * 256) + (int32_t)fifo_data[0];
    data[i].raw = (data[i].raw * 256);
    data[i].hpa = lps22df_from_lsb_to_hPa(data[i].raw);
  }

  return ret;
}


/**
  * @brief  Interrupt pins hardware signal configuration.[set]
  *
  * @param  val   the pins hardware signal settings.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_interrupt_mode_set(lps22df_int_mode_t *val)
{
  lps22df_interrupt_cfg_t interrupt_cfg;
  lps22df_ctrl_reg3_t ctrl_reg3;
  lps22df_ctrl_reg4_t ctrl_reg4;
  uint8_t reg[2];
  int32_t ret;

  ret = readRegister(LPS22DF_CTRL_REG3, reg, 2);
  if (ret == 0) {
    bytecpy((uint8_t *)&ctrl_reg3, &reg[0]);
    bytecpy((uint8_t *)&ctrl_reg4, &reg[1]);

    ctrl_reg3.int_h_l = val->active_low;
    ctrl_reg4.drdy_pls = ~val->drdy_latched;

    bytecpy(&reg[0], (uint8_t *)&ctrl_reg3);
    bytecpy(&reg[1], (uint8_t *)&ctrl_reg4);

    ret = writeRegister(LPS22DF_CTRL_REG3, reg, 2);
  }
  if (ret == 0) {
    ret = readRegister(LPS22DF_INTERRUPT_CFG,
                           (uint8_t *)&interrupt_cfg, 1);
  }
  if (ret == 0) {
    interrupt_cfg.lir = val->int_latched ;
    ret = writeRegister(LPS22DF_INTERRUPT_CFG,
                            (uint8_t *)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Interrupt pins hardware signal configuration.[get]
  *
  * @param  val   the pins hardware signal settings.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_interrupt_mode_get(lps22df_int_mode_t *val)
{
  lps22df_interrupt_cfg_t interrupt_cfg;
  lps22df_ctrl_reg3_t ctrl_reg3;
  lps22df_ctrl_reg4_t ctrl_reg4;
  uint8_t reg[2];
  int32_t ret;

  ret = readRegister(LPS22DF_CTRL_REG3, reg, 2);
  if (ret == 0) {
    ret = readRegister(LPS22DF_INTERRUPT_CFG,
                           (uint8_t *)&interrupt_cfg, 1);
  }

  bytecpy((uint8_t *)&ctrl_reg3, &reg[0]);
  bytecpy((uint8_t *)&ctrl_reg4, &reg[1]);

  val->active_low = ctrl_reg3.int_h_l;
  val->drdy_latched = ~ctrl_reg4.drdy_pls;
  val->int_latched = interrupt_cfg.lir;

  return ret;
}

/**
  * @brief  Route interrupt signals on int1 pin.[set]
  *
  * @param  val   the signals to route on int1 pin.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_pin_int_route_set(lps22df_pin_int_route_t *val)
{
  lps22df_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = readRegister(LPS22DF_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  if (ret == 0) {
    ctrl_reg4.drdy = val->drdy_pres;
    ctrl_reg4.int_f_wtm = val->fifo_th;
    ctrl_reg4.int_f_ovr = val->fifo_ovr;
    ctrl_reg4.int_f_full = val->fifo_full;

    ret = writeRegister(LPS22DF_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
  }
  return ret;
}

/**
  * @brief  Route interrupt signals on int1 pin.[get]
  *
  * @param  val   the signals that are routed on int1 pin.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_pin_int_route_get(lps22df_pin_int_route_t *val)
{
  lps22df_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = readRegister(LPS22DF_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);

  val->drdy_pres =  ctrl_reg4.drdy;
  val->fifo_th = ctrl_reg4.int_f_wtm;
  val->fifo_ovr = ctrl_reg4.int_f_ovr;
  val->fifo_full = ctrl_reg4.int_f_full;

  return ret;

}


/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_int_on_threshold_mode_set(lps22df_int_th_md_t *val)
{
  lps22df_ctrl_reg4_t ctrl_reg4;
  lps22df_interrupt_cfg_t interrupt_cfg;
  lps22df_ths_p_l_t ths_p_l;
  lps22df_ths_p_h_t ths_p_h;
  uint8_t reg[3];
  int32_t ret;

  ret = readRegister(LPS22DF_INTERRUPT_CFG, reg, 3);
  if (ret == 0) {
    bytecpy((uint8_t *)&interrupt_cfg, &reg[0]);
    bytecpy((uint8_t *)&ths_p_l, &reg[1]);
    bytecpy((uint8_t *)&ths_p_h, &reg[2]);

    interrupt_cfg.phe = val->over_th;
    interrupt_cfg.ple = val->under_th;
    ths_p_h.ths = (uint8_t)(val->threshold / 256U);
    ths_p_l.ths = (uint8_t)(val->threshold - (ths_p_h.ths * 256U));

    bytecpy(&reg[0], (uint8_t *)&interrupt_cfg);
    bytecpy(&reg[1], (uint8_t *)&ths_p_l);
    bytecpy(&reg[2], (uint8_t *)&ths_p_h);

    ret = writeRegister(LPS22DF_INTERRUPT_CFG, (uint8_t *)&reg, 3);
  }

  if (ret == 0) {
    ret = readRegister(LPS22DF_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
    if (ret == 0) {
      ctrl_reg4.int_en = PROPERTY_ENABLE;
      ret = writeRegister(LPS22DF_CTRL_REG4,(uint8_t *)&ctrl_reg4, 1); 
    }
  }
  return ret;
}

/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_int_on_threshold_mode_get(lps22df_int_th_md_t *val)
{
  lps22df_interrupt_cfg_t interrupt_cfg;
  lps22df_ths_p_l_t ths_p_l;
  lps22df_ths_p_h_t ths_p_h;
  uint8_t reg[3];
  int32_t ret;

  ret = readRegister(LPS22DF_INTERRUPT_CFG, reg, 3);

  bytecpy((uint8_t *)&interrupt_cfg, &reg[0]);
  bytecpy((uint8_t *)&ths_p_l, &reg[1]);
  bytecpy((uint8_t *)&ths_p_h, &reg[2]);

  val->over_th = interrupt_cfg.phe;
  val->under_th = interrupt_cfg.ple;
  val->threshold = ths_p_h.ths;
  val->threshold = (val->threshold * 256U)  + ths_p_l.ths;

  return ret;
}

/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_reference_mode_set(lps22df_ref_md_t *val)
{
  lps22df_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = readRegister(LPS22DF_INTERRUPT_CFG,
                         (uint8_t *)&interrupt_cfg, 1);
  if (ret == 0) {

    interrupt_cfg.autozero = val->get_ref;
    interrupt_cfg.autorefp = (uint8_t)val->apply_ref & 0x01U;

    interrupt_cfg.reset_az  = ((uint8_t)val->apply_ref & 0x02U) >> 1;
    interrupt_cfg.reset_arp = ((uint8_t)val->apply_ref & 0x02U) >> 1;

    ret = readRegister(LPS22DF_INTERRUPT_CFG,
                           (uint8_t *)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_reference_mode_get(lps22df_ref_md_t *val)
{
  lps22df_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = readRegister(LPS22DF_INTERRUPT_CFG,
                         (uint8_t *)&interrupt_cfg, 1);

  switch ((interrupt_cfg.reset_az << 1) |
          interrupt_cfg.autorefp) {
    case lps22df_ref_md_t::LPS22DF_OUT_AND_INTERRUPT:
      val->apply_ref = lps22df_ref_md_t::LPS22DF_OUT_AND_INTERRUPT;
      break;
    case lps22df_ref_md_t::LPS22DF_ONLY_INTERRUPT:
      val->apply_ref = lps22df_ref_md_t::LPS22DF_ONLY_INTERRUPT;
      break;
    default:
      val->apply_ref = lps22df_ref_md_t::LPS22DF_RST_REFS;
      break;
  }
  val->get_ref = interrupt_cfg.autozero;

  return ret;
}


/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_opc_set(int16_t val)
{
  uint8_t reg[2];
  int32_t ret;

  reg[1] = (uint8_t)(((uint16_t)val & 0xFF00U) / 256U);
  reg[0] = (uint8_t)((uint16_t)val & 0x00FFU);

  ret = writeRegister(LPS22DF_RPDS_L, reg, 2);

  return ret;
}

/**
  * @brief  Configuration of Wake-up and Wake-up to Sleep .[set]
  *
  * @param  val   parameters of configuration.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t LPS22DF::lps22df_opc_get(int16_t *val)
{
  uint8_t reg[2];
  int32_t ret;

  ret = readRegister(LPS22DF_RPDS_L, reg, 2);

  *val = (int16_t)reg[1];
  *val = *val * 256 + (int16_t)reg[0];

  return ret;
}
