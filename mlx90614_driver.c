//******************************************************************************

//    _____ _               _____                        _     _
//   |_   _| |             |  __ \                      | |   (_)
//     | | | |_   _  __ _  | |  | | ___ _ __ _   _  __ _| |__  _ _ __
//     | | | | | | |/ _` | | |  | |/ _ \ '__| | | |/ _` | '_ \| | '_ \
//    _| |_| | |_| | (_| | | |__| |  __/ |  | |_| | (_| | |_) | | | | |
//   |_____|_|\__, |\__,_| |_____/ \___|_|   \__, |\__,_|_.__/|_|_| |_|
//             __/ |                          __/ |
//            |___/                          |___/

//******************************************************************************

#include "mlx90614_driver.h"

#define MEASURE_CYCLES 10

//****************************************************************************
//
//! \brief Read from the MLX90614 with defaults
//!         Reads the data
//!
//! \param[in]    i2cHandle        the handle to the openned i2c device
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int16_t mlx90614GetTemp(I2C_HandleTypeDef * i2cHandle, uint8_t data_addr, uint16_t * data)
{
    char ucRegVal[2];
   HAL_StatusTypeDef i2c_status;

    /* Read the temp */
    i2c_status = HAL_I2C_Mem_Read(i2cHandle, MLX90614_ADDR, 
                                  data_addr, I2C_MEMADD_SIZE_8BIT, 
                                 (uint8_t *) &ucRegVal, 2, 
                                 100);
     if(i2c_status == HAL_OK)
    {
        *data = (ucRegVal[1] << 8 | ucRegVal[0]);
        return HAL_OK;
    }


    return HAL_ERROR;
}


//****************************************************************************
//
//! \brief Read from the MLX90614 with defaults
//!         Reads the object temperature.
//!
//! \param[in]    i2cHandle        the handle to the openned i2c device
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int16_t mlx90614GetObjectTemp(I2C_HandleTypeDef * i2cHandle, float * data)
{
  uint16_t temp_read;
  uint16_t max_temp = 0;
  int i;
  
  for (i = 0; i < MEASURE_CYCLES; i++)
  {
    if (HAL_OK ==  mlx90614GetTemp(i2cHandle, MLX90614_OBJECT_THEMP, &temp_read))
    {
      if (!max_temp) max_temp = temp_read;
      else
      {
        if (max_temp < temp_read) max_temp = temp_read;
      }
    }
    else     
      return HAL_ERROR;
    HAL_Delay(50);
  }
  *data = max_temp * 0.02 - 273.15;
  return HAL_OK;
}


//****************************************************************************
//
//! \brief Read from the MLX90614 with defaults
//!         Reads the ambint temperature.
//!
//! \param[in]    i2cHandle        the handle to the openned i2c device
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int16_t mlx90614GetEnviromentTemp(I2C_HandleTypeDef * i2cHandle, float * data)
{
  uint16_t temp_read;
  if (HAL_OK ==  mlx90614GetTemp(i2cHandle, MLX90614_AMBIENT_THEMP, &temp_read))
  {
    *data = temp_read * 0.02 - 273.15;
    return HAL_OK;
  }
  return HAL_ERROR;
}

//****************************************************************************
//
//! \brief Read from the MLX90614 with defaults
//!         Reads the CHIP ID.
//!   Implementation of I2C read for 16-bit values */
//!
//! \param[in]    i2cHandle        the handle to the openned i2c device
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int32_t mlx90632_i2c_read(I2C_HandleTypeDef * i2cHandle, int16_t register_address, uint16_t *value)
{
	uint8_t data[2];
	int32_t ret;
	ret = HAL_I2C_Mem_Read(i2cHandle, MLX90614_ADDR, register_address, 2, data, sizeof(data), 100);
	*value = data[1]|(data[0]<<8);
	return ret;
}


//****************************************************************************
//
//! \brief Read from the MLX90614 with defaults
//!         Reads the CHIP ID.
//!   Implementation of I2C read for 32-bit values */
//!
//! \param[in]    i2cHandle        the handle to the openned i2c device
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int32_t mlx90632_i2c_read32(I2C_HandleTypeDef * i2cHandle, int16_t register_address, uint32_t *value)
{
	uint8_t data[4];
	int32_t ret;
	ret = HAL_I2C_Mem_Read(i2cHandle, MLX90614_ADDR, register_address, 2, data, sizeof(data), 100);
	//Endianness
	*value = data[2]<<24|data[3]<<16|data[0]<<8|data[1];
	return ret;
}


//****************************************************************************
//
//! \brief Write from the MLX90614 with defaults
//!         Write the CHIP ID.
//!   Implementation of I2C read for 16-bit values */
//!
//! \param[in]    i2cHandle        the handle to the openned i2c device
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int32_t mlx90632_i2c_write(I2C_HandleTypeDef * i2cHandle, int16_t register_address, uint16_t value) 
{
	uint8_t data[2];
	data[0] = value >> 8;
	data[1] = value;
	return HAL_I2C_Mem_Write(i2cHandle, MLX90614_ADDR, register_address, 2, data, 2, 100);
}
