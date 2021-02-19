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
#include "stm32l0xx_hal.h"

#ifndef MLX90614_DRIVER_H_
#define MLX90614_DRIVER_H_


/* Private defines -----------------------------------------------------------*/


//*****************************************************************************
// mlx90614 Device I2C address
//*****************************************************************************
#define MLX90614_ADDR        (0x5A << 1)
#define MLX90614_OBJECT_THEMP       0x07
#define MLX90614_AMBIENT_THEMP       0x06

/* OPCODE DEFINES */
#define MLX90614_OP_RAM		0x00
#define MLX90614_OP_EEPROM	0x20
#define MLX90614_OP_SLEEP	0xFF

/* RAM offsets with 16-bit data, MSB first */
#define MLX90614_RAW1		(MLX90614_OP_RAM | 0x04) /* raw data IR channel 1 */
#define MLX90614_RAW2		(MLX90614_OP_RAM | 0x05) /* raw data IR channel 2 */
#define MLX90614_TAMB 		(MLX90614_OP_RAM | 0x06) /* ambient temperature */
#define MLX90614_TOBJ1 		(MLX90614_OP_RAM | 0x07) /* object 1 temperature */
#define MLX90614_TOBJ2 		(MLX90614_OP_RAM | 0x08) /* object 2 temperature */
/* EEPROM offsets with 16-bit data, MSB first */
#define MLX90614_TOMIN 		(MLX90614_OP_EEPROM | 0x00) /* object temperature min register */
#define MLX90614_TOMAX 		(MLX90614_OP_EEPROM | 0x01) /* object temperature max register */
#define MLX90614_PWMCTRL 	(MLX90614_OP_EEPROM | 0x02) /* pwm configuration register */
#define MLX90614_TARANGE 	(MLX90614_OP_EEPROM | 0x03) /* ambient temperature register */
#define MLX90614_EMISSIVITY (MLX90614_OP_EEPROM | 0x04) /* emissivity correction register */
#define MLX90614_CFG1 		(MLX90614_OP_EEPROM | 0x05) /* configuration register */
#define MLX90614_SA 		(MLX90614_OP_EEPROM | 0x0E) /* slave address register */
#define MLX90614_ID1 		(MLX90614_OP_EEPROM | 0x1C) /*[read-only] 1 ID register */
#define MLX90614_ID2 		(MLX90614_OP_EEPROM | 0x1D) /*[read-only] 2 ID register */
#define MLX90614_ID3 		(MLX90614_OP_EEPROM | 0x1E) /*[read-only] 3 ID register */
#define MLX90614_ID4 		(MLX90614_OP_EEPROM | 0x1F) /*[read-only] 4 ID register */

#define MLX90614_DBG_OFF 0
#define MLX90614_DBG_ON 1
#define MLX90614_DBG_MSG_W 0
#define MLX90614_DBG_MSG_R 1


int16_t mlx90614GetTemp(I2C_HandleTypeDef * i2cHandle, uint8_t data_addr, uint16_t * data);
int16_t mlx90614GetObjectTemp(I2C_HandleTypeDef * i2cHandle, float * data);
int16_t mlx90614GetEnviromentTemp(I2C_HandleTypeDef * i2cHandle, float * data);
int32_t mlx90632_i2c_write(I2C_HandleTypeDef * i2cHandle, int16_t register_address, uint16_t value);
int32_t mlx90632_i2c_read32(I2C_HandleTypeDef * i2cHandle, int16_t register_address, uint32_t *value);
int32_t mlx90632_i2c_read(I2C_HandleTypeDef * i2cHandle, int16_t register_address, uint16_t *value);
#endif /* MLX90614_DRIVER_H_ */

