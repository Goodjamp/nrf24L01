/********************************************************************************
  * @file    i2c_user_interface.h
  * @author  Gerasimchuk A.
  * @version V1.0.0
  * @date    27-October-2017
  * @brief
  */

#ifndef I2C_USER_INTERFACE_H_
#define I2C_USER_INTERFACE_H_

typedef enum{
	I2C_1 = 0,
	I2C_2 = 1,
}I2C_DEF;

typedef enum{
	I2C_STATUS_NOT_CONFIG,
	I2C_STATUS_OK,
	I2C_STATUS_TRANSACTION_PROCESSING,
	I2C_STATUS_TRANSACTION_ERROR,
}I2C_STATUS;

typedef struct{
	uint32_t frequencyI2C;
}I2C_configDef;


I2C_STATUS i2cConfig( I2C_DEF selI2C ,I2C_configDef *config);
I2C_STATUS i2cGetStatus(I2C_DEF I2C_SEL);
I2C_STATUS i2cTxData(I2C_DEF i2c, uint8_t address_dev, uint8_t address_reg, uint8_t num_read, uint8_t *buff);
I2C_STATUS i2cRxData(I2C_DEF i2c, uint8_t address_dev, uint8_t address_reg, uint8_t num_read, uint8_t *buff);

//user implement function
void     i2cInitGpio(uint8_t step);
uint32_t i2cgetTimeMs(void);
void     i2cRecover(uint32_t i2cFRQ);
#endif
