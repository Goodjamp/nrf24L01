
#ifndef PROCESSING_FEQMETTER_H_
#define PROCESSING_FEQMETTER_H_

#define I2C_SENSOR        I2C_2
#define I2C_SENSOR_FRQ_HZ 20000

#define I2C1_SCL            GPIO_Pin_10       //PB10 ch1
#define I2C1_SCL_AF_GPIO    GPIO_PinSource10  //PB8 ch1
#define I2C1_SCL_PORT       GPIOB            // CSCL PORT

#define I2C1_SDA            GPIO_Pin_11       //PB7 ch1
#define I2C1_SDA_AF_GPIO    GPIO_PinSource11  //PB7 ch1
#define I2C1_SDA_PORT       GPIOB            // CSDA PORT

// Transform atmospheric pressure from Pascal to mmHg
#define PASCAL_TO_MMHG_COEF 0.00750062

#endif // PROCESSING_TC_SINAL
