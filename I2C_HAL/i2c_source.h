/**
  ******************************************************************************
  * @file    i2c_sourse.h
  * @author  Gerasimchuk A.
  * @version V1.0.0
  * @date    30-July-2017
  * @brief
  */
#ifndef I2C_SOURCE_H_
#define I2C_SOURCE_H_

#include "stdint.h"

#include "stm32f10x.h"

#include "i2c_user_interface.h"

typedef enum{
	I2C_TRANSACTION_TX,
	I2C_TRANSACTION_RX,
}I2C_TRANSACTION_TYPE;


// ordering status for transmit data
typedef enum{
	I2C_TX_SB      = 0,
	I2C_TX_ADDRESS = 1,
	I2C_TX_TXE     = 2,
	I2C_TX_BTF     = 3
}I2C_TX_STATE;

// ordering status of receive data
typedef enum{
	I2C_RX_SB_F1      = 0,
	I2C_RX_ADDRESS_F1 = 1,
	I2C_RX_BTF_F1     = 2,
	I2C_RX_SB_F2      = 3,
	I2C_RX_ADDRESS_F2 = 4,
	I2C_RX_RXE_F2     = 5,
	I2C_RX_BTF_F2     = 6,
	I2C_RX_RXE_F3     = 7,
	I2C_RX_BTF_F3     = 8,
}I2C_RX_STATE;


typedef struct{
	I2C_TypeDef*         I2C_SEL;           // I2C pointer
	uint32_t             frq;               // frequency of SCL bus
	I2C_STATUS           transactionStatus; // transaction status
	I2C_TRANSACTION_TYPE transactionType;   // type of transaction (Tx/Rx)
	uint32_t             I2C_periodUs;      //
	uint8_t              stateCnt;          // counter state of I2C transaction, point on current state (tx/rx - I2C_TX_STATE/I2C_RX_STATE)
	uint8_t              addressDev;        // slave address
	uint8_t              addressReg;        // start register address for Tx/Rx data
	uint8_t              numData;           // number of Tx/Rx data
	uint8_t              *buffData;         // buffer for Tx/Rx data
	uint16_t             cnt;               // counter Tx/Rx data
}I2CProcessingDef;


#endif /* HP03SA_SOURCE_H_ */
