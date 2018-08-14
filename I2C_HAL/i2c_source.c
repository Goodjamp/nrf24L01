/**
  ******************************************************************************
  * @file    i2c_sourse.c
  * @author  Gerasimchuk A.
  * @version V1.0.0
  * @date    30-July-2017
  * @brief
  */

#include "stdint.h"
#include "stdbool.h"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"

#include "i2c_source.h"
#include "i2c_user_interface.h"


// ----------------static functions prototype------------------------------
static void startTransactionI2C(I2C_DEF  i2cIn, uint8_t address_dev, uint8_t address_reg, uint8_t num_read, uint8_t *buff, I2C_TRANSACTION_TYPE typeTransaction);
static inline void I2CProcessingInterruptRx(I2C_DEF  i2cIn);
static inline void I2CProcessingInterruptTx(I2C_DEF  i2cIn);

static void (*fI2CInterruptProcessing[2])(I2C_DEF  i2cIn) = {
		[I2C_TRANSACTION_RX] = I2CProcessingInterruptRx,
		[I2C_TRANSACTION_TX] = I2CProcessingInterruptTx
};
static volatile I2CProcessingDef I2CProcessing[2] = {
		[I2C_1] = {.I2C_SEL = I2C1,
				   .transactionStatus = I2C_STATUS_NOT_CONFIG},
		[I2C_2] = {.I2C_SEL = I2C2,
				   .transactionStatus = I2C_STATUS_NOT_CONFIG},
};


static void startInitI2C(I2C_TypeDef *selI2C){
	if( selI2C == I2C1 )
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		NVIC_EnableIRQ(I2C1_EV_IRQn);
	}
	if( selI2C == I2C2 ){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		NVIC_EnableIRQ(I2C2_EV_IRQn);
	}
}


static void startTransactionI2C(I2C_DEF  i2cIn, uint8_t address_dev, uint8_t address_reg,
		                          uint8_t num_read, uint8_t *buff, I2C_TRANSACTION_TYPE typeTransaction){
	I2CProcessing[i2cIn].transactionStatus = I2C_STATUS_TRANSACTION_PROCESSING;
	I2CProcessing[i2cIn].transactionType = typeTransaction;
	I2CProcessing[i2cIn].addressDev = address_dev;
	I2CProcessing[i2cIn].addressReg = address_reg;
	I2CProcessing[i2cIn].buffData = buff;
	I2CProcessing[i2cIn].numData = num_read;
	I2CProcessing[i2cIn].cnt = 0;
	// Start Tx data
	I2C_Cmd(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
	I2C_AcknowledgeConfig(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
	I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_EVT, ENABLE);
	I2C_GenerateSTART(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
}


static inline bool clearI2CComunication(I2C_DEF  i2cIn, uint16_t I2C_SR, uint16_t I2C_SR1Field ){

	if( I2C_SR & I2C_SR1Field )
	{
		return true;
	}

	// FULL STOP I2C
	I2CProcessing[i2cIn].transactionStatus = I2C_STATUS_TRANSACTION_ERROR;
	I2CProcessing[i2cIn].stateCnt = 0;
	I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_BUF, DISABLE);
	I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_EVT, DISABLE);
	//I2C_GenerateSTART(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
	I2C_GenerateSTOP(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
	I2C_Cmd(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
	if( I2CProcessing[i2cIn].I2C_SEL == I2C1 )
	{
		NVIC_DisableIRQ(I2C1_EV_IRQn);
	}
	if( I2CProcessing[i2cIn].I2C_SEL == I2C2 ){
		NVIC_DisableIRQ(I2C2_EV_IRQn);
	}
	return false;
}


static inline uint32_t getTimeTransaction(I2C_DEF i2cIn, uint16_t numData){
	return i2cgetTimeMs() + 10 + ( I2CProcessing[i2cIn].I2C_periodUs * (numData + 5))/10000;
}


I2C_STATUS i2cConfig( I2C_DEF i2cIn ,I2C_configDef *config){
	//----CONFIG I2C1--------------------------------
	I2C_InitTypeDef I2C_InitStruct;

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);

	//i2cInitGpio(0);
	i2cRecover(I2CProcessing[i2cIn].frq);

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

	startInitI2C(I2CProcessing[i2cIn].I2C_SEL);
	i2cInitGpio(1);

	I2CProcessing[i2cIn].I2C_periodUs = ( 10000000 * 9) / ( config->frequencyI2C );

	I2C_InitStruct.I2C_ClockSpeed = config->frequencyI2C;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			  // I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	  // 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			  // own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;		  // disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2CProcessing[i2cIn].I2C_SEL, &I2C_InitStruct);			   // init I2C

	I2CProcessing[i2cIn].transactionStatus = I2C_STATUS_OK;
	return I2C_STATUS_OK;
};


I2C_STATUS i2cGetStatus(I2C_DEF i2cIn){

	return I2CProcessing[i2cIn].transactionStatus;
}


I2C_STATUS i2cTxData(I2C_DEF i2cIn, uint8_t address_dev, uint8_t address_reg, uint8_t num_read, uint8_t *buff){
	uint32_t maxTimeTransaction = getTimeTransaction(i2cIn, num_read);

	if( (I2CProcessing[i2cIn].transactionStatus == I2C_STATUS_NOT_CONFIG) ||
		(I2CProcessing[i2cIn].transactionStatus == I2C_STATUS_TRANSACTION_PROCESSING)
		)
	{
		return I2CProcessing[i2cIn].transactionStatus;
	}
	startTransactionI2C(i2cIn, address_dev, address_reg, num_read, buff, I2C_TRANSACTION_TX);
	while( (I2CProcessing[i2cIn].transactionStatus == I2C_STATUS_TRANSACTION_PROCESSING) ||
		   (I2C_ReadRegister(I2CProcessing[i2cIn].I2C_SEL, I2C_Register_CR1) & I2C_CR1_STOP) )
	{
		// user time check function
		if( i2cgetTimeMs() >= maxTimeTransaction )
		{
			// timeout stop transaction
			I2CProcessing[i2cIn].transactionStatus = I2C_STATUS_TRANSACTION_ERROR;
			I2CProcessing[i2cIn].stateCnt = 0;
			break;
		}

	}
	// if error
	if(I2CProcessing[i2cIn].transactionStatus == I2C_STATUS_TRANSACTION_ERROR)
	{
		I2C_Cmd(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
		I2C_Cmd(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
	}
	I2C_GenerateSTOP(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
	I2C_Cmd(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
	return I2CProcessing[i2cIn].transactionStatus;
}


I2C_STATUS i2cRxData(I2C_DEF i2cIn, uint8_t address_dev, uint8_t address_reg, uint8_t num_read, uint8_t *buff){
	uint32_t maxTimeTransaction = getTimeTransaction(i2cIn, num_read);

	if( (I2CProcessing[i2cIn].transactionStatus == I2C_STATUS_NOT_CONFIG) ||
		(I2CProcessing[i2cIn].transactionStatus == I2C_STATUS_TRANSACTION_PROCESSING)
		)
	{
		return I2CProcessing[i2cIn].transactionStatus;
	}
	startTransactionI2C(i2cIn, address_dev, address_reg, num_read, buff, I2C_TRANSACTION_RX);
	while( I2CProcessing[i2cIn].transactionStatus == I2C_STATUS_TRANSACTION_PROCESSING )
	{
		// user time check function

		if( i2cgetTimeMs() >= maxTimeTransaction )
		{
			// timeout stop transaction
			I2CProcessing[i2cIn].transactionStatus = I2C_STATUS_TRANSACTION_ERROR;
			I2CProcessing[i2cIn].stateCnt = 0;
			break;
		}

	};
	// if error
	if(I2CProcessing[i2cIn].transactionStatus == I2C_STATUS_TRANSACTION_ERROR)
	{
		I2C_Cmd(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
		I2C_Cmd(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
	}
	I2C_GenerateSTOP(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
	I2C_Cmd(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
	return I2CProcessing[i2cIn].transactionStatus;
}


void I2C1_EV_IRQHandler(void){
	fI2CInterruptProcessing[I2CProcessing[I2C_1].transactionType](I2C_1);
}


void I2C2_EV_IRQHandler(void){
	fI2CInterruptProcessing[I2CProcessing[I2C_2].transactionType](I2C_2);
}


//Function: I2CProcessingInterrupRx
//Description:State machine for TRANSMITE data to I2C device. Called from I2C interrupt
static inline void I2CProcessingInterruptTx(I2C_DEF  i2cIn){

	uint16_t I2C_SR = I2C_ReadRegister(I2CProcessing[i2cIn].I2C_SEL, I2C_Register_SR1);

	if(I2C_STATUS_TRANSACTION_PROCESSING != I2CProcessing[i2cIn].transactionStatus )
	{
	    return;
	}
	switch(I2CProcessing[i2cIn].stateCnt){
	case I2C_TX_SB:
		// first interrupt should be completed SB generation
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_SB))
		{

			return;
		};
		//I2C_GenerateSTART(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
		I2C_Send7bitAddress(I2CProcessing[i2cIn].I2C_SEL, (uint8_t)(I2CProcessing[i2cIn].addressDev<<1), I2C_Direction_Transmitter);
		I2CProcessing[i2cIn].stateCnt = I2C_TX_ADDRESS;
		break;
	case I2C_TX_ADDRESS:
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_ADDR))
		{
			return;
		};
		I2C_ReadRegister(I2CProcessing[i2cIn].I2C_SEL, I2C_Register_SR2);
		// Enable All buffer interupts
		I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_BUF, ENABLE);
		// Send address register
		I2C_SendData(I2CProcessing[i2cIn].I2C_SEL, I2CProcessing[i2cIn].addressReg);
		I2CProcessing[i2cIn].stateCnt = I2C_TX_TXE;
		break;
	case I2C_TX_TXE:
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_TXE))
		{
			return;
		};
		// Send data
		if(I2CProcessing[i2cIn].cnt >= I2CProcessing[i2cIn].numData)
		{
			I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_BUF, DISABLE);
			I2CProcessing[i2cIn].stateCnt = I2C_TX_BTF;
			break;
		}
		I2C_SendData(I2CProcessing[i2cIn].I2C_SEL, I2CProcessing[i2cIn].buffData[I2CProcessing[i2cIn].cnt]);

		I2CProcessing[i2cIn].cnt++;
		break;
	case I2C_TX_BTF:
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_BTF))
		{
			return;
		};
		I2C_GenerateSTOP(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
		I2CProcessing[i2cIn].transactionStatus = I2C_STATUS_OK;
		I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_BUF, DISABLE);
		I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_EVT, DISABLE);
		I2CProcessing[i2cIn].stateCnt = 0;
		break;
	}
}


//Function: I2CProcessingInterrupRx
//Description:State machine for receive data from I2C device. Called from I2C interrupt
static inline void I2CProcessingInterruptRx(I2C_DEF  i2cIn){
	uint16_t I2C_SR = I2C_ReadRegister(I2CProcessing[i2cIn].I2C_SEL, I2C_Register_SR1);

	if( (I2C_STATUS_TRANSACTION_PROCESSING != I2CProcessing[i2cIn].transactionStatus ) || (I2C_SR == 0))
	{
	    return;
	}
	switch(I2CProcessing[i2cIn].stateCnt){
	case I2C_RX_SB_F1:
		//I2C_GenerateSTART(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
		// first interrupt should be completed SB generation
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_SB))
		{
			return;
		};
		I2C_Send7bitAddress(I2CProcessing[i2cIn].I2C_SEL, (uint8_t)(I2CProcessing[i2cIn].addressDev<<1), I2C_Direction_Transmitter);
		I2CProcessing[i2cIn].stateCnt = I2C_RX_ADDRESS_F1;
		break;
	case I2C_RX_ADDRESS_F1:
		//I2C_GenerateSTART(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_ADDR))
		{
			return;
		};
		I2C_ReadRegister(I2CProcessing[i2cIn].I2C_SEL, I2C_Register_SR2);
		// Send start address register
		I2C_SendData(I2CProcessing[i2cIn].I2C_SEL, I2CProcessing[i2cIn].addressReg);
		I2CProcessing[i2cIn].stateCnt = I2C_RX_BTF_F1;
		break;
	case I2C_RX_BTF_F1:
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_BTF))
		{
			return;
		};
		I2C_ReceiveData(I2CProcessing[i2cIn].I2C_SEL);
		I2C_GenerateSTART(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
		I2CProcessing[i2cIn].stateCnt = I2C_RX_SB_F2;
		break;
	case I2C_RX_SB_F2:
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_SB))
		{
			return;
		};
		//I2C_GenerateSTART(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
		I2C_Send7bitAddress(I2CProcessing[i2cIn].I2C_SEL, (uint8_t)(I2CProcessing[i2cIn].addressDev<<1), I2C_Direction_Receiver);
		I2CProcessing[i2cIn].stateCnt = I2C_RX_ADDRESS_F2;
		break;
	case I2C_RX_ADDRESS_F2:
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_ADDR))
		{
			return;
		};
		if(I2CProcessing[i2cIn].numData == 3) // -----------------------------for RX 3 bytes
		{
			// SR1 was read in clearI2CComunication function
			I2C_ReadRegister(I2CProcessing[i2cIn].I2C_SEL, I2C_Register_SR2);
			I2CProcessing[i2cIn].stateCnt = I2C_RX_BTF_F2;
			break;
		}
		else if(I2CProcessing[i2cIn].numData == 1) // -------------------------for RX 1 bytes
		{
			I2C_AcknowledgeConfig(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
			// SR1 was read in clearI2CComunication function
			I2C_ReadRegister(I2CProcessing[i2cIn].I2C_SEL, I2C_Register_SR2);
			I2C_GenerateSTOP(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
			// Enable buffer interrupt
			I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_BUF, ENABLE);
			I2CProcessing[i2cIn].stateCnt = I2C_RX_RXE_F3;
			break;
		}
		// SR1 was read in clearI2CComunication function
		I2C_ReadRegister(I2CProcessing[i2cIn].I2C_SEL, I2C_Register_SR2);
		// Enable buffer interrupt
		I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_BUF, ENABLE);
		I2CProcessing[i2cIn].stateCnt = I2C_RX_RXE_F2;
		break;
	case I2C_RX_RXE_F2:
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_RXNE))
		{
			return;
		};
		I2CProcessing[i2cIn].buffData[I2CProcessing[i2cIn].cnt] = I2C_ReceiveData(I2CProcessing[i2cIn].I2C_SEL);
		I2CProcessing[i2cIn].cnt++;
		if (I2CProcessing[i2cIn].numData == 2)
		{
			I2C_GenerateSTOP(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
			I2C_AcknowledgeConfig(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
			I2CProcessing[i2cIn].stateCnt = I2C_RX_RXE_F3;
			break;
		}
		if( (I2CProcessing[i2cIn].numData - I2CProcessing[i2cIn].cnt) <= 3 )
		{
			I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_BUF, DISABLE);
			I2CProcessing[i2cIn].stateCnt = I2C_RX_BTF_F2;
		}
		break;
	case I2C_RX_BTF_F2: // in RX mode, I2C_RX_BTF flag set in case of DR not empty and shift reg not empty
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_BTF))
		{
			return;
		};
		I2C_AcknowledgeConfig(I2CProcessing[i2cIn].I2C_SEL, DISABLE);
		// Receive data from DR register and shift register
		I2CProcessing[i2cIn].buffData[I2CProcessing[i2cIn].cnt++] = I2C_ReceiveData(I2CProcessing[i2cIn].I2C_SEL);
		I2C_GenerateSTOP(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
		I2CProcessing[i2cIn].buffData[I2CProcessing[i2cIn].cnt++] = I2C_ReceiveData(I2CProcessing[i2cIn].I2C_SEL);
		I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_BUF, ENABLE);
		I2CProcessing[i2cIn].stateCnt = I2C_RX_RXE_F3;
		break;
	case I2C_RX_RXE_F3:
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_RXNE))
		{
			return;
		};
		I2CProcessing[i2cIn].buffData[I2CProcessing[i2cIn].cnt] = I2C_ReceiveData(I2CProcessing[i2cIn].I2C_SEL);
		I2CProcessing[i2cIn].transactionStatus = I2C_STATUS_OK;
		I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_BUF, DISABLE);
		I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_EVT, DISABLE);
		I2CProcessing[i2cIn].stateCnt = 0;
		break;
	case I2C_RX_BTF_F3:
		if( !clearI2CComunication(i2cIn, I2C_SR, I2C_SR1_BTF))
		{
			return;
		};
		I2C_GenerateSTOP(I2CProcessing[i2cIn].I2C_SEL, ENABLE);
		// Receive data from DR register and shift register
		I2CProcessing[i2cIn].buffData[I2CProcessing[i2cIn].cnt++] = I2C_ReceiveData(I2CProcessing[i2cIn].I2C_SEL);
		I2CProcessing[i2cIn].buffData[I2CProcessing[i2cIn].cnt++] = I2C_ReceiveData(I2CProcessing[i2cIn].I2C_SEL);
		I2CProcessing[i2cIn].transactionStatus = I2C_STATUS_OK;
		I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_BUF, DISABLE);
		I2C_ITConfig(I2CProcessing[i2cIn].I2C_SEL, I2C_IT_EVT, DISABLE);
		I2CProcessing[i2cIn].stateCnt = 0;
		break;
	}
}
