/*
 * task.ñ
 *
 *  Created on: January 29, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "stdint.h"
#include "stddef.h"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "timProcessing.h"

#include "NRF24L01user.h"
#include "NRF24L01.h"
#include "moduleHWInit.h"

#include "i2c_user_interface.h"
#include "BME280_user_interface.h"

#include "sensorProcessing.h"





#define MAX_SENSOR_INIT_ITEM   4

static uint8_t defAddress[5] = "METEO";

#pragma pack(push, 1)
typedef struct
{
	uint8_t status;
    int16_t temperature;
    int16_t humifity;
    int16_t atmPressure;
}transactionT;
#pragma pack(pop)

typedef union
{
	uint8_t      *buf;
	transactionT *data;
}transactionBuffT;


BME280_STATUS bmeStatus;
float rezMesHumidity;
float rezMesTemperature;
float rezMesPressure;

STATUS statusRx;
STATUS statusTx;
nrfHeader nrfTx, nrfRx;
NRF_ERROR rez;
transactionT txData;
transactionT rxData;
transactionBuffT txBuff =
{
	.data = &txData
};
transactionBuffT rxBuff =
{
	.data = &rxData
};


//---------------------------------I2C user implementation functions-----------------------
void i2cInitGpio(uint8_t step){

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPBEN, ENABLE);


	// config I2C CSCL GPIO
	GPIO_InitStructure.GPIO_Pin = I2C1_SCL;
	GPIO_InitStructure.GPIO_Mode = (step) ? (GPIO_Mode_AF_OD) : (GPIO_Mode_IPU);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStructure);
	// config I2C CSDA GPIO
	GPIO_InitStructure.GPIO_Pin = ( I2C1_SDA);
	GPIO_InitStructure.GPIO_Mode =  (step) ? (GPIO_Mode_AF_OD) : ( GPIO_Mode_IPU);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStructure);
	// Enable Alternate function
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void delay_us(uint32_t timeout)
{
	uint32_t steps = timeout/10;
    volatile uint32_t cnt = 0;
	while( cnt++ < steps){};
}


void i2cRecover(uint32_t i2cFRQ)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPBEN, ENABLE);

	if(GPIO_ReadInputDataBit(I2C1_SCL_PORT, I2C1_SDA) == Bit_SET)
	{
		return;
	}

	GPIO_SetBits(I2C1_SCL_PORT, I2C1_SCL);
	// config I2C CSCL GPIO
	GPIO_InitStructure.GPIO_Pin   = I2C1_SCL;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	/*Config CSL functions of I2C1*/
	GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStructure);


	GPIO_SetBits(I2C1_SDA_PORT, I2C1_SDA);
	// config I2C CSDA GPIO
	GPIO_InitStructure.GPIO_Pin   = I2C1_SDA;
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStructure);


	//gen 9 pulses
	uint32_t halfPeriodUs = SystemCoreClock/(i2cFRQ * 2);
    volatile uint8_t cnt = 0;
	#define NUMBER_PULSES    9
	while(cnt++ < NUMBER_PULSES)
	{
		GPIO_ResetBits(I2C1_SCL_PORT, I2C1_SCL);
		delay_us(halfPeriodUs);
		GPIO_SetBits(I2C1_SCL_PORT, I2C1_SCL);
		delay_us(halfPeriodUs);
	}

	// generate stop order
	GPIO_ResetBits(I2C1_SCL_PORT, I2C1_SCL);
	GPIO_ResetBits(I2C1_SCL_PORT, I2C1_SDA);
	delay_us(halfPeriodUs);
	GPIO_SetBits(I2C1_SCL_PORT, I2C1_SCL);
	delay_us(halfPeriodUs);
	GPIO_SetBits(I2C1_SCL_PORT, I2C1_SDA);
}


uint32_t i2cgetTimeMs(void)
{
	return xTaskGetTickCount();
}

//-----------------------------------sensor user implementftion functions------------------------------
TRANSACION_STATUS BMEReadData (uint8_t sensorAddress, uint8_t sensorReagister, uint8_t *data, uint8_t numData){
	if(i2cRxData(I2C_SENSOR, sensorAddress, sensorReagister, numData, data) != I2C_STATUS_OK)
	{
		return TRANSACION_STATUS_ERROR;
	};
	return TRANSACION_STATUS_OK;
}


TRANSACION_STATUS BMEWriteData(uint8_t sensorAddress, uint8_t sensorReagister, uint8_t *data, uint8_t numData){
	if( i2cTxData(I2C_SENSOR, sensorAddress, sensorReagister, numData, data) != I2C_STATUS_OK)
	{
		return TRANSACION_STATUS_ERROR;
	};
	return TRANSACION_STATUS_OK;
}


uint32_t  sensorGetTime(void)
{
	return xTaskGetTickCount();
}


void i2c_init(void){
	I2C_configDef i2c_configParamiters = {
			.frequencyI2C = I2C_SENSOR_FRQ_HZ
	};
	i2cConfig(I2C_SENSOR, &i2c_configParamiters);
}


BME280Handler sensorHandler;


BME280_STATUS initI2C_Sensor(void){
	BME280_STATUS bmeStatus;
	bool sensorIsOnLine = false;

	i2c_init();

	BME280_setI2CAddress(&sensorHandler, BME280_ADDRESS_HIGHT);
    // Is sensor online ?
	if(BME280_STATUS_OK  != (bmeStatus = BME280_isOnLine(&sensorHandler, &sensorIsOnLine) ) ){
        return bmeStatus;
	}

	if( !sensorIsOnLine ){
		return BME280_STATUS_SENSOR_ERROR;
	}

	if(BME280_STATUS_OK  != (bmeStatus = BME280_init(&sensorHandler) ) )
	{
		return bmeStatus;
	}
	// Enable measurement all value with 16 oversemple
	if(BME280_STATUS_OK  != (bmeStatus = BME280_setValueMesState(&sensorHandler, MES_VALUE_TEMPERATURE, MES_STATE_ENABLE) ) )
	{
		return bmeStatus;
	}
	if(BME280_STATUS_OK  != (bmeStatus = BME280_setOverSample(&sensorHandler, MES_VALUE_HUMIDITY, OVERSEMPLE_16) ) )
	{
		return bmeStatus;
	}
	if(BME280_STATUS_OK  != (bmeStatus = BME280_setValueMesState(&sensorHandler, MES_VALUE_PRESSURE, MES_STATE_ENABLE) ) )
	{
		return bmeStatus;
	}
	if(BME280_STATUS_OK  != (bmeStatus = BME280_setOverSample(&sensorHandler, MES_VALUE_HUMIDITY, OVERSEMPLE_16) ) )
	{
		return bmeStatus;
	}
	if(BME280_STATUS_OK  != (bmeStatus = BME280_setValueMesState(&sensorHandler, MES_VALUE_HUMIDITY, MES_STATE_ENABLE) ) )
	{
		return bmeStatus;
	}
	if(BME280_STATUS_OK  != (bmeStatus = BME280_setOverSample(&sensorHandler, MES_VALUE_HUMIDITY, OVERSEMPLE_16) ) )
	{
		return bmeStatus;
	}
    // set delay between measurement equal 65 ms
	if(BME280_STATUS_OK  != (bmeStatus = BME280_setMesDelay(&sensorHandler, MEASUREMENT_DELAY_65_5ms) ) )
	{
		return bmeStatus;
	}
	return BME280_STATUS_OK;
}

void delay_(uint32_t delay){
	uint32_t counter=0;
	while(counter < delay)
	{
		counter++;
	}
}


void task_nrf24l01(void){
	delay_(5000000);

	// initilisation sensor
	for(uint8_t cnt = 0; cnt < MAX_SENSOR_INIT_ITEM; cnt++)
	{
	    if( (bmeStatus = initI2C_Sensor()) ==  BME280_STATUS_OK)
	    {
	        break;
	    }
	}


	nrfTx = NRF24L01_init(NRF_INTERFACE_N01);
	nrfRx = NRF24L01_init(NRF_INTERFACE_N02);

   /*----------------START RECEIVER---------------------*/
	NRF24L01_power_switch(nrfRx, NRF_SET);
	NRF24L01_FLUSH_RX(nrfRx);
	NRF24L01_FLUSH_TX(nrfRx);
	NRF24L01_set_RX_address(nrfRx, PIPE0, defAddress);
	NRF24L01_set_crco(nrfRx, CRCO_2_BYTES);
	//NRF24L01_set_TX_addres (NRF2, defAddress);
	// set payload width
	NRF24L01_set_TX_PayloadSize(nrfRx, PIPE0, sizeof(transactionT));
	NRF24L01_clear_interrupt(nrfRx, STATUS_RX_DR); // Clear status NRF
	//Set Rx mode
	NRF24L01_set_rx_mode(nrfRx);
	//wait set Rx mode
	delay_(100);


	 /*----------------START TRANSMITER---------------------*/
	NRF24L01_power_switch(nrfTx, NRF_SET);
	NRF24L01_FLUSH_TX(nrfTx);
	NRF24L01_FLUSH_RX(nrfTx);
	NRF24L01_set_RX_address(nrfTx, PIPE0, defAddress);
	NRF24L01_set_TX_addres (nrfTx, defAddress);
	NRF24L01_set_crco(nrfTx, CRCO_2_BYTES);
	// set payload width
	NRF24L01_set_TX_PayloadSize(nrfTx, PIPE0, sizeof(transactionT));
	// clear all interrupt flags;
	NRF24L01_clear_interrupt(nrfTx, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT );
    // setup number of retransmit
	NRF24L01_set_num_retr(nrfTx, 2);
	NRF24L01_set_delay_retr(nrfTx, WAIT_500uS);
	NRF24L01_set_tx_mode(nrfTx);

	//------------ wait receive  data-------------------
	//while(statusRx.RX_DR == 0){
	while( 1 ){
		NRF24L01_get_status_tx_rx(nrfRx, &statusRx);
		NRF24L01_get_status_tx_rx(nrfTx, &statusTx);
		if(statusTx.MAX_RT == 1)
		{
			NRF24L01_FLUSH_TX(nrfTx);
			NRF24L01_clear_interrupt(nrfTx, STATUS_MAX_RT );
			NRF24L01_send_data(nrfTx, sizeof(transactionT), txBuff.buf);
			continue;
		}
		if(statusTx.TX_DS == 1)
		{
			NRF24L01_clear_interrupt(nrfTx, STATUS_MAX_RT );
            if( bmeStatus !=  BME280_STATUS_OK)
            {
				for(uint8_t cnt = 0; cnt < MAX_SENSOR_INIT_ITEM; cnt++)
				{
					if( (bmeStatus = initI2C_Sensor()) ==  BME280_STATUS_OK)
			        {
			            break;
			        }
                }
			}

            if( bmeStatus ==  BME280_STATUS_OK)
            {
                if(BME280_STATUS_OK  == (bmeStatus = BME280_forcedMes(&sensorHandler, &rezMesTemperature,
							                                                          &rezMesPressure,
								                                                      &rezMesHumidity) ) )
                {
				    txData.temperature = rezMesTemperature *10;
				    txData.humifity    = rezMesHumidity    *10;
				    txData.atmPressure = rezMesPressure;
				    txData.status      = 0;
			   }
                else
                {
                	txData.status = 1;
                }
			}
            else
            {
            	txData.status = 1;
            }
			NRF24L01_read_rx_data(nrfRx, sizeof(transactionT), rxBuff.buf);
			delay_(72 * 70000);
			NRF24L01_send_data(nrfTx, sizeof(transactionT), txBuff.buf);
		}
	}; //wait interrupt
	NRF24L01_clear_interrupt(nrfRx,STATUS_RX_DR);
	NRF24L01_get_status_tx_rx(nrfRx, &statusRx);
	NRF24L01_read_rx_data(nrfRx, sizeof(transactionT), rxBuff.buf);

	//==============transmit second part of data=======================
	NRF24L01_send_data(nrfTx, sizeof(transactionT), txBuff.buf);

	//------------ wait receive data-------------------
	//while(!nrfRx.f_interrupt){
	while(statusRx.RX_DR==0){
		if(statusTx.MAX_RT == 1)
		{
			NRF24L01_clear_interrupt(nrfTx, STATUS_MAX_RT );
			NRF24L01_send_data(nrfTx, sizeof(transactionT), txBuff.buf);
		}
		NRF24L01_get_status_tx_rx(nrfRx, &statusRx);
		NRF24L01_get_status_tx_rx(nrfTx, &statusTx);
	}; //wait interrupt
	NRF24L01_clear_interrupt(nrfRx,STATUS_RX_DR);
	//--------------------------------------------------

	delay_(1000);
	NRF24L01_get_status_tx_rx(nrfRx,&statusRx);
	NRF24L01_read_rx_data(nrfRx, sizeof(transactionT), rxBuff.buf);
	NRF24L01_read_rx_data(nrfRx, sizeof(transactionT), rxBuff.buf);
	NRF24L01_read_rx_data(nrfRx, sizeof(transactionT), rxBuff.buf);
}
