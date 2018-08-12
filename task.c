/*
 * task.ñ
 *
 *  Created on: January 29, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "stdint.h"

#include "NRF24L01user.h"
#include "NRF24L01.h"
#include "moduleHWInit.h"

#include "task.h"


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



void delay_(uint32_t delay){
	uint32_t counter=0;
	while(counter < delay)
	{
		counter++;
	}
}

void task_nrf24l01(void){
	delay_(5000000);

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
	//==============start transmit =======================

	rxData.atmPressure = 13;
	rxData.humifity    = 13;
	rxData.temperature = 13;

	txData.atmPressure = 500;
	txData.humifity    = 30;
	txData.temperature = 0;
	NRF24L01_send_data(nrfTx, sizeof(transactionT), txBuff.buf);

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
			txData.atmPressure++;
			txData.humifity++;
			txData.temperature++;
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
