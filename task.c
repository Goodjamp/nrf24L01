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

nrfHeader NRF1, NRF2;
NRF_ERROR rez;

uint8_t read_buf[50]={1,2,3,4,5,6,7,8,9,10,101,102,103,104,105,106,107,108,109,110};
S_GLOBAL_REG_MAP *ps_NRF_reg_map=(S_GLOBAL_REG_MAP*)&read_buf[0];
S_GLOBAL_REG_MAP *pGlobalReg=(S_GLOBAL_REG_MAP*)&read_buf[0];

void delay_(uint32_t delay){
	uint32_t counter=0;
	while(counter<delay){counter++;}
}


typedef struct {
	CONFIG config_reg;
	EN_AA en_aa;
	EN_RXADDR en_rxaddr;
	RX_ADDR_P rx_addr_p0;
	TX_ADDR tx_addr;
	RX_PW_P rx_pw_p0;
}NRF_STATE_CONFIG;
NRF_STATE_CONFIG nrf1_state, nrf2_state;
STATUS status_reg;
STATUS status_reg1;
OBSERVE_TX observe_tx;


static void nrf_get_state(nrfHeader inNRF, NRF_STATE_CONFIG* pnrf_state){
	/*
	 * 	CONFIG config_reg;
	EN_AA en_aa;
	EN_RXADDR en_rxaddr;
	RX_ADDR_P0 rx_addr_p0;-
	TX_ADDR tx_addr;-
	RX_PW_P0 rx_pw_p0;-
	 */
	NRF24L01_get_config(inNRF, &pnrf_state->config_reg);
	NRF24L01_get_RX_address(inNRF,PIPE0,(uint8_t*)&pnrf_state->rx_addr_p0);
	NRF24L01_get_TX_addres(inNRF,(uint8_t*)&pnrf_state->tx_addr);
	NRF24L01_get_TX_PayloadSize(inNRF,PIPE0,(uint8_t*)&pnrf_state->rx_pw_p0);
	//NRF24L01_get_enable_AA(inNRF, &pnrf_state->en_aa);
	//NRF24L01_get_enable_pipe(inNRF, &pnrf_state->en_rxaddr);
}


void task_nrf24l01(void){
	uint8_t size_payload=10;
	delay_(5000000);

	NRF1 = NRF24L01_init();
	NRF2 = NRF24L01_init();
	mcu_nrf_init(NRF1, USER_SPI1);
	mcu_nrf_init(NRF2, USER_SPI2);
	nrf24l01_ce_puls(NRF2);

	//nrf_get_state(&NRF1, &nrf1_state);
	//nrf_get_state(&NRF2, &nrf2_state);

   /*----------------START RECEIVER---------------------*/
	NRF24L01_power_switch(NRF2, NRF_SET);
	NRF24L01_FLUSH_RX(NRF2);
	NRF24L01_FLUSH_TX(NRF2);
	// set payload width
	NRF24L01_set_TX_PayloadSize(NRF2, PIPE0, &size_payload);
	// get status NRF
	NRF24L01_get_status_tx_rx(NRF2,&status_reg);
	NRF24L01_clear_interrupt(NRF2, STATUS_RX_DR); // Clear status NRF
	//Set Rx mode
	NRF24L01_set_rx_mode(NRF2);
	//wait set Rx mode
	delay_(100);


	 /*----------------START TRANSMITER---------------------*/
	NRF24L01_power_switch(NRF1, NRF_SET);
	NRF24L01_FLUSH_TX(NRF1);
	NRF24L01_FLUSH_RX(NRF1);
	// set payload width
	NRF24L01_set_TX_PayloadSize(NRF1,PIPE0,&size_payload);
	// clear all interrupt flags;
	NRF24L01_clear_interrupt(NRF1, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT );
    // setup number of retransmit
	SETUP_RETR setup_retr;
	NRF24L01_read_reg(NRF1,SETUP_RETR_ADDRESS,1,(uint8_t*)&setup_retr);
	setup_retr.ARC=1;
	setup_retr.ARD=0;
	NRF24L01_write_reg(NRF1,SETUP_RETR_ADDRESS,1,(uint8_t*)&setup_retr);
	NRF24L01_set_tx_mode(NRF1);
	//==============transmit first part of data=======================
	NRF24L01_send_data(NRF1,size_payload, read_buf);

	//NRF24L01_set_tx_mode(NRF1);


	nrf24l01_ce_puls (NRF1);
	uint8_t cntWait = 0;

	//------------ wait receive  data-------------------
	NRF24L01_get_status_tx_rx(NRF2,&status_reg);
	while(status_reg.RX_DR == 0){
		if(status_reg1.MAX_RT == 1)
		{
			NRF24L01_clear_interrupt(NRF1, STATUS_MAX_RT );
			NRF24L01_send_data(NRF1,size_payload, read_buf);
			cntWait = 0;
		}
		NRF24L01_get_status_tx_rx(NRF2,&status_reg);
		NRF24L01_get_status_tx_rx(NRF1,&status_reg1);
		cntWait++;
	}; //wait interrupt
	cntWait = 0;
	NRF24L01_clear_interrupt(NRF2,STATUS_RX_DR);
	NRF24L01_get_status_tx_rx(NRF2,&status_reg);

	//--------------------------------------------------

	delay_(1000);
	NRF24L01_get_status_tx_rx(NRF2,&status_reg);
	NRF24L01_read_rx_data(NRF2,size_payload, &read_buf[20]);
	//==============transmit second part of data=======================
	NRF24L01_send_data(NRF1,size_payload,(uint8_t*)&read_buf[10]);

	//------------ wait receive  data-------------------
	//while(!NRF2.f_interrupt){
	while(status_reg.RX_DR==0){
		if(status_reg1.MAX_RT)
		{
			nrf24l01_ce_clear(NRF1);
			NRF24L01_clear_interrupt(NRF1, STATUS_MAX_RT );
			NRF24L01_send_data(NRF1,size_payload, read_buf);
		}
		NRF24L01_get_status_tx_rx(NRF2,&status_reg);
	}; //wait interrupt
	NRF24L01_clear_interrupt(NRF2,STATUS_RX_DR);
	//--------------------------------------------------

	delay_(1000);
	NRF24L01_get_status_tx_rx(NRF2,&status_reg);
	NRF24L01_read_reg(NRF2,OBSERVE_TX_ADDRESS,1,(uint8_t*)&observe_tx);
	NRF24L01_read_rx_data(NRF2,size_payload, &read_buf[30]);
	NRF24L01_read_rx_data(NRF2,size_payload, &read_buf[30]);
	NRF24L01_read_rx_data(NRF2,size_payload, &read_buf[30]);
}
