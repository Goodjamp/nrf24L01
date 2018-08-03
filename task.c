/*
 * task.ñ
 *
 *  Created on: January 29, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "stdint.h"

#include "task.h"

S_nrf_config NRF1, NRF2;
NRF_ERROR rez;

u8 read_buf[50]={1,2,3,4,5,6,7,8,9,10,101,102,103,104,105,106,107,108,109,110};
S_GLOBAL_REG_MAP *ps_NRF_reg_map=(S_GLOBAL_REG_MAP*)&read_buf[0];
S_peref s_nrf_hardware_config;
S_GLOBAL_REG_MAP *pGlobalReg=(S_GLOBAL_REG_MAP*)&read_buf[0];

void delay_(u32 delay){
	u32 counter=0;
	while(counter<delay){counter++;}
}


typedef struct {
	CONFIG config_reg;
	EN_AA en_aa;
	EN_RXADDR en_rxaddr;
	RX_ADDR_P0 rx_addr_p0;
	TX_ADDR tx_addr;
	RX_PW_P0 rx_pw_p0;
}NRF_STATE_CONFIG;
NRF_STATE_CONFIG nrf1_state, nrf2_state;
STATUS status_reg;
STATUS status_reg1;
OBSERVE_TX observe_tx;

void nrf_get_state(S_nrf_config *const pNRF, NRF_STATE_CONFIG* pnrf_state);

void task_nrf24l01(void){
	u8 counter=0;
	u8 size_payload=10;
	delay_(500000);
	delay_(500000);
// --------init data NRF ¹1-------------------------
	NRF1.peref.CE_pin=NRF1_CE_PIN;
	NRF1.peref.CE_port=NRF1_CE_PORT;
	NRF1.peref.CSN_pin=NRF1_CSN_PIN;
	NRF1.peref.CSN_port=NRF1_CSN_PORT;
	NRF1.peref.IRQ_pin=NRF1_IRQ_PIN;
	NRF1.peref.IRQ_port=NRF1_IRQ_PORT;
	NRF1.peref.SPI=NRF1_SPI;
	NRF24L01_init_mcu(&NRF1);  // init NRF 1

// --------init data NRF 2-------------------------
	NRF2.peref.CE_pin=NRF2_CE_PIN;
	NRF2.peref.CE_port=NRF2_CE_PORT;
	NRF2.peref.CSN_pin=NRF2_CSN_PIN;
	NRF2.peref.CSN_port=NRF2_CSN_PORT;
	NRF2.peref.IRQ_pin=NRF2_IRQ_PIN;
	NRF2.peref.IRQ_port=NRF2_IRQ_PORT;
	NRF2.peref.SPI=NRF2_SPI;
	NRF24L01_init_mcu(&NRF2);  // init NRF 2

	//nrf_get_state(&NRF1, &nrf1_state);
	//nrf_get_state(&NRF2, &nrf2_state);

   /*----------------START RECEIVER---------------------*/
	NRF24L01_FLUSH_RX(&NRF2);
	NRF24L01_FLUSH_TX(&NRF2);
	// set payload width
	NRF24L01_set_TX_PayloadSize(&NRF2,PIPE0,&size_payload);
	// get status NRF
	NRF24L01_get_status(&NRF2,(u8*)&status_reg);
	NRF24L01_reset_status_interrupt(&NRF2,MASK_RX_DR); // Clear status NRF
	//Set Rx mode
	NRF24L01_set_rx_mode(&NRF2);
	//wait set Rx mode
	delay_(100);


	 /*----------------START TRANSMITER---------------------*/
	NRF24L01_FLUSH_TX(&NRF1);
	NRF24L01_FLUSH_RX(&NRF1);
	// set payload width
	NRF24L01_set_TX_PayloadSize(&NRF1,PIPE0,&size_payload);
    // setup number of retransmit
	SETUP_RETR setup_retr;
	NRF24L01_read_reg(&NRF1,SETUP_RETR_ADDRESS,1,(u8*)&setup_retr);
	setup_retr.ARC=15;
	setup_retr.ARD=15;
	NRF24L01_write_reg(&NRF1,SETUP_RETR_ADDRESS,1,(u8*)&setup_retr);
	//Set Tx mode
	NRF24L01_set_tx_mode(&NRF1);

	//==============transmit first part of data=======================
	NRF24L01_write_fifo_tx(&NRF1,size_payload, read_buf);

	//------------ wait receive  data-------------------
	while(status_reg.RX_DR==0){
		NRF24L01_get_status(&NRF2,(u8*)&status_reg);
		NRF24L01_get_status(&NRF1,(u8*)&status_reg1);
	}; //wait interrupt
	NRF24L01_reset_status_interrupt(&NRF2,MASK_RX_DR);
	NRF24L01_get_status(&NRF2,(u8*)&status_reg);
	NRF2.f_interrupt=0;
	//--------------------------------------------------

	delay_(1000);
	NRF24L01_get_status(&NRF2,(u8*)&status_reg);
	NRF24L01_read_fifo_rx(&NRF2,size_payload, &read_buf[20]);
	//==============transmit second part of data=======================
	NRF24L01_write_fifo_tx(&NRF1,size_payload,(u8*)&read_buf[10]);
	NRF_CE_PULSE((&NRF1))

	//------------ wait receive  data-------------------
	//while(!NRF2.f_interrupt){
	while(status_reg.RX_DR==0){
		NRF24L01_get_status(&NRF2,(u8*)&status_reg);
	}; //wait interrupt
	NRF24L01_reset_status_interrupt(&NRF2,MASK_RX_DR);
	NRF2.f_interrupt=0;
	//--------------------------------------------------

	delay_(1000);
	NRF24L01_get_status(&NRF2,(u8*)&status_reg);
	NRF24L01_read_reg(&NRF2,OBSERVE_TX_ADDRESS,1,(u8*)&observe_tx);
	NRF24L01_read_fifo_rx(&NRF2,size_payload, &read_buf[30]);

	//}

}

void nrf_get_state(S_nrf_config *const pNRF, NRF_STATE_CONFIG* pnrf_state){
	/*
	 * 	CONFIG config_reg;
	EN_AA en_aa;
	EN_RXADDR en_rxaddr;
	RX_ADDR_P0 rx_addr_p0;-
	TX_ADDR tx_addr;-
	RX_PW_P0 rx_pw_p0;-
	 */
	NRF24L01_get_config(pNRF,(u8*)&pnrf_state->config_reg);
	NRF24L01_get_RX_address(pNRF,PIPE0,(u8*)&pnrf_state->rx_addr_p0);
	NRF24L01_get_TX_addres(pNRF,(u8*)&pnrf_state->tx_addr);
	NRF24L01_get_TX_PayloadSize(pNRF,PIPE0,(u8*)&pnrf_state->rx_pw_p0);
	//NRF24L01_get_enable_AA(pNRF, &pnrf_state->en_aa);
	//NRF24L01_get_enable_pipe(pNRF, &pnrf_state->en_rxaddr);
}
