/*
 * NRF24L01.c
 *
 *  Created on: January 29, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "stdint.h"

#include "NRF24L01.h"

NRF_ERROR NRF24L01_init_mcu( S_nrf_config *const pNRF){
	if(mcu_nrf_init(pNRF)>0){return NRF_ERROR_AMOUNT_NRF;};
	return NRF_ERROR_OK;
}


//---------NRF24L01_init-----------------
// function NRF24L01_init - init NRF24l01 modull
//
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
//          - ps_nrf_init - pointer on structure which contain initialization data
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_init_nrf(S_nrf_config *const pNRF, const S_NRF_Init* ps_nrf_init){

	NRF24L01_enable_crc(pNRF,ps_nrf_init->crc_state);
	NRF24L01_set_crco(pNRF,ps_nrf_init->crc_width);
	NRF24L01_set_address_width(pNRF,ps_nrf_init->address_width);
	NRF24L01_set_num_retransmit(pNRF,ps_nrf_init->auto_retransmit_count);
	NRF24L01_set_delay_retransmit(pNRF,ps_nrf_init->auto_retransmit_delay);
	return NRF_ERROR_OK;
}


//---------S_NRR_init_default-----------------
// function S_NRR_init_default - fill S_NRR_Init structure default parameters
// input arg:
//          - ps_nrf_init - pointer on structure which contain initialization data
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR S_NRF_default(S_NRF_Init* ps_nrf_init){
	ps_nrf_init->crc_state=CRC_STATE_DEFAULT;
	ps_nrf_init->crc_width=CRC_SIZE_DEFAULT;
	ps_nrf_init->address_width=ADDRESS_SIZE_DEFAULT;
	ps_nrf_init->auto_retransmit_delay=AUTO_RETR_DELAY_DEFAULT;
	ps_nrf_init->auto_retransmit_count=AUTO_RETR_COUNT_DEFAULT;
	ps_nrf_init->rf_chanel=RF_CHANEL_DEFAULT;
	ps_nrf_init->speed=SPEED_DEFAULT;
	ps_nrf_init->out_amplifare=OUT_AMPLIFARE_DEFAULT;
	ps_nrf_init->dynamic_payload_state=DYNAMIC_PAYLOAD_STATE_DEFAULT;
	TX_ADDRESS_DEFAULT(ps_nrf_init->transmit_address);
	return NRF_ERROR_OK;
}


//---------NRF24L01_get_config-----------------
// function NRF24L01_get_config - read CONFIG register
// input arg:
//          - pNRF   - pointer on selected NRF24L01+
//  pread_status_reg - status register
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_config(S_nrf_config *const pNRF, uint8_t *pread_status_reg){
	if(NRF24L01_read_reg(pNRF,CONFIG_ADDRESS, 1, pread_status_reg)){return NRF_ERROR_BUSY;}
	return NRF_ERROR_OK;
}


//---------NRF24L01_get_status-----------------
// function NRF24L01_get_status - read STATUS register
// input arg:
//          - pNRF   - pointer on selected NRF24L01+
//  pread_status_reg - status register
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_status(S_nrf_config *const pNRF, uint8_t *ppread_status_reg){
	if(NRF24L01_read_reg(pNRF,STATUS_ADDRESS, 1, ppread_status_reg)){return NRF_ERROR_BUSY;}
	return NRF_ERROR_OK;
}

//---------NRF24L01_reset_flag_interrupt-----------------
// function NRF24L01_reset_flag_interrupt - reset selected interrupt flag
// input arg:
//               - pNRF  - pointer on selected NRF24L01+
// clear_interrupt_flag  - selected interrupt
//  pread_status_reg - status register
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_reset_status_interrupt(S_nrf_config *const pNRF,INTERUPT_MASK clear_interrupt_flag){
	uint8_t pread_status_reg;
	if(IS_INTERUPT_MASK(clear_interrupt_flag)){return NRF_ERROR_INTERUPT_MASK;}
	if(NRF24L01_read_reg(pNRF,STATUS_ADDRESS, 1, &pread_status_reg)){return NRF_ERROR_BUSY;}
	pread_status_reg|=clear_interrupt_flag;
	if(NRF24L01_write_reg(pNRF,STATUS_ADDRESS, 1, &pread_status_reg)){return NRF_ERROR_BUSY;}
	return NRF_ERROR_OK;
}


//---------NRF24L01_get_fifo_status-----------------
// function NRF24L01_get_fifo_status - read STATUS register
// input arg:
//          - pNRF   - pointer on selected NRF24L01+
//  pread_status_reg - status register
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_fifo_status(S_nrf_config *const pNRF, uint8_t *pread_status_reg){
	if(NRF24L01_read_reg(pNRF,FIFO_STATUS_ADDRESS, 1, pread_status_reg)){return NRF_ERROR_BUSY;}
	return NRF_ERROR_OK;
}


//---------NRF24L01_power_switch-----------------
// function NRF24L01_power_switch - set/reset PWR_UP bit in CONFIG register
// input arg:
//          - pNRF   - pointer on selected NRF24L01+
//          - state  - new state
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_power_switch(S_nrf_config *const pNRF, NRF_STATE new_pwr_up_state){
	uint8_t config_temp_reg;
	if(!IS_NRF_STATE(new_pwr_up_state)){return NRF_ERROR_PWR_UP_STATE;}
	if(NRF24L01_read_reg(pNRF,CONFIG_ADDRESS, 1, &config_temp_reg)){return NRF_ERROR_BUSY;}
	((CONFIG*)&config_temp_reg)->PWR_UP=new_pwr_up_state;
	return NRF24L01_write_reg(pNRF,CONFIG_ADDRESS, 1, &config_temp_reg);
	if(NRF24L01_read_reg(pNRF,CONFIG_ADDRESS, 1, &config_temp_reg)){return NRF_ERROR_BUSY;}
	return NRF_ERROR_OK;
}


//---------NRF24L01_enable_crc-----------------
// function NRF24L01_enable_crc - set/reset EN_CRC bit in CONFIG register
// input arg:
//          - pNRF   - pointer on selected NRF24L01+
//          - state  - new state
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_enable_crc(S_nrf_config *const pNRF, NRF_STATE new_crc_state){
	uint8_t config_temp_reg;
	if(!IS_NRF_STATE(new_crc_state)){return NRF_ERROR_PWR_UP_STATE;}
	if(NRF24L01_read_reg(pNRF,CONFIG_ADDRESS, 1, &config_temp_reg)){return NRF_ERROR_BUSY;}
	((CONFIG*)&config_temp_reg)->EN_CRC=new_crc_state;
	return NRF24L01_write_reg(pNRF,CONFIG_ADDRESS, 1, &config_temp_reg);
}


//---------NRF24L01_set_crco-----------------
// function NRF24L01_set_crco - set new value CRCO bit in CONFIG register
// input arg:
//          - pNRF   - pointer on selected NRF24L01+
//          - state  - new state
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_crco(S_nrf_config *const pNRF, CRCO new_crco){
	uint8_t config_temp_reg;
	if(!IS_CRCO(new_crco)){return NRF_ERROR_PWR_UP_STATE;}
	if(NRF24L01_read_reg(pNRF,CONFIG_ADDRESS, 1, &config_temp_reg)){return NRF_ERROR_BUSY;}
	((CONFIG*)&config_temp_reg)->EN_CRC=new_crco;
	return NRF24L01_write_reg(pNRF,CONFIG_ADDRESS, 1, &config_temp_reg);
}


//---------NRF24L01_set_interrupt-----------------
// function NRF24L01_set_interrupt - set new interrupt
// input arg:
//          - pNRF          - pointer on selected NRF24L01+
//          - set_interupt  - sel interrupt: OR INTERUPT_MASK definition
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_interrupt(S_nrf_config *const pNRF, INTERUPT_MASK set_interupt){
	uint8_t config_temp_reg;
	if(IS_INTERUPT_MASK(set_interupt)){return NRF_ERROR_INTERUPT_MASK;}
	if(NRF24L01_read_reg(pNRF,CONFIG_ADDRESS, 1, &config_temp_reg)){return NRF_ERROR_BUSY;}
	config_temp_reg|=set_interupt;
	return NRF24L01_write_reg(pNRF,CONFIG_ADDRESS, 1, &config_temp_reg);
}



//---------NRF24L01_enable_pipe-----------------
// function NRF24L01_enable_pipe - enable selected pipe
// input arg:
//          - pNRF         - pointer on selected NRF24L01+
//          - pipe_number  - number of pipe, using PIPS_DEF "enum"
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_enable_pipe(S_nrf_config *const pNRF, PIPS_DEF PipeNumber){
	uint8_t config_temp_reg;
	if(!IS_PIPE(PipeNumber)){return NRF_ERROR_PIPE_NUMBER;}
	if(NRF24L01_read_reg(pNRF,EN_RXADDR_ADDRESS, 1, &config_temp_reg)){return NRF_ERROR_BUSY;}
	config_temp_reg|=(1<<PipeNumber);
	return NRF24L01_write_reg(pNRF,EN_AA_ADDRESS, 1, &config_temp_reg);
}


//---------NRF24L01_set_address_width-----------------
// function NRF24L01_set_address_width - set address width sel NRF
// input arg:
//          - pNRF           - pointer on selected NRF24L01+
//          - num_retransmit - number of retransmit
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_address_width(S_nrf_config *const pNRF, AW  address_width){
	if(!IS_AW(address_width)){
		return NRF_ERROR_ADDRESS_WIDTH;
	}
	return NRF24L01_write_reg(pNRF,SETUP_AW_ADDRESS, 1, &address_width);
}


//---------NRF24L01_set_num_retransmit-----------------
// function NRF24L01_set_num_retransmit - set number retransmit
// input arg:
//          - pNRF           - pointer on selected NRF24L01+
//          - num_retransmit - number of retransmits, less or equal than  MAX_QUANTITY_RETRANSMIT definition
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_num_retransmit(S_nrf_config *const pNRF, uint8_t  num_retransmit){
	SETUP_RETR S_setup_retr;
	if(num_retransmit>MAX_QUANTITY_RETRANSMIT){
		return NRF_ERROR_EXCEED_QUANTITY_RETRANSMIT;
	}
	if(NRF24L01_read_reg(pNRF,SETUP_RETR_ADDRESS, 1, (uint8_t*)&S_setup_retr)){return NRF_ERROR_BUSY;}
	S_setup_retr.ARC=num_retransmit;
	return NRF24L01_write_reg(pNRF,SETUP_RETR_ADDRESS, 1,(uint8_t*)&S_setup_retr);
}

//---------NRF24L01_set_delay_retransmit-----------------
// function NRF24L01_set_delay_retransmit - set max timeout before retransmit
// input arg:
//          - pNRF            - pointer on selected NRF24L01+
//          - auto_retr_delay - time delay definition, ref RETRANSMIT_DELAY enum
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_delay_retransmit(S_nrf_config *const pNRF, RETRANSMIT_DELAY  auto_retr_delay){
	SETUP_RETR S_setup_retr;
	if(!IS_RETRANSMIT_DELAY(auto_retr_delay)){return NRF_ERROR_RETRANSMIT_DELAY;}
	if(NRF24L01_read_reg(pNRF,SETUP_RETR_ADDRESS, 1, (uint8_t*)&S_setup_retr)){return NRF_ERROR_BUSY;}
	S_setup_retr.ARD=auto_retr_delay;
	return NRF24L01_write_reg(pNRF,SETUP_RETR_ADDRESS, 1,(uint8_t*)&S_setup_retr);
}

//---------NRF24L01_set_rf_chanel-----------------
// function NRF24L01_set_rf_chanel - set number rf channel
// input arg:
//          - pNRF             - pointer on selected NRF24L01+
//          - number_rf_chanel - selected number of rf channel
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_rf_chanel(S_nrf_config *const pNRF, uint8_t number_rf_chanel){
	if(number_rf_chanel>MAX_NUMBER_RF_CHANEL){return NRF_ERROR_EXCEED_RF_CHANNEL;}
	return NRF24L01_write_reg(pNRF,RF_CH_ADDRESS, 1,&number_rf_chanel);
}



//---------NRF24L01_set_RX_addres-----------------
// function NRF24L01_set_RX_addres - set pipe address
// input arg:
//          - pNRF         - pointer on selected NRF24L01+
//          - pipe_number - number of pipe, using PIPS_DEF "enum"
//          - pipe_addres - pointer on Rx address array[5]. For 2-6 pipe have to use member[0] (LSBA)
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_RX_address(S_nrf_config *const pNRF, PIPS_DEF PipeNumber, uint8_t *pPipeAddress){
	if(!IS_PIPE(PipeNumber)){return NRF_ERROR_PIPE_NUMBER;}
	return NRF24L01_write_reg(pNRF,RX_ADDR_P0_ADDRESS+PipeNumber, 5, pPipeAddress);
}


//---------NRF24L01_get_RX_address-----------------
// function NRF24L01_get_RX_address - get pipe address
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
//          - pipe_number - number of pipe, using PIPS_DEF "enum"
//          - pipe_addres - pointer on Rx address array[5]. For 2-6 pipe have to use member[0] (LSBA)
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_RX_address(S_nrf_config *const pNRF, PIPS_DEF PipeNumber, uint8_t *pPipeAddress){
	if(!IS_PIPE(PipeNumber)){return NRF_ERROR_PIPE_NUMBER;}
	return NRF24L01_read_reg(pNRF,RX_ADDR_P0_ADDRESS+PipeNumber, 5, pPipeAddress);
}



//---------NRF24L01_set_TX_addres-----------------
// function NRF24L01_set_TX_addres - set pipe Tx address (address remote Rx )
// input arg:
//          - pNRF         - pointer on selected NRF24L01+
//          - pipe_addres - pointer on Tx address array[5].
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_TX_addres(S_nrf_config *const pNRF,uint8_t *pPipeAddress){
	return NRF24L01_write_reg(pNRF,TX_ADDR_ADDRESS, 5, pPipeAddress);
}


//---------NRF24L01_get_TX_addres-----------------
// function NRF24L01_get_TX_addres - get pipe Tx address (address remote Rx )
// input arg:
//          - pNRF         - pointer on selected NRF24L01+
//          - pipe_addres - pointer on Tx address array[5].
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_TX_addres(S_nrf_config *const pNRF,uint8_t *pPipeAddress){
	return NRF24L01_read_reg(pNRF,TX_ADDR_ADDRESS, 5, pPipeAddress);
}


//---------NRF24L01_set_TX_payload-----------------
// function NRF24L01_set_TX_payload - set pipe address
// input arg:
//          - pNRF         - pointer on selected NRF24L01+
//          - pipe_number  - number of pipe, using PIPS_DEF "enum"
//          - payload_size - size of payload
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_TX_PayloadSize(S_nrf_config *const pNRF,PIPS_DEF PipeNumber,uint8_t *ppayload_size){
	if(!IS_PIPE(PipeNumber)){return NRF_ERROR_PIPE_NUMBER;}
	if(*ppayload_size>MAX_PAYLOAD_SIZE){return NRF_ERROR_PAYLOAD_SIZE;}
	return NRF24L01_write_reg(pNRF,RX_PW_P0_ADDRESS+PipeNumber, 1, ppayload_size);
}


//---------NRF24L01_get_TX_payload-----------------
// function NRF24L01_get_TX_payload - get pipe address
// input arg:
//          - pNRF         - pointer on selected NRF24L01+
//          - pipe_number  - number of pipe, using PIPS_DEF "enum"
//          - payload_size - size of payload
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_TX_PayloadSize(S_nrf_config *const pNRF,PIPS_DEF PipeNumber,uint8_t *ppayload_size){
	uint8_t payload_size;
	if(!IS_PIPE(PipeNumber)){return NRF_ERROR_PIPE_NUMBER;}
	if(*ppayload_size>MAX_PAYLOAD_SIZE){return NRF_ERROR_PAYLOAD_SIZE;}
	       NRF24L01_read_reg(pNRF ,RX_PW_P0_ADDRESS, 1, &payload_size);
	return NRF24L01_read_reg(pNRF ,RX_PW_P0_ADDRESS+PipeNumber, 1, ppayload_size);
}


//---------NRF24L01_send_data-----------------
// function NRF24L01_send_data - send data
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
//          - data_length - number of byte to transmit
//          - p_data      - pointer on Tx data
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_send_data(S_nrf_config *const pNRF,uint8_t data_length, uint8_t const *p_data){
	uint8_t config_temp_reg;
	if(data_length>MAX_NUM_BYTES_IN_RX){return NRF_ERROR_MAX_DATA_SIZE;}
	NRF24L01_write_fifo_tx(pNRF, data_length,p_data); // Load Tx data on fifo_Tx buffer
	// Clearing PRIM_RX bit to enable PTX mode
	if(NRF24L01_read_reg(pNRF,CONFIG_ADDRESS,1,&config_temp_reg)){return NRF_ERROR_BUSY;}
	((CONFIG*)(&config_temp_reg))->PRIM_RX=NRF_RESET;
	NRF24L01_write_reg(pNRF,CONFIG_ADDRESS,1,&config_temp_reg);
	// Making short pulse on CE for start transmit
	NRF_CE_PULSE(pNRF);
	return NRF_ERROR_OK;
}


//---------NRF24L01_set_rx_mode-----------------
// function NRF24L01_set_rx_mode - enable RX mode: set PWR_UP and PRIM_RX bit in
//                                 CONFIG register, set CE pin
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_rx_mode(S_nrf_config *const pNRF){
	uint8_t config_temp_reg;
	if(NRF24L01_read_reg(pNRF,CONFIG_ADDRESS,1,&config_temp_reg)){return NRF_ERROR_BUSY;}
	((CONFIG*)(&config_temp_reg))->PWR_UP  = NRF_SET;
	((CONFIG*)(&config_temp_reg))->PRIM_RX = NRF_SET;
	((CONFIG*)(&config_temp_reg))->CRCO    = NRF_SET;
	NRF24L01_write_reg(pNRF,CONFIG_ADDRESS,1,&config_temp_reg);
	NRF_CE_SET(pNRF);
	return NRF_ERROR_OK;
}

//---------NRF24L01_set_tx_mode-----------------
// function NRF24L01_set_tx_mode - enable RX mode: set PWR_UP, clear PRIM_RX bit in
//                                 CONFIG register, set CE pin
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_tx_mode(S_nrf_config *const pNRF){
	uint8_t config_temp_reg;
	if(NRF24L01_read_reg(pNRF,CONFIG_ADDRESS,1,&config_temp_reg)){return NRF_ERROR_BUSY;}
	((CONFIG*)(&config_temp_reg))->PWR_UP  = NRF_SET;
	((CONFIG*)(&config_temp_reg))->PRIM_RX = NRF_RESET;
	((CONFIG*)(&config_temp_reg))->CRCO    = NRF_SET;
	NRF24L01_write_reg(pNRF,CONFIG_ADDRESS,1,&config_temp_reg);
	NRF_CE_SET(pNRF);
	return NRF_ERROR_OK;
}







// const variable
int const* ptra;
const int* ptrb;

//const address
int *const ptrc;

//const var and data in this address
int const* const ptrd;
const int* const ptre;
