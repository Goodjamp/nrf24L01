/*
 * NRF24L01.c
 *
 *  Created on: January 29, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "stdint.h"
#include "stddef.h"
#include "string.h"

#include "NRF24L01.h"
#include "NRF24L01user.h"


#define restricAvoidType(USER_TYPE, UNION_NAME, IN_ARG) \
union                                                   \
{                                                       \
    uint8_t    *simplType;                              \
    USER_TYPE  *userType;                               \
}UNION_NAME =                                           \
{                                                       \
	.userType = IN_ARG                                  \
};

struct{
    nrfHeaderT nrfP[MAX_NUMBER_OF_NRF];
    uint8_t cntNRF;
}nrfInitState;


nrfHeader NRF24L01_init(NRF_INTERFACE inINterface)
{
	if(nrfInitState.cntNRF == sizeof(nrfInitState.nrfP) / sizeof(nrfInitState.nrfP[0]))
	{
		return NULL;
	}

	// configuration all need interfaces
	nrf24l01_interface_init(&nrfInitState.nrfP[nrfInitState.cntNRF], inINterface);
	// read all memory map pf selected NRF
	restricAvoidType(S_GLOBAL_REG_MAP, config, &nrfInitState.nrfP[nrfInitState.cntNRF].global_reg_map);
	uint8_t cnt = 0;
	uint8_t lenRead = 1;
	for(;cnt < (REG_MAP_FIRST_PART_LEN + REG_MAP_SECOND_PART_LEN); cnt++)
	{
		lenRead = (cnt == RX_ADDR_P0_ADDRESS ||
				   cnt == RX_ADDR_P1_ADDRESS ||
				   cnt == TX_ADDR_ADDRESS)    ?  (5):(1);
		if(NRF24L01_read_reg(&nrfInitState.nrfP[nrfInitState.cntNRF],
				             ((cnt < REG_MAP_FIRST_PART_LEN) ? (CONFIG_ADDRESS + cnt) : (DYNPD_ADDRESS + cnt - REG_MAP_FIRST_PART_LEN)),
				             lenRead,
				             config.simplType + cnt))
	    {
		return NULL;
	    }
	}
	return &nrfInitState.nrfP[nrfInitState.cntNRF++];
}


//---------NRF24L01_init-----------------
// function NRF24L01_init - init NRF24l01 modull
//
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
//          - ps_nrf_init - pointer on structure which contain initialization data
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_init_nrf(nrfHeader inNRF, const S_NRF_Init* ps_nrf_init)
{
	NRF24L01_enable_crc(inNRF,ps_nrf_init->crc_state);
	NRF24L01_set_crco(inNRF,ps_nrf_init->crc_width);
	NRF24L01_set_address_width(inNRF,ps_nrf_init->address_width);
	NRF24L01_set_num_retr(inNRF,ps_nrf_init->auto_retransmit_count);
	NRF24L01_set_delay_retr(inNRF,ps_nrf_init->auto_retransmit_delay);
	return NRF_OK;
}


//---------S_NRR_init_default-----------------
// function S_NRR_init_default - fill S_NRR_Init structure default parameters
// input arg:
//          - ps_nrf_init - pointer on structure which contain initialization data
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR S_NRF_default_init(S_NRF_Init* ps_nrf_init)
{
	ps_nrf_init->crc_state             = CRC_STATE_DEFAULT;
	ps_nrf_init->crc_width             = CRC_SIZE_DEFAULT;
	ps_nrf_init->address_width         = ADDRESS_SIZE_DEFAULT;
	ps_nrf_init->auto_retransmit_delay = AUTO_RETR_DELAY_DEFAULT;
	ps_nrf_init->auto_retransmit_count = AUTO_RETR_COUNT_DEFAULT;
	ps_nrf_init->rf_chanel             = RF_CHANEL_DEFAULT;
	ps_nrf_init->dr                    = SPEED_DEFAULT;
	ps_nrf_init->out_amplifare         = OUT_AMPLIFARE_DEFAULT;
	ps_nrf_init->dynamic_payload_state = DYNAMIC_PAYLOAD_STATE_DEFAULT;
	TX_ADDRESS_DEFAULT(ps_nrf_init->transmit_address);
	return NRF_OK;
}


//---------NRF24L01_get_config-----------------
// function NRF24L01_get_config - read CONFIG register
// input arg:
//          - inNRF   - pointer on selected NRF24L01+
//  pread_status_reg - status register
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_config(nrfHeader inNRF, CONFIG *pread_configuration_reg)
{
	*pread_configuration_reg = inNRF->global_reg_map.config;
	return NRF_OK;
}


//---------NRF24L01_get_status-----------------
// function NRF24L01_get_status - read STATUS register
// input arg:
//          - inNRF   - pointer on selected NRF24L01+
//  pread_status_reg - status register
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_status_tx_rx(nrfHeader inNRF, STATUS *ppread_satus_reg)
{
	restricAvoidType(STATUS, readReg, ppread_satus_reg)
	if(NRF24L01_read_reg(inNRF,STATUS_ADDRESS, 1, readReg.simplType))
	{
		return NRF_BUSY;
	}
	return NRF_OK;
}

//---------NRF24L01_reset_flag_interrupt-----------------
// function NRF24L01_reset_flag_interrupt - reset selected interrupt flag
// input arg:
//               - inNRF  - pointer on selected NRF24L01+
// clear_interrupt_flag  - selected interrupt
//  pread_status_reg - status register
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_clear_interrupt(nrfHeader inNRF,STATUS_MASK clear_interrupt_flag)
{
	uint8_t pread_status_reg;
	if(IS_INTERUPT_MASK(clear_interrupt_flag))
	{
		return NRF_ERROR_INTERUPT_MASK;
	}
	if(NRF24L01_read_reg(inNRF,STATUS_ADDRESS, 1, &pread_status_reg))
	{
		return NRF_BUSY;
	}
	pread_status_reg |= clear_interrupt_flag;
	if(NRF24L01_write_reg(inNRF,STATUS_ADDRESS, 1, &pread_status_reg))
	{
		return NRF_BUSY;
	}
	return NRF_OK;
}


//---------NRF24L01_get_fifo_status-----------------
// function NRF24L01_get_fifo_status - read STATUS register
// input arg:
//          - inNRF   - pointer on selected NRF24L01+
//  pread_status_reg - status register
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_fifo_status(nrfHeader inNRF, uint8_t *pread_status_reg)
{
	if(NRF24L01_read_reg(inNRF,FIFO_STATUS_ADDRESS, 1, pread_status_reg))
	{
		return NRF_BUSY;
	}
	return NRF_OK;
}


//---------NRF24L01_power_switch-----------------
// function NRF24L01_power_switch - set/reset PWR_UP bit in CONFIG register
// input arg:
//          - inNRF   - pointer on selected NRF24L01+
//          - state  - new state
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_power_switch(nrfHeader inNRF, NRF_STATE new_pwr_up_state)
{
	if(!IS_NRF_STATE(new_pwr_up_state))
	{
		return NRF_ERROR_PWR_UP_STATE;
	}
	inNRF->global_reg_map.config.PWR_UP = new_pwr_up_state;
	restricAvoidType(CONFIG, readReg, &inNRF->global_reg_map.config);
	return NRF24L01_write_reg(inNRF, CONFIG_ADDRESS, 1, readReg.simplType);
}


//---------NRF24L01_enable_crc-----------------
// function NRF24L01_enable_crc - set/reset EN_CRC bit in CONFIG register
// input arg:
//          - inNRF   - pointer on selected NRF24L01+
//          - state  - new state
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_enable_crc(nrfHeader inNRF, NRF_STATE new_crc_state)
{
	inNRF->global_reg_map.config.EN_CRC = new_crc_state;
	restricAvoidType(CONFIG, readReg, &inNRF->global_reg_map.config);
	return NRF24L01_write_reg(inNRF, CONFIG_ADDRESS, 1, readReg.simplType);
}


//---------NRF24L01_set_crco-----------------
// function NRF24L01_set_crco - set new value CRCO bit in CONFIG register
// input arg:
//          - inNRF   - pointer on selected NRF24L01+
//          - state  - new state
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_crco(nrfHeader inNRF, CRCO new_crco)
{
	if(!IS_CRCO(new_crco))
	{
		return NRF_ERROR_PWR_UP_STATE;
	}
	inNRF->global_reg_map.config.CRCO = new_crco;
	restricAvoidType(CONFIG, readReg, &inNRF->global_reg_map.config);
	return NRF24L01_write_reg(inNRF, CONFIG_ADDRESS, 1, readReg.simplType);
}


//---------NRF24L01_set_interrupt-----------------
// function NRF24L01_set_interrupt - set new interrupt
// input arg:
//          - inNRF          - pointer on selected NRF24L01+
//          - set_interupt  - sel interrupt: OR INTERUPT_MASK definition
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_interrupt(nrfHeader inNRF, STATUS_MASK set_interupt)
{
	if(IS_INTERUPT_MASK(set_interupt))
	{
		return NRF_ERROR_INTERUPT_MASK;
	}
	restricAvoidType(CONFIG, readReg, &inNRF->global_reg_map.config);
	*readReg.simplType |= set_interupt;
	return NRF24L01_write_reg(inNRF, CONFIG_ADDRESS, 1, readReg.simplType);
}



//---------NRF24L01_enable_pipe-----------------
// function NRF24L01_enable_pipe - enable selected pipe
// input arg:
//          - inNRF         - pointer on selected NRF24L01+
//          - pipe_number  - number of pipe, using PIPS_DEF "enum"
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_enable_pipe(nrfHeader inNRF, PIPS_DEF PipeNumber)
{
	if(!IS_PIPE(PipeNumber))
	{
		return NRF_ERROR_PIPE_NUMBER;
	}
	restricAvoidType(EN_AA, readReg, &inNRF->global_reg_map.en_aa);
	*readReg.simplType |= (1<<PipeNumber);
	return NRF24L01_write_reg(inNRF, EN_AA_ADDRESS, 1, readReg.simplType);
}


//---------NRF24L01_set_address_width-----------------
// function NRF24L01_set_address_width - set address width sel NRF
// input arg:
//          - inNRF           - pointer on selected NRF24L01+
//          - num_retransmit - number of retransmit
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_address_width(nrfHeader inNRF, AW  address_width)
{
	if(!IS_AW(address_width))
	{
		return NRF_ERROR_ADDRESS_WIDTH;
	}
	inNRF->global_reg_map.setup_aw.AW = address_width;
	restricAvoidType(SETUP_AW, readReg, &inNRF->global_reg_map.setup_aw);
	return NRF24L01_write_reg(inNRF,SETUP_AW_ADDRESS, 1, readReg.simplType);
}


//---------NRF24L01_set_num_retransmit-----------------
// function NRF24L01_set_num_retransmit - set number retransmit
// input arg:
//          - inNRF           - pointer on selected NRF24L01+
//          - num_retransmit - number of retransmits, less or equal than  MAX_QUANTITY_RETRANSMIT definition
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_num_retr(nrfHeader inNRF, uint8_t  num_retransmit)
{
	if(num_retransmit > MAX_QUANTITY_RETRANSMIT)
	{
		return NRF_ERROR_QUANTITY_RETRANSMIT;
	}
	inNRF->global_reg_map.setup_petr.ARC = num_retransmit;
	restricAvoidType(SETUP_RETR, readReg, &inNRF->global_reg_map.setup_petr);
	return NRF24L01_write_reg(inNRF, SETUP_RETR_ADDRESS, 1, readReg.simplType);
}

//---------NRF24L01_set_delay_retransmit-----------------
// function NRF24L01_set_delay_retransmit - set max timeout before retransmit
// input arg:
//          - inNRF            - pointer on selected NRF24L01+
//          - auto_retr_delay - time delay definition, ref RETRANSMIT_DELAY enum
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_delay_retr(nrfHeader inNRF, RETRANSMIT_DELAY  auto_retr_delay)
{
	if(!IS_RETRANSMIT_DELAY(auto_retr_delay))
	{
		return NRF_ERROR_RETRANSMIT_DELAY;
	}
	inNRF->global_reg_map.setup_petr.ARD = auto_retr_delay;
	restricAvoidType(SETUP_RETR, readReg, &inNRF->global_reg_map.setup_petr);
	return NRF24L01_write_reg(inNRF, SETUP_RETR_ADDRESS, 1, readReg.simplType);
}

NRF_ERROR NRF24L01_set_rf_dr(nrfHeader inNRF, RF_DR  inDR)
{
	if(!IS_RF_DR(inDR))
	{
		return NRF_ERROR_ARG;
	}
	inNRF->global_reg_map.rf_setup.RF_DR_LOW   = (uint8_t)0b01 & inDR;
	inNRF->global_reg_map.rf_setup.RF_DR_HIGHT = (uint8_t)0b10 & inDR;
	restricAvoidType(RF_SETUP, readReg, &inNRF->global_reg_map.rf_setup);
	return NRF24L01_write_reg(inNRF, RF_SETUP_ADDRESS, 1, readReg.simplType);
}


//---------NRF24L01_set_rf_chanel-----------------
// function NRF24L01_set_rf_chanel - set number rf channel
// input arg:
//          - inNRF             - pointer on selected NRF24L01+
//          - number_rf_chanel - selected number of rf channel
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_rf_chanel(nrfHeader inNRF, uint8_t number_rf_chanel)
{
	if(number_rf_chanel>MAX_NUMBER_RF_CHANEL)
	{
		return NRF_ERROR_EXCEED_RF_CHANNEL;
	}
	inNRF->global_reg_map.rf_ch.RF_CH = number_rf_chanel;
	restricAvoidType(RF_CH, readReg, &inNRF->global_reg_map.rf_ch);
	return NRF24L01_write_reg(inNRF, RF_CH_ADDRESS, 1, readReg.simplType);
}


//---------NRF24L01_set_RX_addres-----------------
// function NRF24L01_set_RX_addres - set pipe address
// input arg:
//          - inNRF         - pointer on selected NRF24L01+
//          - pipe_number - number of pipe, using PIPS_DEF "enum"
//          - pipe_addres - pointer on Rx address array[5]. For 2-6 pipe have to use member[0] (LSBA)
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_RX_address(nrfHeader inNRF, PIPS_DEF PipeNumber, uint8_t *pPipeAddress)
{
	switch(PipeNumber)
	{
	case PIPE0:
	{
		restricAvoidType(RX_ADDR_P0_1, readReg, &inNRF->global_reg_map.rx_addr_p0);
		memcpy(readReg.simplType, pPipeAddress, 5);
		return NRF24L01_write_reg(inNRF, RX_ADDR_P0_ADDRESS, 5, readReg.simplType);
	}
	case PIPE1:
	{
		restricAvoidType(RX_ADDR_P0_1, readReg, &inNRF->global_reg_map.rx_addr_p1);
		memcpy(readReg.simplType, pPipeAddress, 5);
		return NRF24L01_write_reg(inNRF, RX_ADDR_P1_ADDRESS, 5, readReg.simplType);
	}
	case PIPE2:
	{
		inNRF->global_reg_map.rx_addr_lsb_p2.RX_ADDR_P_LSB = *pPipeAddress;
		restricAvoidType(RX_ADDR_P2_5, readReg, &inNRF->global_reg_map.rx_addr_lsb_p2);
		return NRF24L01_write_reg(inNRF, RX_ADDR_P2_ADDRESS, 1, readReg.simplType);
	}
	case PIPE3:
	{
		inNRF->global_reg_map.rx_addr_lsb_p3.RX_ADDR_P_LSB = *pPipeAddress;
		restricAvoidType(RX_ADDR_P2_5, readReg, &inNRF->global_reg_map.rx_addr_lsb_p3);
		return NRF24L01_write_reg(inNRF, RX_ADDR_P3_ADDRESS, 1, readReg.simplType);
	}
	case PIPE4:
	{
		inNRF->global_reg_map.rx_addr_lsb_p4.RX_ADDR_P_LSB = *pPipeAddress;
		restricAvoidType(RX_ADDR_P2_5, readReg, &inNRF->global_reg_map.rx_addr_lsb_p4);
		return NRF24L01_write_reg(inNRF, RX_ADDR_P4_ADDRESS, 1, readReg.simplType);
	}
	case PIPE5:
	{
		inNRF->global_reg_map.rx_addr_lsb_p5.RX_ADDR_P_LSB = *pPipeAddress;
		restricAvoidType(RX_ADDR_P2_5, readReg, &inNRF->global_reg_map.rx_addr_lsb_p5);
		return NRF24L01_write_reg(inNRF, RX_ADDR_P5_ADDRESS, 1, readReg.simplType);
	}
	default: return NRF_ERROR_PIPE_NUMBER;
	}
}


//---------NRF24L01_get_RX_address-----------------
// function NRF24L01_get_RX_address - get pipe address
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
//          - pipe_number - number of pipe, using PIPS_DEF "enum"
//          - pipe_addres - pointer on Rx address array[5]. For 2-6 pipe have to use member[0] (LSBA)
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_RX_address(nrfHeader inNRF, PIPS_DEF PipeNumber, uint8_t *pPipeAddress)
{
	switch(PipeNumber)
	{
	case PIPE0:
		memcpy(pPipeAddress, &inNRF->global_reg_map.rx_addr_p0, 5);
	case PIPE1:
		memcpy(pPipeAddress, &inNRF->global_reg_map.rx_addr_p0, 5);
	case PIPE2:
		*pPipeAddress = inNRF->global_reg_map.rx_addr_lsb_p2.RX_ADDR_P_LSB;
	case PIPE3:
		*pPipeAddress = inNRF->global_reg_map.rx_addr_lsb_p3.RX_ADDR_P_LSB;
	case PIPE4:
		*pPipeAddress = inNRF->global_reg_map.rx_addr_lsb_p4.RX_ADDR_P_LSB;
	case PIPE5:
		*pPipeAddress = inNRF->global_reg_map.rx_addr_lsb_p5.RX_ADDR_P_LSB;
	default: return NRF_ERROR_PIPE_NUMBER;
	}
	return NRF_OK;
}



//---------NRF24L01_set_TX_addres-----------------
// function NRF24L01_set_TX_addres - set pipe Tx address (address remote Rx )
// input arg:
//          - inNRF         - pointer on selected NRF24L01+
//          - pipe_addres - pointer on Tx address array[5].
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_TX_addres(nrfHeader inNRF, uint8_t *pPipeAddress)
{
	restricAvoidType(TX_ADDR, readReg, &inNRF->global_reg_map.tx_addr);
	memcpy(readReg.simplType, pPipeAddress, 5);
	return NRF24L01_write_reg(inNRF, TX_ADDR_ADDRESS, 5, readReg.simplType);
}


//---------NRF24L01_get_TX_addres-----------------
// function NRF24L01_get_TX_addres - get pipe Tx address (address remote Rx )
// input arg:
//          - inNRF         - pointer on selected NRF24L01+
//          - pipe_addres - pointer on Tx address array[5].
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_TX_addres(nrfHeader inNRF,uint8_t *pPipeAddress)
{
	memcpy(pPipeAddress, &inNRF->global_reg_map.tx_addr, 5);
	return NRF_OK;
}


//---------NRF24L01_set_TX_payload-----------------
// function NRF24L01_set_TX_payload - set pipe address
// input arg:
//          - inNRF         - pointer on selected NRF24L01+
//          - pipe_number  - number of pipe, using PIPS_DEF "enum"
//          - payload_size - size of payload
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_TX_PayloadSize(nrfHeader inNRF, PIPS_DEF PipeNumber, uint8_t ppayload_size)
{
	if( ppayload_size > MAX_PAYLOAD_SIZE)
	{
		return NRF_ERROR_PAYLOAD_SIZE;
	}

	switch(PipeNumber)
	{
	case PIPE0:
	{
		restricAvoidType(RX_PW_P, readReg, &inNRF->global_reg_map.rx_pw_p0);
		inNRF->global_reg_map.rx_pw_p0.RX_PW_P = ppayload_size;
		return NRF24L01_write_reg(inNRF, RX_PW_P0_ADDRESS, 1, readReg.simplType);
	}
	case PIPE1:
	{
		restricAvoidType(RX_PW_P, readReg, &inNRF->global_reg_map.rx_pw_p1);
		inNRF->global_reg_map.rx_pw_p1.RX_PW_P = ppayload_size;
		return NRF24L01_write_reg(inNRF, RX_PW_P1_ADDRESS, 1, readReg.simplType);
	}
	case PIPE2:
	{
		restricAvoidType(RX_PW_P, readReg, &inNRF->global_reg_map.rx_pw_p2);
		inNRF->global_reg_map.rx_pw_p2.RX_PW_P = ppayload_size;
		return NRF24L01_write_reg(inNRF, RX_PW_P2_ADDRESS, 1, readReg.simplType);
	}
	case PIPE3:
	{
		restricAvoidType(RX_PW_P, readReg, &inNRF->global_reg_map.rx_pw_p3);
		inNRF->global_reg_map.rx_pw_p3.RX_PW_P = ppayload_size;
		return NRF24L01_write_reg(inNRF, RX_PW_P3_ADDRESS, 1, readReg.simplType);
	}
	case PIPE4:
	{
		restricAvoidType(RX_PW_P, readReg, &inNRF->global_reg_map.rx_pw_p4);
		inNRF->global_reg_map.rx_pw_p4.RX_PW_P = ppayload_size;
		return NRF24L01_write_reg(inNRF, RX_PW_P4_ADDRESS, 1, readReg.simplType);
	}
	case PIPE5:
	{
		restricAvoidType(RX_PW_P, readReg, &inNRF->global_reg_map.rx_pw_p5);
		inNRF->global_reg_map.rx_pw_p5.RX_PW_P = ppayload_size;
		return NRF24L01_write_reg(inNRF, RX_PW_P5_ADDRESS, 1, readReg.simplType);
	}
	default: return NRF_ERROR_PIPE_NUMBER;
	}
}


//---------NRF24L01_get_TX_payload-----------------
// function NRF24L01_get_TX_payload - get pipe address
// input arg:
//          - inNRF         - pointer on selected NRF24L01+
//          - pipe_number  - number of pipe, using PIPS_DEF "enum"
//          - payload_size - size of payload
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_get_RX_PayloadSize(nrfHeader inNRF, PIPS_DEF PipeNumber, uint8_t *ppayload_size){

	switch(PipeNumber)
	{
	case PIPE0:
		*ppayload_size = inNRF->global_reg_map.rx_pw_p0.RX_PW_P;
	case PIPE1:
		*ppayload_size = inNRF->global_reg_map.rx_pw_p1.RX_PW_P;
	case PIPE2:
		*ppayload_size = inNRF->global_reg_map.rx_pw_p2.RX_PW_P;
	case PIPE3:
		*ppayload_size = inNRF->global_reg_map.rx_pw_p3.RX_PW_P;
	case PIPE4:
		*ppayload_size = inNRF->global_reg_map.rx_pw_p4.RX_PW_P;
	case PIPE5:
		*ppayload_size = inNRF->global_reg_map.rx_pw_p5.RX_PW_P;
	default: return NRF_ERROR_PIPE_NUMBER;
	}
	return NRF_OK;
}


//---------NRF24L01_send_data-----------------
// function NRF24L01_send_data - send data
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
//          - data_length - number of byte to transmit
//          - p_data      - pointer on Tx data
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_send_data(nrfHeader inNRF,uint8_t data_length, uint8_t *p_data)
{
	if(data_length > MAX_NUM_BYTES_IN_RX)
	{
		return NRF_ERROR_MAX_DATA_SIZE;
	}
	NRF24L01_write_fifo_tx(inNRF, data_length,p_data); // Load Tx data on fifo_Tx buffer
	nrf24l01_ce_puls(inNRF);
	return NRF_OK;
}

//---------NRF24L01_read_rx_data-----------------
// function NRF24L01_read_fifo_rx - read NRF24L01+ fifo Rx data
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
//          - num         -  amount of read data
//          - pdata_write - pointer to data
// output arg:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_read_rx_data(nrfHeader inNRF , uint8_t num, uint8_t *pdata_read)
{
	return NRF24L01_read_fifo_rx(inNRF, num, pdata_read);
}


//---------NRF24L01_set_rx_mode-----------------
// function NRF24L01_set_rx_mode - enable RX mode: set PWR_UP and PRIM_RX bit in
//                                 CONFIG register, set CE pin
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_rx_mode(nrfHeader inNRF){
	restricAvoidType(CONFIG, readReg, &inNRF->global_reg_map.config);
	inNRF->global_reg_map.config.PRIM_RX = NRF_SET;
	NRF24L01_write_reg(inNRF, CONFIG_ADDRESS, 1, readReg.simplType);
	nrf24l01_ce_set(inNRF);
	return NRF_OK;
}

//---------NRF24L01_set_tx_mode-----------------
// function NRF24L01_set_tx_mode - enable RX mode: set PWR_UP, clear PRIM_RX bit in
//                                 CONFIG register, set CE pin
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_set_tx_mode(nrfHeader inNRF){
	nrf24l01_ce_clear(inNRF);
	restricAvoidType(CONFIG, readReg, &inNRF->global_reg_map.config);
	inNRF->global_reg_map.config.PRIM_RX = NRF_RESET;
	NRF24L01_write_reg(inNRF, CONFIG_ADDRESS, 1, readReg.simplType);
	return NRF_OK;
}




void NRF24L01_IRQ_event(nrfHeader inNRF)
{
	inNRF->isInterrupt = true;
}
