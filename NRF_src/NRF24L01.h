/*
 * NRF24L01.h
 *
 *  Created on: January 25, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef NRF24L01_H_
#define  NRF24L01_H_
#include "stdint.h"
#include "stdbool.h"
#include "NRF24L01user.h"

typedef struct nrfHeaderT
{
    bool isInterrupt;
}nrfHeaderT;

// --------------definitions property of NRF24L01+ ------------------------
#define MAX_NUMBER_RF_CHANEL        (uint8_t)125
#define MAX_QUANTITY_RETRANSMIT     (uint8_t)0b1111
#define MAX_NUM_BYTES_IN_RX         (uint8_t)32
#define MAX_PAYLOAD_SIZE            (uint8_t)32

//----------------------SPI command----------------------------------------
typedef enum{
      R_REGISTER          =   (uint8_t)0b00000000,
      W_REGISTER          =   (uint8_t)0b00100000,
      R_RX_PAYLOAD        =   (uint8_t)0b01100001,
      W_TX_PAYLOAD        =   (uint8_t)0b10100000,
      FLUSH_TX            =   (uint8_t)0b11100001,
      FLUSH_RX            =   (uint8_t)0b11100010,
      REUSE_TX_PL         =   (uint8_t)0b11100011,
      R_RX_PL_WID         =   (uint8_t)0b01100000,
      W_ACK_PAYLOAD       =   (uint8_t)0b10101000,
      W_TX_PAYLOAD_NO_ACK =   (uint8_t)0b10110000,
      NOP                 =   (uint8_t)0b11111111
}NRF24L01_COMAND;
//----------------------Registers Address----------------------------------
#define CHECK_NRF_ADDRESS(X)  ((X==CONFIG_ADDRESS)||(X==EN_AA_ADDRESS)||(X==EN_RXADDR_ADDRESS)||(X==SETUP_AW_ADDRESS)||\
							   (X==SETUP_RETR_ADDRESS)||(X==RF_CH_ADDRESS)||(X==RF_SETUP_ADDRESS)||(X==STATUS_ADDRESS)||\
							   (X==OBSERVE_TX_ADDRESS)||(X==RPD_ADDRESS)||(X==RX_ADDR_P0_ADDRESS)||(X==RX_ADDR_P1_ADDRESS)||\
							   (X==RX_ADDR_P2_ADDRESS)||(X==RX_ADDR_P3_ADDRESS)||(X==RX_ADDR_P4_ADDRESS)||(X==RX_ADDR_P5_ADDRESS)||\
							   (X==TX_ADDR_ADDRESS)||(X==RX_PW_P0_ADDRESS)||(X==RX_PW_P1_ADDRESS)||(X==RX_PW_P2_ADDRESS)||\
							   (X==RX_PW_P3_ADDRESS)||(X==RX_PW_P4_ADDRESS)||(X==RX_PW_P5_ADDRESS)||(X==FIFO_STATUS_ADDRESS))


#pragma pack(push, 1)

//---------------------global reg map---------------------------------------------
typedef struct{
	CONFIG      config;
	EN_AA       en_aa;
	EN_RXADDR   en_rxaddr;
	SETUP_AW    setup_aw;
	SETUP_RETR  setup_petr;
	RF_CH       rf_ch;
	RF_SETUP    rf_setup;
	STATUS      status;
	OBSERVE_TX  observe_tx;
	RPD         rpd;
	RX_ADDR_P   rx_addr_p0;
	RX_ADDR_P   rx_addr_p1;
	RX_ADDR_P   rx_addr_p2;
	RX_ADDR_P   rx_addr_p3;
	RX_ADDR_P   rx_addr_p4;
	RX_ADDR_P   rx_addr_p5;
	TX_ADDR     tx_addr;
	RX_PW_P     rx_pw_p0;
	RX_PW_P     rx_pw_p1;
	RX_PW_P     rx_pw_p2;
	RX_PW_P     rx_pw_p3;
	RX_PW_P     rx_pw_p4;
	RX_PW_P     rx_pw_p5;
	FIFO_STATUS fifo_status;
}S_GLOBAL_REG_MAP;

//----------Init pipe struct-----------------
typedef struct{
	NRF_STATE   crc_state;
	CRCO        crc_width;
	AW          address_width;
	uint8_t     auto_retransmit_delay;
	uint8_t     auto_retransmit_count;  // 0 - retransmit disabled
	uint8_t     rf_chanel;              // radio frequency channel number 0-127 1 mHz resolution
	RF_DR_HIGHT speed;
	RF_PWR      out_amplifare;
	uint8_t     transmit_address[5];
	NRF_STATE   dynamic_payload_state;
}S_NRF_Init;

//----------Init pipe struct-----------------
//selected pipe enable automatically
typedef struct{
	PIPS_DEF  num_pipe;
	NRF_STATE auto_ack_state;
	NRF_STATE dunamic_payload_state;
	uint8_t   size_payload;
	uint8_t   address_pipe_rx[5];
}S_NRF_Pipe_Init;


#pragma pack(pop)

// --------DEFAULT INIT PARAMITERS----------------
#define  CRC_STATE_DEFAULT              NRF_SET
#define  CRC_SIZE_DEFAULT               CRCO_2_BYTES
#define  ADDRESS_SIZE_DEFAULT           AW_5_BYTES
#define  AUTO_RETR_DELAY_DEFAULT        15
#define  AUTO_RETR_COUNT_DEFAULT        15
#define  RF_CHANEL_DEFAULT              0b10
#define  SPEED_DEFAULT                  DATA_SPEED_2M
#define  OUT_AMPLIFARE_DEFAULT          DATA_PWR_0dBm
#define  TX_ADDRESS_DEFAULT(X)          X[0]=0xE7;X[1]=0xE7;X[2]=0xE7;X[3]=0xE7;X[4]=0xE7;
#define  DYNAMIC_PAYLOAD_STATE_DEFAULT  NRF_RESET


NRF_ERROR NRF24L01_read_reg     (nrfHeader inNRF, NRF24L01_REG_ADDRESS address_reg, uint8_t num, uint8_t *pdata_read);
NRF_ERROR NRF24L01_write_reg    (nrfHeader inNRF, NRF24L01_REG_ADDRESS address_reg, uint8_t num, uint8_t *pdata_write);
NRF_ERROR NRF24L01_write_fifo_tx(nrfHeader inNRF, uint8_t num, uint8_t *pdata_write);
NRF_ERROR NRF24L01_read_fifo_rx (nrfHeader inNRF, uint8_t num, uint8_t *pdata_read);
NRF_ERROR NRF24L01_FLUSH_TX     (nrfHeader inNRF);
NRF_ERROR NRF24L01_FLUSH_RX     (nrfHeader inNRF);

#endif
