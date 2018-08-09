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

//---------------------Configuration Register-----------------------------------
typedef struct{
	uint8_t PRIM_RX:1;
	uint8_t PWR_UP:1;
	uint8_t CRCO:1;
	uint8_t EN_CRC:1;
	uint8_t MASK_MAX_RT:1;
	uint8_t MASK_T_DS:1;
	uint8_t MASK_RX_DR:1;
    uint8_t Reserved:1;
}CONFIG;

//---------------------Enable ‘Auto Acknowledgment’ Function Disable-------------
typedef struct{
	uint8_t ENAA_P0:1;
	uint8_t ENAA_P1:1;
	uint8_t ENAA_P2:1;
	uint8_t ENAA_P3:1;
	uint8_t ENAA_P4:1;
	uint8_t ENAA_P5:1;
    uint8_t Reserved:2;
}EN_AA;

//---------------------Enabled RX Addresses--------------------------------------
typedef struct{
	uint8_t ERX_P0:1;
	uint8_t ERX_P1:1;
	uint8_t ERX_P2:1;
	uint8_t ERX_P3:1;
	uint8_t ERX_P4:1;
	uint8_t ERX_P5:1;
    uint8_t Reserved:2;
}EN_RXADDR;

//---------------------Setup of Address Widths (common for all data pipes)------
typedef struct{
	uint8_t AW:2;
    uint8_t Reserved:5;
}SETUP_AW;

//---------------------Setup of Automatic Retransmission-------------------------
typedef struct{
	uint8_t ARC:4;
    uint8_t ARD:4;
}SETUP_RETR;

//---------------------RF Channel selekted---------------------------------------
typedef struct{
	uint8_t RF_CH:7;
    uint8_t Reserved:1;
}RF_CH;

//---------------------RF Setup Register-----------------------------------------
typedef struct{
	uint8_t Obsolete:1;
    uint8_t RF_PWR:2;
    uint8_t RF_DR_HIGHT:1;
    uint8_t PLL_LOCK:1;
    uint8_t RF_DR_LOW:1;
    uint8_t Reserved:1;
    uint8_t CONT_WAVE:1;
}RF_SETUP;

//---------------------Status Register------------------------------------------
typedef struct{
	uint8_t TX_FULL:1;
    uint8_t RX_P_NO:3;
    uint8_t MAX_RT:1;
    uint8_t TX_DS:1;
    uint8_t RX_DR:1;
    uint8_t Reserved:1;
}STATUS;

//---------------------Transmit observe register--------------------------------
typedef struct{
	uint8_t ARC_CNT:4;
    uint8_t PLOS_CNT:4;
}OBSERVE_TX;

//---------------------Received Power Detector register-------------------------
typedef struct{
	uint8_t RPD:1;
    uint8_t Reserved:6;
}RPD;
//---------------------Receive address data pipe 0 register. 5 Bytes maximum length---------------------

typedef struct{
	uint32_t RX_ADDR_P0;
    uint8_t  RX_ADDR_P0_MSB;
}RX_ADDR_P0;

//---------------------Receive address data pipe 1 register. 5 Bytes maximum length--------------------
typedef struct{
	uint32_t RX_ADDR_P1;
    uint8_t  RX_ADDR_P1_MSB;
}RX_ADDR_P1;

//---------------------Receive address data pipe 2. Only LSB. MSBytes are equal to RX_ADDR_P1-----------
typedef struct{
	uint32_t RX_ADDR_P2;
    uint8_t  RX_ADDR_P2_MSB;
}RX_ADDR_P2;

//---------------------Receive address data pipe 3. Only LSB. MSBytes are equal to RX_ADDR_P1-----------
typedef struct{
	uint32_t RX_ADDR_P3;
    uint8_t  RX_ADDR_P3_MSB;
}RX_ADDR_P3;

//---------------------Receive address data pipe 4. Only LSB. MSBytes are equal to RX_ADDR_P1-----------
typedef struct{
	uint32_t RX_ADDR_P4;
    uint8_t  RX_ADDR_P4_MSB;
}RX_ADDR_P4;

//---------------------Receive address data pipe 5. Only LSB. MSBytes are equal to RX_ADDR_P1-----------
typedef struct{
	uint32_t RX_ADDR_P5;
    uint8_t  RX_ADDR_P5_MSB;
}RX_ADDR_P5;

//---------------------Transmit address. Used for a PTX device only--------------------
typedef struct{
	uint32_t TX_ADDR;
    uint8_t  TX_ADDR_MSB;
}TX_ADDR;

//---------------------Number of bytes in RX payload in data pipe 0 register-----------
typedef struct{
	uint8_t RX_PW_P0:6;
    uint8_t Reserved:2;
}RX_PW_P0;

//---------------------Number of bytes in RX payload in data pipe 1 register-----------
typedef struct{
	uint8_t RX_PW_P1:6;
    uint8_t Reserved:2;
}RX_PW_P1;

//---------------------Number of bytes in RX payload in data pipe 2 register-----------
typedef struct{
	uint8_t RX_PW_P2:6;
    uint8_t Reserved:2;
}RX_PW_P2;

//---------------------Number of bytes in RX payload in data pipe 3 register-----------
typedef struct{
	uint8_t RX_PW_P3:6;
    uint8_t Reserved:2;
}RX_PW_P3;

//---------------------Number of bytes in RX payload in data pipe 4 register-----------
typedef struct{
	uint8_t RX_PW_P4:6;
    uint8_t Reserved:2;
}RX_PW_P4;

//---------------------Number of bytes in RX payload in data pipe 5 register-----------
typedef struct{
	uint8_t RX_PW_P5:6;
    uint8_t Reserved:2;
}RX_PW_P5;

//---------------------FIFO Status Register---------------------------------------------
typedef struct{
	uint8_t RX_EMPTY:1;
	uint8_t RX_FULL:1;
    uint8_t Reserved0:2;
	uint8_t TX_EMPTY:1;
	uint8_t TX_FULL:1;
	uint8_t TX_REUSE:1;
	uint8_t Reserved1:1;
}FIFO_STATUS;

//---------------------DYNPD Register---------------------------------------------
typedef struct{
	uint8_t DPL_P0:1;
	uint8_t DPL_P1:1;
    uint8_t DPL_P2:1;
	uint8_t DPL_P3:1;
	uint8_t DPL_P4:1;
	uint8_t DPL_P5:1;
	uint8_t Reserved:2;
}DYNPD;

//---------------------FEATURE Register---------------------------------------------
typedef struct{
	uint8_t EN_DYN_ACK:1;
	uint8_t EN_ACK_PAY:1;
    uint8_t EN_DPL:1;
	uint8_t Reserved:5;
}FEATURE;

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
	RX_ADDR_P0  rx_addr_p0;
	RX_ADDR_P1  rx_addr_p1;
	RX_ADDR_P2  rx_addr_p2;
	RX_ADDR_P3  rx_addr_p3;
	RX_ADDR_P4  rx_addr_p4;
	RX_ADDR_P5  rx_addr_p5;
	TX_ADDR     tx_addr;
	RX_PW_P0    rx_pw_p0;
	RX_PW_P1    rx_pw_p1;
	RX_PW_P2    rx_pw_p2;
	RX_PW_P3    rx_pw_p3;
	RX_PW_P4    rx_pw_p4;
	RX_PW_P5    rx_pw_p5;
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
