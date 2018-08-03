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

#include "moduleHWInit.h"

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
typedef enum{
              CONFIG_ADDRESS    =   (uint8_t)0x00,
              EN_AA_ADDRESS     =   (uint8_t)0x01,
              EN_RXADDR_ADDRESS =   (uint8_t)0x02,
              SETUP_AW_ADDRESS  =   (uint8_t)0x03,
              SETUP_RETR_ADDRESS=   (uint8_t)0x04,
              RF_CH_ADDRESS     =   (uint8_t)0x05,
              RF_SETUP_ADDRESS  =   (uint8_t)0x06,
              STATUS_ADDRESS    =   (uint8_t)0x07,
              OBSERVE_TX_ADDRESS=   (uint8_t)0x08,
              RPD_ADDRESS       =   (uint8_t)0x09,
              RX_ADDR_P0_ADDRESS=   (uint8_t)0x0a,
              RX_ADDR_P1_ADDRESS=   (uint8_t)0x0b,
              RX_ADDR_P2_ADDRESS=   (uint8_t)0x0c,
              RX_ADDR_P3_ADDRESS=   (uint8_t)0x0d,
              RX_ADDR_P4_ADDRESS=   (uint8_t)0x0e,
              RX_ADDR_P5_ADDRESS=   (uint8_t)0x0f,
              TX_ADDR_ADDRESS   =   (uint8_t)0x10,
              RX_PW_P0_ADDRESS  =   (uint8_t)0x11,
              RX_PW_P1_ADDRESS  =   (uint8_t)0x12,
              RX_PW_P2_ADDRESS  =   (uint8_t)0x13,
              RX_PW_P3_ADDRESS  =   (uint8_t)0x14,
              RX_PW_P4_ADDRESS  =   (uint8_t)0x15,
              RX_PW_P5_ADDRESS  =   (uint8_t)0x16,
              FIFO_STATUS_ADDRESS=  (uint8_t)0x17,
              DYNPD_ADDRESS     =   (uint8_t)0x1C,
              FEATURE_ADDRESS   =   (uint8_t)0x1D
}NRF24L01_REG_ADDRESS;

#define CHECK_NRF_ADDRESS(X)  ((X==CONFIG_ADDRESS)||(X==EN_AA_ADDRESS)||(X==EN_RXADDR_ADDRESS)||(X==SETUP_AW_ADDRESS)||\
							   (X==SETUP_RETR_ADDRESS)||(X==RF_CH_ADDRESS)||(X==RF_SETUP_ADDRESS)||(X==STATUS_ADDRESS)||\
							   (X==OBSERVE_TX_ADDRESS)||(X==RPD_ADDRESS)||(X==RX_ADDR_P0_ADDRESS)||(X==RX_ADDR_P1_ADDRESS)||\
							   (X==RX_ADDR_P2_ADDRESS)||(X==RX_ADDR_P3_ADDRESS)||(X==RX_ADDR_P4_ADDRESS)||(X==RX_ADDR_P5_ADDRESS)||\
							   (X==TX_ADDR_ADDRESS)||(X==RX_PW_P0_ADDRESS)||(X==RX_PW_P1_ADDRESS)||(X==RX_PW_P2_ADDRESS)||\
							   (X==RX_PW_P3_ADDRESS)||(X==RX_PW_P4_ADDRESS)||(X==RX_PW_P5_ADDRESS)||(X==FIFO_STATUS_ADDRESS))

// ------SET/RESET NRF state definition------------
typedef enum{
	NRF_RESET = (uint8_t)0,
	NRF_SET   = (uint8_t)1
}NRF_STATE;

#define IS_NRF_STATE(X) ((X==NRF_RESET)||(X==NRF_SET))

//===================DEFINITION BIT CONFIG REGISTER=============================//
typedef enum{
	MASK_RX_DR  =(uint8_t)0b01000000,
	MASK_TX_DS  =(uint8_t)0b00100000,
	MASK_MAX_RT =(uint8_t)0b00010000
}INTERUPT_MASK;

#define IS_INTERUPT_MASK(X) ( (~(MASK_RX_DR|MASK_TX_DS|MASK_MAX_RT)) & X)

typedef enum{
	CRCO_1_BYTES=(uint8_t)0,
	CRCO_2_BYTES=(uint8_t)1
}CRCO;

#define IS_CRCO(X) ((X==CRCO_1_BYTES)||(X==CRCO_2_BYTES))

//===================DEFINITION BIT EN_AA REGISTER=================================//
//===================DEFINITION BIT EN_RXADDR REGISTER=============================//
// -----------pipes definition-------------------------
typedef enum{
	PIPE0=(uint8_t)0,
	PIPE1=(uint8_t)1,
	PIPE2=(uint8_t)2,
	PIPE3=(uint8_t)3,
	PIPE4=(uint8_t)4,
	PIPE5=(uint8_t)5
}PIPS_DEF;

#define IS_PIPE(X) ((X==PIPE0)||(X==PIPE1)||\
		            (X==PIPE2)||(X==PIPE3)||\
		            (X==PIPE4))

//===================DEFINITION BIT SETUP_AW REGISTER=============================//
//---------------RX/TX Address field width DEFINITION----------------------------
typedef enum{
	AW_3_BYTES=(uint8_t)0b01,
	AW_4_BYTES=(uint8_t)0b10,
	AW_5_BYTES=(uint8_t)0b11
}AW;

#define IS_AW(X) ((X==AW_3_BYTES)||(X==AW_4_BYTES)||\
                 (X==AW_5_BYTES))


//===================DEFINITION BIT SETUP_RETR REGISTER=============================//
//-------------------Auto Retransmit Delay definition-------------------------------
typedef enum{
	WAIT_250uS=(uint8_t)0b0000,
	WAIT_500uS=(uint8_t)0b0001,
	WAIT_750uS=(uint8_t)0b0010,
	WAIT_1000uS=(uint8_t)0b0011,
	WAIT_1250uS=(uint8_t)0b0100,
	WAIT_1500uS=(uint8_t)0b0101,
	WAIT_1750uS=(uint8_t)0b0110,
	WAIT_2000uS=(uint8_t)0b0111,
	WAIT_2250uS=(uint8_t)0b1000,
	WAIT_2500uS=(uint8_t)0b1001,
	WAIT_2750uS=(uint8_t)0b1010,
	WAIT_3000uS=(uint8_t)0b1011,
	WAIT_3250uS=(uint8_t)0b1100,
	WAIT_3500uS=(uint8_t)0b1101,
	WAIT_3750uS=(uint8_t)0b1110,
	WAIT_4000uS=(uint8_t)0b1111,
}RETRANSMIT_DELAY;

#define IS_RETRANSMIT_DELAY(X) ((X==WAIT_250uS)||(X==WAIT_500uS)||(X==WAIT_750uS)||(X==WAIT_1000uS)||\
		               	   	   (X==WAIT_1250uS)||(X==WAIT_1500uS)||(X==WAIT_1750uS)||(X==WAIT_2000uS)||\
		               	   	   (X==WAIT_2250uS)||(X==WAIT_2500uS)||(X==WAIT_2750uS)||(X==WAIT_3000uS)||\
		               	   	   (X==WAIT_3250uS)||(X==WAIT_3500uS)||(X==WAIT_3750uS)||(X==WAIT_4000uS))


//===================DEFINITION BIT RH_CH REGISTER==================================//

//===================DEFINITION BIT RH_SETUP REGISTER===============================//
//--------------Select between the high speed data rates DEFINITION--------------
typedef enum{
	DATA_SPEED_1M   =(uint8_t)0b00,
	DATA_SPEED_2M   =(uint8_t)0b01,
	DATA_SPEED_250K =(uint8_t)0b10
}RF_DR_HIGHT;

#define IS_RF_DR_HIGHT(X) ((X==DATA_SPEED_1M)||(X==DATA_SPEED_2M)||\
		                  (X==DATA_SPEED_250K))

//-------------Set RF output power in TX mode  DEFINITION------------------------
typedef enum{
	DATA_PWR_m18dBm =(uint8_t)0b00,
	DATA_PWR_m12dBm =(uint8_t)0b01,
	DATA_PWR_m6dBm  =(uint8_t)0b10,
	DATA_PWR_0dBm   =(uint8_t)0b11
}RF_PWR;

#define IS_RF_PWR(X) ((X==DATA_PWR_m18dBm)||(X==DATA_PWR_m12dBm)||\
		             (X==DATA_PWR_m6dBm)||(X==DATA_PWR_0dBm))


//======================ERROR SYAYUS NRF24L01=========================================
typedef enum{
	NRF_ERROR_OK            =(uint8_t)0,
	NRF_ERROR_BUSY          =(uint8_t)1,
	NRF_ERROR_PIPE_NUMBER   =(uint8_t)2,
	NRF_ERROR_ADDRESS_REG   =(uint8_t)3,
	NRF_ERROR_AMOUNT_NRF    =(uint8_t)4,
	NRF_ERROR_EXCEED_QUANTITY_RETRANSMIT =(uint8_t)5,
	NRF_ERROR_ADDRESS_WIDTH =(uint8_t)6,
	NRF_ERROR_MAX_DATA_SIZE =(uint8_t)7,
	NRF_ERROR_PWR_UP_STATE  =(uint8_t)8,
	NRF_ERROR_INTERUPT_MASK =(uint8_t)9,
	NRF_ERROR_PAYLOAD_SIZE  =(uint8_t)10,
	NRF_ERROR_RETRANSMIT_DELAY =(uint8_t)11,
	NRF_ERROR_EXCEED_RF_CHANNEL =(uint8_t)12
}NRF_ERROR;

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
	uint8_t RF_CH:2;
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
	NRF_STATE crc_state;
	CRCO      crc_width;
	AW        address_width;
	uint8_t        auto_retransmit_delay;
	uint8_t        auto_retransmit_count;  // 0 - retransmit disabled
	uint8_t        rf_chanel;              // radio frequency channel number 0-127 1 mHz resolution
	RF_DR_HIGHT speed;
	RF_PWR    out_amplifare;
	uint8_t        transmit_address[5];
	NRF_STATE dynamic_payload_state;
}S_NRF_Init;

//----------Init pipe struct-----------------
//selected pipe enable automatically
typedef struct{
	PIPS_DEF  num_pipe;
	NRF_STATE auto_ack_state;
	NRF_STATE dunamic_payload_state;
	uint8_t        size_payload;
	uint8_t        address_pipe_rx[5];
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



NRF_ERROR NRF24L01_read_reg(S_nrf_config *const pNRF ,NRF24L01_REG_ADDRESS address_reg, uint8_t num, uint8_t *pdata_read);
NRF_ERROR NRF24L01_write_reg(S_nrf_config *const pNRF ,NRF24L01_REG_ADDRESS address_reg, uint8_t num,const uint8_t *pdata_write);
NRF_ERROR NRF24L01_write_fifo_tx(S_nrf_config *const pNRF , uint8_t num,const uint8_t *pdata_write);
NRF_ERROR NRF24L01_read_fifo_rx(S_nrf_config *const pNRF , uint8_t num,uint8_t *pdata_read);
NRF_ERROR NRF24L01_FLUSH_TX(S_nrf_config *const pNRF);
NRF_ERROR NRF24L01_FLUSH_RX(S_nrf_config *const pNRF);


NRF_ERROR NRF24L01_init_mcu(S_nrf_config *const pNRF);
NRF_ERROR NRF24L01_get_config(S_nrf_config *const pNRF, uint8_t *pread_status_reg);
NRF_ERROR NRF24L01_get_status(S_nrf_config *const pNRF, uint8_t *ppread_status_reg);
NRF_ERROR NRF24L01_reset_status_interrupt(S_nrf_config *const pNRF,INTERUPT_MASK clear_interrupt_flag);
NRF_ERROR NRF24L01_get_fifo_status(S_nrf_config *const pNRF, uint8_t *pread_status_reg);
NRF_ERROR NRF24L01_power_switch(S_nrf_config *const pNRF, NRF_STATE new_pwr_up_state);
NRF_ERROR NRF24L01_enable_crc(S_nrf_config *const pNRF, NRF_STATE new_crc_state);
NRF_ERROR NRF24L01_set_crco(S_nrf_config *const pNRF, CRCO new_crco);
NRF_ERROR NRF24L01_set_interrupt(S_nrf_config *const pNRF, INTERUPT_MASK set_interupt);
NRF_ERROR NRF24L01_enable_AA(S_nrf_config *const pNRF, PIPS_DEF PipeNumber);
NRF_ERROR NRF24L01_enable_pipe(S_nrf_config *const pNRF, PIPS_DEF PipeNumber);
NRF_ERROR NRF24L01_set_address_width(S_nrf_config *const pNRF, AW  address_width);
NRF_ERROR NRF24L01_set_num_retransmit(S_nrf_config *const pNRF, uint8_t  num_retransmit);
NRF_ERROR NRF24L01_set_delay_retransmit(S_nrf_config *const pNRF, RETRANSMIT_DELAY  auto_retr_delay);
NRF_ERROR NRF24L01_set_RX_address(S_nrf_config *const pNRF, PIPS_DEF PipeNumber, uint8_t *pPipeAddress);
NRF_ERROR NRF24L01_get_RX_address(S_nrf_config *const pNRF, PIPS_DEF PipeNumber, uint8_t *pPipeAddress);
NRF_ERROR NRF24L01_set_TX_addres(S_nrf_config *const pNRF,uint8_t *pPipeAddress);
NRF_ERROR NRF24L01_get_TX_PayloadSize(S_nrf_config *const pNRF,PIPS_DEF PipeNumber,uint8_t *ppayload_size);
NRF_ERROR NRF24L01_set_TX_PayloadSize(S_nrf_config *const pNRF,PIPS_DEF PipeNumber,uint8_t *ppayload_size);
NRF_ERROR NRF24L01_get_TX_addres(S_nrf_config *const pNRF,uint8_t *pPipeAddress);
NRF_ERROR NRF24L01_set_rx_mode(S_nrf_config *const pNRF);
NRF_ERROR NRF24L01_set_tx_mode(S_nrf_config *const pNRF);
NRF_ERROR NRF24L01_send_data(S_nrf_config *const pNRF,uint8_t data_length, uint8_t const *p_data);

#endif
