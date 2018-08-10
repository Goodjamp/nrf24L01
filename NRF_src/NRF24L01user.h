/*
 * NRF24L01user.h
 *
 *  Created on: September 6, 2018
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef NRF24L01USER_H_
#define NRF24L01USER_H_

#define MAX_NUMBER_OF_NRF 2

typedef struct nrfHeaderT *nrfHeader;

typedef enum{
              CONFIG_ADDRESS      =   (uint8_t)0x00,
              EN_AA_ADDRESS       =   (uint8_t)0x01,
              EN_RXADDR_ADDRESS   =   (uint8_t)0x02,
              SETUP_AW_ADDRESS    =   (uint8_t)0x03,
              SETUP_RETR_ADDRESS  =   (uint8_t)0x04,
              RF_CH_ADDRESS       =   (uint8_t)0x05,
              RF_SETUP_ADDRESS    =   (uint8_t)0x06,
              STATUS_ADDRESS      =   (uint8_t)0x07,
              OBSERVE_TX_ADDRESS  =   (uint8_t)0x08,
              RPD_ADDRESS         =   (uint8_t)0x09,
              RX_ADDR_P0_ADDRESS  =   (uint8_t)0x0a,
              RX_ADDR_P1_ADDRESS  =   (uint8_t)0x0b,
              RX_ADDR_P2_ADDRESS  =   (uint8_t)0x0c,
              RX_ADDR_P3_ADDRESS  =   (uint8_t)0x0d,
              RX_ADDR_P4_ADDRESS  =   (uint8_t)0x0e,
              RX_ADDR_P5_ADDRESS  =   (uint8_t)0x0f,
              TX_ADDR_ADDRESS     =   (uint8_t)0x10,
              RX_PW_P0_ADDRESS    =   (uint8_t)0x11,
              RX_PW_P1_ADDRESS    =   (uint8_t)0x12,
              RX_PW_P2_ADDRESS    =   (uint8_t)0x13,
              RX_PW_P3_ADDRESS    =   (uint8_t)0x14,
              RX_PW_P4_ADDRESS    =   (uint8_t)0x15,
              RX_PW_P5_ADDRESS    =   (uint8_t)0x16,
              FIFO_STATUS_ADDRESS =  (uint8_t)0x17,
              DYNPD_ADDRESS       =   (uint8_t)0x1C,
              FEATURE_ADDRESS     =   (uint8_t)0x1D
}NRF24L01_REG_ADDRESS;


/************************DEFINITION BIT CONFIG REGISTER************************/
typedef enum{
	STATUS_RX_DR   = (uint8_t)0b01000000,
	STATUS_TX_DS   = (uint8_t)0b00100000,
	STATUS_MAX_RT  = (uint8_t)0b00010000,
	STATUS_RX_P_NO = (uint8_t)0b00001110,
	STATUS_TX_FULL = (uint8_t)0b00000001,
}STATUS_MASK;

#define IS_INTERUPT_MASK(X) ( (~(STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT )) & X)

typedef enum{
	CRCO_1_BYTES = (uint8_t)0,
	CRCO_2_BYTES = (uint8_t)1
}CRCO;

#define IS_CRCO(X) ((X == CRCO_1_BYTES) || (X == CRCO_2_BYTES))

/************************DEFINITION BIT EN_AA REGISTER************************/
/************************DEFINITION BIT EN_RXADDR REGISTER************************/
typedef enum{
	PIPE0 = (uint8_t)0,
	PIPE1 = (uint8_t)1,
	PIPE2 = (uint8_t)2,
	PIPE3 = (uint8_t)3,
	PIPE4 = (uint8_t)4,
	PIPE5 = (uint8_t)5
}PIPS_DEF;

#define IS_PIPE(X) ((X == PIPE0) || (X == PIPE1)||\
		            (X == PIPE2) || (X == PIPE3)||\
		            (X == PIPE4))

/************************DEFINITION BIT SETUP_AW REGISTER************************/
typedef enum{
	AW_3_BYTES = (uint8_t)0b01,
	AW_4_BYTES = (uint8_t)0b10,
	AW_5_BYTES = (uint8_t)0b11
}AW;

#define IS_AW(X) ((X == AW_3_BYTES) || (X == AW_4_BYTES)||\
                  (X == AW_5_BYTES))


/************************DEFINITION BIT SETUP_RETR REGISTER************************/
typedef enum{
	WAIT_250uS  = (uint8_t)0b0000,
	WAIT_500uS  = (uint8_t)0b0001,
	WAIT_750uS  = (uint8_t)0b0010,
	WAIT_1000uS = (uint8_t)0b0011,
	WAIT_1250uS = (uint8_t)0b0100,
	WAIT_1500uS = (uint8_t)0b0101,
	WAIT_1750uS = (uint8_t)0b0110,
	WAIT_2000uS = (uint8_t)0b0111,
	WAIT_2250uS = (uint8_t)0b1000,
	WAIT_2500uS = (uint8_t)0b1001,
	WAIT_2750uS = (uint8_t)0b1010,
	WAIT_3000uS = (uint8_t)0b1011,
	WAIT_3250uS = (uint8_t)0b1100,
	WAIT_3500uS = (uint8_t)0b1101,
	WAIT_3750uS = (uint8_t)0b1110,
	WAIT_4000uS = (uint8_t)0b1111,
}RETRANSMIT_DELAY;

#define IS_RETRANSMIT_DELAY(X) ((X == WAIT_250uS)  || (X == WAIT_500uS)  || (X == WAIT_750uS)  || (X == WAIT_1000uS)||\
		                	   	(X == WAIT_1250uS) || (X == WAIT_1500uS) || (X == WAIT_1750uS) || (X == WAIT_2000uS)||\
		                	   	(X == WAIT_2250uS) || (X == WAIT_2500uS) || (X == WAIT_2750uS) || (X == WAIT_3000uS)||\
		                	   	(X == WAIT_3250uS) || (X == WAIT_3500uS) || (X == WAIT_3750uS) || (X == WAIT_4000uS))


/************************DEFINITION BIT RH_CH REGISTER************************/
/************************DEFINITION BIT RH_SETUP REGISTER*********************/
typedef enum{
	DATA_SPEED_1M   = (uint8_t)0b00,
	DATA_SPEED_2M   = (uint8_t)0b01,
	DATA_SPEED_250K = (uint8_t)0b10
}RF_DR_HIGHT;

#define IS_RF_DR_HIGHT(X) ((X == DATA_SPEED_1M) || (X == DATA_SPEED_2M)||\
		                   (X == DATA_SPEED_250K))

typedef enum{
	DATA_PWR_m18dBm =(uint8_t)0b00,
	DATA_PWR_m12dBm =(uint8_t)0b01,
	DATA_PWR_m6dBm  =(uint8_t)0b10,
	DATA_PWR_0dBm   =(uint8_t)0b11
}RF_PWR;

#define IS_RF_PWR(X) ((X == DATA_PWR_m18dBm) || (X == DATA_PWR_m12dBm)||\
		              (X == DATA_PWR_m6dBm)  || (X == DATA_PWR_0dBm))

/************************ERROR STATUS NRF24L01************************/
typedef enum{
	NRF_OK                      = (uint8_t)0,
	NRF_BUSY                    = (uint8_t)1,
	NRF_ERROR_PIPE_NUMBER       = (uint8_t)2,
	NRF_ERROR_ADDRESS_REG       = (uint8_t)3,
	NRF_ERROR_AMOUNT_NRF        = (uint8_t)4,
	NRF_ERROR_QUANTITY_RETRANSMIT =(uint8_t)5,
	NRF_ERROR_ADDRESS_WIDTH     = (uint8_t)6,
	NRF_ERROR_MAX_DATA_SIZE     = (uint8_t)7,
	NRF_ERROR_PWR_UP_STATE      = (uint8_t)8,
	NRF_ERROR_INTERUPT_MASK     = (uint8_t)9,
	NRF_ERROR_PAYLOAD_SIZE      = (uint8_t)10,
	NRF_ERROR_RETRANSMIT_DELAY  = (uint8_t)11,
	NRF_ERROR_EXCEED_RF_CHANNEL = (uint8_t)12
}NRF_ERROR;

typedef enum{
	NRF_RESET = (uint8_t)0,
	NRF_SET   = (uint8_t)1
}NRF_STATE;

#define IS_NRF_STATE(X) ((X == NRF_RESET) || (X == NRF_SET))


/************************Configuration Register************************/
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

/************************Enable ‘Auto Acknowledgment’ Function Disable************************/
typedef struct{
	uint8_t ENAA_P0:1;
	uint8_t ENAA_P1:1;
	uint8_t ENAA_P2:1;
	uint8_t ENAA_P3:1;
	uint8_t ENAA_P4:1;
	uint8_t ENAA_P5:1;
    uint8_t Reserved:2;
}EN_AA;

/************************Enabled RX Addresses************************/
typedef struct{
	uint8_t ERX_P0:1;
	uint8_t ERX_P1:1;
	uint8_t ERX_P2:1;
	uint8_t ERX_P3:1;
	uint8_t ERX_P4:1;
	uint8_t ERX_P5:1;
    uint8_t Reserved:2;
}EN_RXADDR;

/************************Setup of Address Widths ************************/
typedef struct{
	uint8_t AW:2;
    uint8_t Reserved:5;
}SETUP_AW;

/************************Setup of Automatic Retransmission************************/
typedef struct{
	uint8_t ARC:4;
    uint8_t ARD:4;
}SETUP_RETR;

/************************RF Channel selected************************/
typedef struct{
	uint8_t RF_CH:7;
    uint8_t Reserved:1;
}RF_CH;

/************************RF Setup Register************************/
typedef struct{
	uint8_t Obsolete:1;
    uint8_t RF_PWR:2;
    uint8_t RF_DR_HIGHT:1;
    uint8_t PLL_LOCK:1;
    uint8_t RF_DR_LOW:1;
    uint8_t Reserved:1;
    uint8_t CONT_WAVE:1;
}RF_SETUP;

/************************Status Register************************/
typedef struct{
	uint8_t TX_FULL:1;
    uint8_t RX_P_NO:3;
    uint8_t MAX_RT:1;
    uint8_t TX_DS:1;
    uint8_t RX_DR:1;
    uint8_t Reserved:1;
}STATUS;

/************************Transmit observe register************************/
typedef struct{
	uint8_t ARC_CNT:4;
    uint8_t PLOS_CNT:4;
}OBSERVE_TX;

/************************Received Power Detector register************************/
typedef struct{
	uint8_t RPD:1;
    uint8_t Reserved:6;
}RPD;
/************************Receive address data pipe************************/

typedef struct{
	uint32_t RX_ADDR_P;
    uint8_t  RX_ADDR_P_MSB;
}RX_ADDR_P;


/************************Transmit address************************/
typedef struct{
	uint32_t TX_ADDR;
    uint8_t  TX_ADDR_MSB;
}TX_ADDR;

/************************Number of bytes in RX payload in data pipe************************/
typedef struct{
	uint8_t RX_PW_P:6;
    uint8_t Reserved:2;
}RX_PW_P;

/************************FIFO Status Register************************/
typedef struct{
	uint8_t RX_EMPTY:1;
	uint8_t RX_FULL:1;
    uint8_t Reserved0:2;
	uint8_t TX_EMPTY:1;
	uint8_t TX_FULL:1;
	uint8_t TX_REUSE:1;
	uint8_t Reserved1:1;
}FIFO_STATUS;

/************************DYNPD Register************************/
typedef struct{
	uint8_t DPL_P0:1;
	uint8_t DPL_P1:1;
    uint8_t DPL_P2:1;
	uint8_t DPL_P3:1;
	uint8_t DPL_P4:1;
	uint8_t DPL_P5:1;
	uint8_t Reserved:2;
}DYNPD;

/************************FEATURE Register************************/
typedef struct{
	uint8_t EN_DYN_ACK:1;
	uint8_t EN_ACK_PAY:1;
    uint8_t EN_DPL:1;
	uint8_t Reserved:5;
}FEATURE;


nrfHeader NRF24L01_init(void);
NRF_ERROR NRF24L01_get_config        (nrfHeader inNRF, CONFIG *pread_configuration_reg);
NRF_ERROR NRF24L01_get_status_tx_rx  (nrfHeader inNRF, STATUS *ppread_satus_reg);
NRF_ERROR NRF24L01_get_status_fifo   (nrfHeader inNRF, FIFO_STATUS *ppread_satus_reg);
NRF_ERROR NRF24L01_get_rx_pipe_size  (nrfHeader inNRF, uint8_t *rx_size);
NRF_ERROR NRF24L01_get_fifo_status   (nrfHeader inNRF, uint8_t *pread_status_reg);
NRF_ERROR NRF24L01_clear_interrupt   (nrfHeader inNRF, STATUS_MASK clear_interrupt_flag);
NRF_ERROR NRF24L01_power_switch      (nrfHeader inNRF, NRF_STATE new_pwr_up_state);
NRF_ERROR NRF24L01_enable_crc        (nrfHeader inNRF, NRF_STATE new_crc_state);
NRF_ERROR NRF24L01_set_crco          (nrfHeader inNRF, CRCO new_crco);
NRF_ERROR NRF24L01_set_interrupt     (nrfHeader inNRF, STATUS_MASK set_interupt);
NRF_ERROR NRF24L01_enable_AA         (nrfHeader inNRF, PIPS_DEF PipeNumber);
NRF_ERROR NRF24L01_enable_pipe       (nrfHeader inNRF, PIPS_DEF PipeNumber);
NRF_ERROR NRF24L01_set_num_retr      (nrfHeader inNRF, uint8_t num_retransmit);
NRF_ERROR NRF24L01_set_delay_retr    (nrfHeader inNRF, RETRANSMIT_DELAY auto_retr_delay);
NRF_ERROR NRF24L01_set_address_width (nrfHeader inNRF, AW  address_width);
NRF_ERROR NRF24L01_set_RX_address    (nrfHeader inNRF, PIPS_DEF PipeNumber, uint8_t *pPipeAddress);
NRF_ERROR NRF24L01_get_RX_address    (nrfHeader inNRF, PIPS_DEF PipeNumber, uint8_t *pPipeAddress);
NRF_ERROR NRF24L01_set_TX_addres     (nrfHeader inNRF, uint8_t *pPipeAddress);
NRF_ERROR NRF24L01_get_TX_addres     (nrfHeader inNRF, uint8_t *pPipeAddress);
NRF_ERROR NRF24L01_set_TX_PayloadSize(nrfHeader inNRF, PIPS_DEF PipeNumber,uint8_t *ppayload_size);
NRF_ERROR NRF24L01_get_TX_PayloadSize(nrfHeader inNRF, PIPS_DEF PipeNumber,uint8_t *ppayload_size);
NRF_ERROR NRF24L01_set_rx_mode       (nrfHeader inNRF);
NRF_ERROR NRF24L01_set_tx_mode       (nrfHeader inNRF);
NRF_ERROR NRF24L01_send_data         (nrfHeader inNRF, uint8_t data_length, uint8_t *p_data);
NRF_ERROR NRF24L01_read_rx_data      (nrfHeader inNRF , uint8_t num, uint8_t *pdata_read);

/*************************USER SHOULD CALL IN CASE OF IRQ EVENT**********************/
void      NRF24L01_set_interupt      (nrfHeader inNRF);

/*************************USER IMPLEMENTATION FUNCTION*******************************/
typedef enum{
	STATE_OK   = 0,
	STATE_BUSY = 1,
}STATE;

STATE nrf24l01_spi_TX  (nrfHeader inNRF, uint8_t comand, uint8_t *p_data, uint8_t length);
STATE nrf24l01_spi_RX  (nrfHeader inNRF, uint8_t comand, uint8_t *p_data, uint8_t length);
void  nrf24l01_ce_set  (nrfHeader inNRF);
void  nrf24l01_ce_clear(nrfHeader inNRF);
void  nrf24l01_ce_puls (nrfHeader inNRF);

#endif
