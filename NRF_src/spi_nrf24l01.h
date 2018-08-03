/*
 * spi_nrf24l01.h
 *
 *  Created on: January 28, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef SPI_NRF24L01_H_
#define SPI_NRF24L01_H_

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

//-----maximum amount of NRF24L01+ in system
#define MAX_NUM_NRF   4

#define BUFF_LIFO_SIZE    50


#define NRF_CE_SET(X)   GPIO_SetBits(X->peref.CE_port,X->peref.CE_pin)
#define NRF_CE_RESET(X) GPIO_ResetBits(X->peref.CE_port,X->peref.CE_pin)
#define NRF_CE_PULSE(X) NRF_CE_SET(X);\
	                    asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						asm("nop");\
						NRF_CE_RESET(X);

#define SPI_CSN_SET(X)   GPIO_SetBits(X->peref.CSN_port,X->peref.CSN_pin)
#define SPI_CSN_RESET(X) GPIO_ResetBits(X->peref.CSN_port,X->peref.CSN_pin)

#define NRF_DEBUG_SET   GPIOA->ODR|=GPIO_ODR_ODR0;
#define NRF_DEBUG_RESET GPIOA->ODR&=~(GPIO_ODR_ODR0);

//============================ENABLE SPI INTERRRUPT============================
#define ENABLE_TX_RX_INTERUPT(X)    X->CR2|=(SPI_CR2_TXEIE |SPI_CR2_RXNEIE);

#define ENABLE_TX_INTERUPT(X)       X->CR2|=(SPI_CR2_TXEIE);
#define ENABLE_RX_INTERUPT(X)       X->CR2|=(SPI_CR2_RXNEIE);
//============================DISABLE SPI INTERRRUPT============================
#define DISABLE_TX_RX_INTERUPT(X)   X->CR2&=~(SPI_CR2_TXEIE |SPI_CR2_RXNEIE);

#define DISABLE_TX_INTERUPT(X)      X->CR2&=~(SPI_CR2_TXEIE);
#define DISABLE_RX_INTERUPT(X)      X->CR2&=~(SPI_CR2_RXNEIE);
//====================================================================================


/*
//============================ENABLE SPI INTERRRUPT====================================
#define ENABLE_TX_RX_INTERUPT(X)    SPI_I2S_ITConfig(X,(u8)SPI_I2S_IT_RXNE,ENABLE);\
                                    SPI_I2S_ITConfig(X,(u8)SPI_I2S_IT_TXE,ENABLE);
#define ENABLE_TX_INTERUPT(X)       SPI_I2S_ITConfig(X,(u8)SPI_I2S_IT_TXE,ENABLE);
#define ENABLE_RX_INTERUPT(X)       SPI_I2S_ITConfig(X,(u8)SPI_I2S_IT_RXNE,ENABLE);
//============================DISABLE SPI INTERRRUPT============================
#define DISABLE_TX_RX_INTERUPT(X)   SPI_I2S_ITConfig(X,(u8)SPI_I2S_IT_RXNE,DISABLE);\
                                    SPI_I2S_ITConfig(X,(u8)SPI_I2S_IT_TXE,DISABLE);
#define DISABLE_TX_INTERUPT(X)      SPI_I2S_ITConfig(X,(u8)SPI_I2S_IT_TXE,DISABLE);
#define DISABLE_RX_INTERUPT(X)      SPI_I2S_ITConfig(X,(u8)SPI_I2S_IT_RXNE,DISABLE);
//====================================================================================
*/

typedef enum{
	OK_STATE=0,
	BUSY_STATE=1
}STATE;

typedef enum{
	FREE_MODE=0,
	RX_MODE,
	TX_MODE
}TEMP_RXTX_MODE;

typedef enum{
	PER_ERROR_OK=0,
	PER_ERROR_CE_PIN,
	PER_ERROR_IRQ_PIN,
	PER_ERROR_CSN_PIN,
	PER_ERROR_CE_PORT,
	PER_ERROR_IRQ_PORT,
	PER_ERROR_CSN_PORT
}PER_ERROR;



typedef struct{
	u8 *p_buff_rx;
	u8 buff_tx[BUFF_LIFO_SIZE];
	u8 counter_tx;
	u8 counter_rx;
	u8 num;
}S_buf_rx_rx;

typedef struct{
	u8 f_interrupt;
	S_buf_rx_rx buf;
	TEMP_RXTX_MODE tx_rx_mode;
}S_buf_spi;


typedef struct{
	GPIO_TypeDef *CE_port;
	GPIO_TypeDef *CSN_port;
	GPIO_TypeDef *IRQ_port;
	u16 CE_pin;
	u16 CSN_pin;
	u16 IRQ_pin;
	SPI_TypeDef *SPI;
} S_peref;

typedef struct{
	u8 f_interrupt;
	S_buf_spi buf_g;
	S_peref peref;
}S_nrf_config;



u8 mcu_nrf_init( S_nrf_config *const pNRF);
void init_spi(volatile  S_nrf_config *p_nrf);
S_nrf_config* processing_init(const S_peref *per);
PER_ERROR init_gpio(const S_nrf_config *pnrf_gpio);
STATE spi_nrf24l01_TX(S_nrf_config *p_nrf,u8 comand, const u8 *p_data, u8 length);
STATE spi_nrf24l01_RX(S_nrf_config *p_nrf,u8 comand, u8 *p_data, u8 length);
STATE spi_nrf24l01_get_state(S_nrf_config *p_nrf);
void remap_out_pin(void);
u8 rcc_gpio_enable(const GPIO_TypeDef *gpio_enable);


#endif
