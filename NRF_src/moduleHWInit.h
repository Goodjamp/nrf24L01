/*
 * spi_nrf24l01.h
 *
 *  Created on: January 28, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef SPI_NRF24L01_H_
#define SPI_NRF24L01_H_
#include "stdint.h"

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
#define ENABLE_TX_RX_INTERUPT(X)    SPI_I2S_ITConfig(X,(uint8_t)SPI_I2S_IT_RXNE,ENABLE);\
                                    SPI_I2S_ITConfig(X,(uint8_t)SPI_I2S_IT_TXE,ENABLE);
#define ENABLE_TX_INTERUPT(X)       SPI_I2S_ITConfig(X,(uint8_t)SPI_I2S_IT_TXE,ENABLE);
#define ENABLE_RX_INTERUPT(X)       SPI_I2S_ITConfig(X,(uint8_t)SPI_I2S_IT_RXNE,ENABLE);
//============================DISABLE SPI INTERRRUPT============================
#define DISABLE_TX_RX_INTERUPT(X)   SPI_I2S_ITConfig(X,(uint8_t)SPI_I2S_IT_RXNE,DISABLE);\
                                    SPI_I2S_ITConfig(X,(uint8_t)SPI_I2S_IT_TXE,DISABLE);
#define DISABLE_TX_INTERUPT(X)      SPI_I2S_ITConfig(X,(uint8_t)SPI_I2S_IT_TXE,DISABLE);
#define DISABLE_RX_INTERUPT(X)      SPI_I2S_ITConfig(X,(uint8_t)SPI_I2S_IT_RXNE,DISABLE);
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
	uint8_t *p_buff_rx;
	uint8_t buff_tx[BUFF_LIFO_SIZE];
	uint8_t counter_tx;
	uint8_t counter_rx;
	uint8_t num;
}S_buf_rx_rx;

typedef struct{
	uint8_t f_interrupt;
	S_buf_rx_rx buf;
	TEMP_RXTX_MODE tx_rx_mode;
}S_buf_spi;


typedef struct{
	GPIO_TypeDef *CE_port;
	GPIO_TypeDef *CSN_port;
	GPIO_TypeDef *IRQ_port;
	uint16_t CE_pin;
	uint16_t CSN_pin;
	uint16_t IRQ_pin;
	SPI_TypeDef *SPI;
} S_peref;

typedef struct{
	uint8_t f_interrupt;
	S_buf_spi buf_g;
	S_peref peref;
}S_nrf_config;



uint8_t mcu_nrf_init( S_nrf_config *const pNRF);
void init_spi(volatile  S_nrf_config *p_nrf);
S_nrf_config* processing_init(const S_peref *per);
PER_ERROR init_gpio(const S_nrf_config *pnrf_gpio);
STATE spi_nrf24l01_TX(S_nrf_config *p_nrf,uint8_t comand, const uint8_t *p_data, uint8_t length);
STATE spi_nrf24l01_RX(S_nrf_config *p_nrf,uint8_t comand, uint8_t *p_data, uint8_t length);
STATE spi_nrf24l01_get_state(S_nrf_config *p_nrf);
void remap_out_pin(void);
uint8_t rcc_gpio_enable(const GPIO_TypeDef *gpio_enable);


#endif
