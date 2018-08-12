/*
 * spi_nrf24l01.h
 *
 *  Created on: January 28, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef SPI_NRF24L01_H_
#define SPI_NRF24L01_H_

//=================NRF1 DEFINITION========================
//--------CSN pin define---------------------
#define SPI1_CSN_PORT  GPIOA
#define SPI1_CSN_PIN   GPIO_Pin_4
//--------CE pin define---------------------
#define SPI1_CE_PORT   GPIOB
#define SPI1_CE_PIN    GPIO_Pin_1
//--------IRQ pin define---------------------
#define SPI1_IRQ_PORT  GPIOB
#define SPI1_IRQ_PIN   GPIO_Pin_0


//=================NRF2 DEFINITION========================
//--------CSN pin define---------------------
#define SPI2_CSN_PORT  GPIOB
#define SPI2_CSN_PIN   GPIO_Pin_12
//--------CE pin define---------------------
#define SPI2_CE_PORT   GPIOA
#define SPI2_CE_PIN    GPIO_Pin_9
//--------IRQ pin define---------------------
#define SPI2_IRQ_PORT  GPIOA
#define SPI2_IRQ_PIN   GPIO_Pin_8

#define MAX_NUM_SPI 2

#define CSN_PIN_SET(X)   GPIO_SetBits(X.CSN_port,X.CSN_pin)
#define CSN_PIN_RESET(X) GPIO_ResetBits(X.CSN_port,X.CSN_pin)

#define CE_PIN_SET(X)    GPIO_SetBits(X.CE_port,X.CE_pin)
#define CE_PIN_RESET(X)  GPIO_ResetBits(X.CE_port,X.CE_pin)

typedef enum{
	PER_ERROR_OK=0,
	PER_ERROR_CE_PIN,
	PER_ERROR_IRQ_PIN,
	PER_ERROR_CSN_PIN,
	PER_ERROR_CE_PORT,
	PER_ERROR_IRQ_PORT,
	PER_ERROR_CSN_PORT,
	PER_ERROR_SPI_NUMBER
}PER_ERROR;

#endif
