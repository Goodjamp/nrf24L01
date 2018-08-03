/*
 * task.h
 *
 *  Created on: January 29, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef TASK_H_
#define TASK_H_
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "NRF24L01.h"
#include "spi_nrf24l01.h"


//=================NRF1 DEFINITION========================
//--------CSN pin define---------------------
#define NRF1_CSN_PORT  GPIOA
#define NRF1_CSN_PIN   GPIO_Pin_4
//--------CE pin define---------------------
#define NRF1_CE_PORT  GPIOB
#define NRF1_CE_PIN   GPIO_Pin_1
//--------IRQ pin define---------------------
#define NRF1_IRQ_PORT  GPIOB
#define NRF1_IRQ_PIN   GPIO_Pin_0
//--------SPI define---------------------
#define NRF1_SPI  SPI1


//=================NRF2 DEFINITION========================
//--------CSN pin define---------------------
#define NRF2_CSN_PORT  GPIOB
#define NRF2_CSN_PIN   GPIO_Pin_12
//--------CE pin define---------------------
#define NRF2_CE_PORT  GPIOA
#define NRF2_CE_PIN   GPIO_Pin_9
//--------IRQ pin define---------------------
#define NRF2_IRQ_PORT  GPIOA
#define NRF2_IRQ_PIN   GPIO_Pin_8
//--------SPI define---------------------
#define NRF2_SPI  SPI2

void task_nrf24l01(void);

#endif
