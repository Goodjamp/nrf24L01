
/*
 * spi_nrf24l01.c
 *
 *  Created on: January 29, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"

#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

#include "NRF24L01user.h"
#include "moduleHWInit.h"


#define NUMBER_OF_SPI      2
#define DEFAULT_SPI_INIT   {                                     \
                            .comState    = SPI_STATE_COMPLITED,  \
                            .data        = NULL,                 \
                            .dataCnt     = 0,                    \
                            .startTxFlag = true,                 \
                           }

/***************ENABLE SPI INTERRRUPT***************/
#define ENABLE_TX_RX_INTERUPT(X)    X->CR2|=(SPI_CR2_TXEIE |SPI_CR2_RXNEIE);
#define ENABLE_TX_INTERUPT(X)       X->CR2|=(SPI_CR2_TXEIE);
#define ENABLE_RX_INTERUPT(X)       X->CR2|=(SPI_CR2_RXNEIE);
/***************DISABLE SPI INTERRRUPT***************/
#define DISABLE_TX_RX_INTERUPT(X)   X->CR2&=~(SPI_CR2_TXEIE |SPI_CR2_RXNEIE);
#define DISABLE_TX_INTERUPT(X)      X->CR2&=~(SPI_CR2_TXEIE);
#define DISABLE_RX_INTERUPT(X)      X->CR2&=~(SPI_CR2_RXNEIE);

typedef enum{
	SPI_STATE_COMPLITED,
	SPI_STATE_RX,
	SPI_STATE_TX,
	SPI_STATE_ERROR,
}SPI_COM_STATE;

volatile static struct
{
    uint8_t                *data;
    uint8_t                dataCnt;
    uint8_t                totalNumber; // total number of RX / Tx bytes
    bool                   startTxFlag; // flag for start receive, used for mark first receive byte (NRF status register)
    volatile SPI_COM_STATE comState;
}spiComState[MAX_NUM_SPI]  =
{
		DEFAULT_SPI_INIT,
		DEFAULT_SPI_INIT
};

const struct{
	GPIO_TypeDef *CE_port;
	GPIO_TypeDef *CSN_port;
	GPIO_TypeDef *IRQ_port;
	uint16_t     CE_pin;
	uint16_t     CSN_pin;
	uint16_t     IRQ_pin;
}gpioComState[MAX_NUM_SPI] =
{
	[NRF_INTERFACE_N01] =
	{
		.CE_port  = SPI1_CE_PORT,
		.CSN_port = SPI1_CSN_PORT,
		.IRQ_port = SPI1_IRQ_PORT,
		.CE_pin   = SPI1_CE_PIN,
		.CSN_pin  = SPI1_CSN_PIN,
		.IRQ_pin  = SPI1_IRQ_PIN,
	},
	[NRF_INTERFACE_N02] =
	{
		.CE_port  = SPI2_CE_PORT,
		.CSN_port = SPI2_CSN_PORT,
		.IRQ_port = SPI2_IRQ_PORT,
		.CE_pin   = SPI2_CE_PIN,
		.CSN_pin  = SPI2_CSN_PIN,
		.IRQ_pin  = SPI2_IRQ_PIN,
	},
};

struct
{
	uint8_t   cnt;
	nrfHeader selNrf[MAX_NUM_SPI];
}perephirialInitList;

static SPI_TypeDef *listOfSPI[MAX_NUM_SPI] =
{
		[NRF_INTERFACE_N01] = SPI1,
		[NRF_INTERFACE_N02] = SPI2
};


static bool rcc_gpio_enable(const GPIO_TypeDef *gpio_enable)
{
	if(gpio_enable == GPIOA)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	}
	else if(gpio_enable == GPIOB)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	}
	else if(gpio_enable == GPIOC)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	}
	else if(gpio_enable == GPIOD)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	}
	else if(gpio_enable == GPIOE)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	}
	else
	{
		return false;
	}
	return true;
}


static PER_ERROR init_gpio(NRF_INTERFACE inINterface)
{
	GPIO_InitTypeDef GPIO_InitTypeDef_NRF;
	EXTI_InitTypeDef EXTI_InitTypeDef_IRQ;

	//  calc num external line from pin definition
	uint8_t take_ext_line(u16 pin)
	{
	    uint8_t counter = 0;
	    for(; counter < 16; counter++)
	    {
	        if((1 << counter) == pin)
		    {
			    return counter;
		    }
	    }
	    return 0xFF;
	};
	/******************check pin CE IRQ CSN*******************/
	if(!IS_GET_GPIO_PIN(gpioComState[inINterface].CE_pin))
	{
		return PER_ERROR_CE_PIN;
	}
	else if(!IS_GET_GPIO_PIN(gpioComState[inINterface].IRQ_pin))
	{
		return PER_ERROR_IRQ_PIN;
	}
	else if(!IS_GET_GPIO_PIN(gpioComState[inINterface].CSN_pin))
	{
		return PER_ERROR_CSN_PIN;
	}
	/******************check port CE IRQ CSN*******************/
	if(!IS_GPIO_ALL_PERIPH(gpioComState[inINterface].CE_port))
	{
		return PER_ERROR_CE_PORT;
	}
	else if(!IS_GPIO_ALL_PERIPH(gpioComState[inINterface].IRQ_port))
	{
		return PER_ERROR_IRQ_PORT;
	}
	else if(!IS_GPIO_ALL_PERIPH(gpioComState[inINterface].CSN_port))
	{
		return PER_ERROR_CSN_PORT;
	}

	/******************config gpio SCN*******************/
	rcc_gpio_enable(gpioComState[inINterface].CSN_port);
	GPIO_InitTypeDef_NRF.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeDef_NRF.GPIO_Pin=gpioComState[inINterface].CSN_pin;
	GPIO_InitTypeDef_NRF.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(gpioComState[inINterface].CSN_port, &GPIO_InitTypeDef_NRF);
	CSN_PIN_SET(gpioComState[inINterface]);

	/******************config gpioBuff CE*******************/
	rcc_gpio_enable(gpioComState[inINterface].CE_port);
	GPIO_InitTypeDef_NRF.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeDef_NRF.GPIO_Pin=gpioComState[inINterface].CE_pin;
	GPIO_InitTypeDef_NRF.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(gpioComState[inINterface].CE_port, &GPIO_InitTypeDef_NRF);
	CE_PIN_RESET(gpioComState[inINterface]);

	/*****************config gpioBuff IRQ******************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	rcc_gpio_enable(gpioComState[inINterface].IRQ_port);
	GPIO_InitTypeDef_NRF.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitTypeDef_NRF.GPIO_Pin=gpioComState[inINterface].IRQ_pin;
	GPIO_InitTypeDef_NRF.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(gpioComState[inINterface].IRQ_port, &GPIO_InitTypeDef_NRF);

	if(gpioComState[inINterface].IRQ_port==GPIOA)
	{
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA ,take_ext_line(gpioComState[inINterface].IRQ_pin));
	}
	else if(gpioComState[inINterface].IRQ_port==GPIOB)
	{
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB ,take_ext_line(gpioComState[inINterface].IRQ_pin));
	}
	else if(gpioComState[inINterface].IRQ_port==GPIOC)
	{
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC ,take_ext_line(gpioComState[inINterface].IRQ_pin));
	}
	else if(gpioComState[inINterface].IRQ_port==GPIOD)
	{
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOD ,take_ext_line(gpioComState[inINterface].IRQ_pin));
	}
	else if(gpioComState[inINterface].IRQ_port==GPIOE)
	{
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOE ,take_ext_line(gpioComState[inINterface].IRQ_pin));
	}

	EXTI_InitTypeDef_IRQ.EXTI_Line=gpioComState[inINterface].IRQ_pin;
	EXTI_InitTypeDef_IRQ.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitTypeDef_IRQ.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitTypeDef_IRQ.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitTypeDef_IRQ);

	// set interupt in vector
	if(gpioComState[inINterface].IRQ_pin == EXTI_Line0)
	{
		NVIC_EnableIRQ(EXTI0_IRQn);
	}
	else if(gpioComState[inINterface].IRQ_pin == EXTI_Line1)
	{
		NVIC_EnableIRQ(EXTI1_IRQn);
	}
	else if(gpioComState[inINterface].IRQ_pin == EXTI_Line2)
	{
		NVIC_EnableIRQ(EXTI2_IRQn);
	}
	else if(gpioComState[inINterface].IRQ_pin == EXTI_Line3)
	{
		NVIC_EnableIRQ(EXTI3_IRQn);
	}
	else if(gpioComState[inINterface].IRQ_pin == EXTI_Line4)
	{
		NVIC_EnableIRQ(EXTI4_IRQn);
	}
	else if((gpioComState[inINterface].IRQ_pin >= EXTI_Line5) && (gpioComState[inINterface].IRQ_pin <= EXTI_Line9))
	{
		NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
	else if((gpioComState[inINterface].IRQ_pin >= EXTI_Line10) && (gpioComState[inINterface].IRQ_pin <= EXTI_Line15))
	{
		NVIC_EnableIRQ(EXTI15_10_IRQn);
	}

	__enable_irq();

	return PER_ERROR_OK;
}


static inline void spi_nrf_find_irq(u16 pin_input){
	uint8_t cnt = 0;
	for(; cnt < perephirialInitList.cnt; cnt++)
	{
		if(gpioComState[cnt].IRQ_pin == pin_input)
		{
			NRF24L01_set_interupt(perephirialInitList.selNrf[cnt]);
			return;
		}
	}
}


void EXTI0_IRQHandler(void){
	EXTI_ClearITPendingBit(EXTI_Line0);
	// set f_interrupt
	spi_nrf_find_irq(GPIO_Pin_0);
}


void EXTI1_IRQHandler(void){
	EXTI_ClearITPendingBit(EXTI_Line1);
	// set f_interrupt
	spi_nrf_find_irq(GPIO_Pin_1);
}


void EXTI2_IRQHandler(void){
	EXTI_ClearITPendingBit(EXTI_Line2);
	// read stattus register
	spi_nrf_find_irq(GPIO_Pin_2);
}


void EXTI3_IRQHandler(void){
	EXTI_ClearITPendingBit(EXTI_Line3);
	// read stattus register
	EXTI_ClearFlag(EXTI_Line3);
	//spi_nrf_find_irq(GPIO_Pin_3);
}


void EXTI4_IRQHandler(void){
	EXTI_ClearITPendingBit(EXTI_Line4);
	// read stattus register
	spi_nrf_find_irq(GPIO_Pin_4);
}


void EXTI9_5_IRQHandler(void){
	u16 ext_line=EXTI->PR;
	EXTI_ClearITPendingBit(ext_line);
	// read stattus register
	spi_nrf_find_irq(ext_line);
}

void EXTI15_10_IRQHandler(void){
	u16 ext_line=EXTI->PR;
	EXTI_ClearITPendingBit(ext_line);
	// read stattus register
	spi_nrf_find_irq(ext_line);
}


static void remap_out_pin(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2,ENABLE);
}


static inline uint8_t getSPIIndex(nrfHeader inNRF)
{
	uint8_t cnt = 0;
	for(; cnt < MAX_NUM_SPI; cnt++)
	{
		if(inNRF == perephirialInitList.selNrf[cnt])
		{
			return cnt;
		}
	}
	return 0xFF;
}


volatile struct
{
	uint16_t srSPI_1[100];
	uint16_t txe_1[100];
	uint8_t cnt_1;
	uint16_t srSPI_2[100];
	uint16_t txe_2[100];
	uint8_t cnt_2;
}stateOfSPI;

static void spiTXProcessing(NRF_INTERFACE inINterface)
{

	if(inINterface == 0)
	{
		stateOfSPI.srSPI_1[stateOfSPI.cnt_1++] = SPI1->SR;
		//stateOfSPI.txe_1[stateOfSPI.cnt_1++] = SPI_I2S_GetITStatus(listOfSPI[inINterface], SPI_I2S_IT_TXE);
	}
	else
	{
		stateOfSPI.srSPI_2[stateOfSPI.cnt_2++] = SPI2->SR;
		//stateOfSPI.txe_2[stateOfSPI.cnt_2++]   = SPI_I2S_GetITStatus(listOfSPI[inINterface], SPI_I2S_IT_TXE);
	}

	if(SPI_I2S_GetITStatus(listOfSPI[inINterface], SPI_I2S_IT_TXE) == RESET)
	{

		//spiComState[inINterface].comState = SPI_STATE_ERROR;
		return;
	}

	if(spiComState[inINterface].dataCnt == spiComState[inINterface].totalNumber)
	{
		spiComState[inINterface].comState = SPI_STATE_COMPLITED;
		DISABLE_TX_INTERUPT(listOfSPI[inINterface]);
	}
	else
	{
		// send next frame
		SPI_I2S_SendData(listOfSPI[inINterface], spiComState[inINterface].data[spiComState[inINterface].dataCnt++]);
	}
}


static inline void spiRXProcessing(NRF_INTERFACE inINterface)
{
	if(SPI_I2S_GetITStatus(listOfSPI[inINterface], SPI_I2S_IT_RXNE) == RESET)
	{
		//spiComState[inINterface].comState = SPI_STATE_ERROR;
		return;
	}
	if(spiComState[inINterface].startTxFlag)  // ignore first Rx byte, because this is status register
	{
		spiComState[inINterface].startTxFlag = false;
		(void)listOfSPI[inINterface]->DR;
		SPI_I2S_SendData(listOfSPI[inINterface], (uint16_t)0);
		return;
	}
	spiComState[inINterface].data[spiComState[inINterface].dataCnt++] = (uint8_t)SPI_I2S_ReceiveData(listOfSPI[inINterface]);
	if(spiComState[inINterface].dataCnt == spiComState[inINterface].totalNumber)
	{
		spiComState[inINterface].comState = SPI_STATE_COMPLITED;
		DISABLE_RX_INTERUPT(listOfSPI[inINterface]);
	}
	else
	{
		SPI_I2S_SendData(listOfSPI[inINterface], (uint16_t)0);
	}
}


static inline void nrfTransactionProcessing(NRF_INTERFACE inINterface){
	switch(spiComState[inINterface].comState)
    {
    case SPI_STATE_TX:
    	spiTXProcessing(inINterface);
    	break;
    case SPI_STATE_RX:
    	spiRXProcessing(inINterface);
    	break;
    default:
    	break;
    }
}


void SPI1_IRQHandler(void)
{
	nrfTransactionProcessing(NRF_INTERFACE_N01);
}


void SPI2_IRQHandler(void)
{
	nrfTransactionProcessing(NRF_INTERFACE_N02);
}


STATE nrf24l01_spi_TX(nrfHeader inNRF, uint8_t comand, uint8_t *p_data, uint8_t length){
	uint8_t spiIndex;
	if( (spiIndex = getSPIIndex(inNRF)) == 0xFF)
	{
		return 0;  //TODO return correct status!!!!!!!!!!!!!!!!!!!!!!!!!!
	}
    if((spiComState[spiIndex].comState != SPI_STATE_COMPLITED) &&
       (spiComState[spiIndex].comState != SPI_STATE_ERROR))
    {
    	return spiComState[spiIndex].comState;
    }
    spiComState[spiIndex].data        = p_data;
    spiComState[spiIndex].dataCnt 	   = 0;
    spiComState[spiIndex].totalNumber = length;
    spiComState[spiIndex].startTxFlag = true;
    spiComState[spiIndex].comState    = SPI_STATE_TX;

    SPI_Cmd(listOfSPI[spiIndex], ENABLE);
    CSN_PIN_RESET(gpioComState[spiIndex]);
	SPI_I2S_SendData(listOfSPI[spiIndex],(u16)(comand));
	ENABLE_TX_INTERUPT(listOfSPI[spiIndex]);
	while(spiComState[spiIndex].comState == SPI_STATE_TX){}  // Wait completing transaction
	// according  RM0090 25.3.8
	while(SPI_I2S_GetFlagStatus(listOfSPI[spiIndex], SPI_I2S_FLAG_BSY) == SET){}
	SPI_Cmd(listOfSPI[spiIndex], DISABLE);
	CSN_PIN_SET(gpioComState[spiIndex]);

	return STATE_OK;
}


STATE nrf24l01_spi_RX(nrfHeader inNRF, uint8_t comand, uint8_t *p_data, uint8_t length){
	uint8_t spiIndex;
	if( (spiIndex = getSPIIndex(inNRF)) == 0xFF)
	{
		return 0;  //TODO return correct status!!!!!!!!!!!!!!!!!!!!!!!!!!
	}
    if((spiComState[spiIndex].comState != SPI_STATE_COMPLITED) &&
       (spiComState[spiIndex].comState != SPI_STATE_ERROR))
    {
    	return spiComState[spiIndex].comState;
    }
    spiComState[spiIndex].data        = p_data;
    spiComState[spiIndex].dataCnt 	   = 0;
    spiComState[spiIndex].totalNumber = length;
    spiComState[spiIndex].startTxFlag = true;
    spiComState[spiIndex].comState    = SPI_STATE_RX;

    SPI_Cmd(listOfSPI[spiIndex], ENABLE);
    SPI_I2S_ReceiveData(listOfSPI[spiIndex]);
    CSN_PIN_RESET(gpioComState[spiIndex]);
	SPI_I2S_ReceiveData(listOfSPI[spiIndex]);            // read DR for clear RXNE flag ()
	SPI_I2S_SendData(listOfSPI[spiIndex],(u16)(comand));
	ENABLE_RX_INTERUPT(listOfSPI[spiIndex]);
	while(spiComState[spiIndex].comState == SPI_STATE_RX){}  // Wait completing transaction
	// according  RM0090 25.3.8
	while(SPI_I2S_GetFlagStatus(listOfSPI[spiIndex], SPI_I2S_FLAG_RXNE) == SET){}
	SPI_I2S_ReceiveData(listOfSPI[spiIndex]);                 // read DR for clear RXNE flag
	CSN_PIN_SET(gpioComState[spiIndex]);
	SPI_Cmd(listOfSPI[spiIndex], DISABLE);

	return STATE_OK;
}


static void initSPI(NRF_INTERFACE inINterface)
{
	GPIO_TypeDef *port_MISO, *port_MOSI, *port_SCK;
	u16 pin_MISO, pin_MOSI, pin_SCK;
	GPIO_InitTypeDef GPIO_InitTypeDef_SPI;
	SPI_InitTypeDef spiInitDef;
	if(listOfSPI[inINterface] == SPI1)
	{
		// SCK   -  PA5  Alternate function push-pull
		// MISO  -  PA6  Input floating
		// MOSI  -  PA7  Alternate function push-pull
		pin_SCK=GPIO_Pin_5;
		pin_MOSI=GPIO_Pin_7;
		pin_MISO=GPIO_Pin_6;
		port_SCK=GPIOA;
		port_MOSI=GPIOA;
		port_MISO=GPIOA;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	}
	else if(listOfSPI[inINterface] == SPI2)
	{
		// SCK   -  PB13  Alternate function push-pull
		// MISO  -  PB14  Input floating
		// MOSI  -  PB15  Alternate function push-pull
		pin_SCK=GPIO_Pin_13;
		pin_MOSI=GPIO_Pin_15;
		pin_MISO=GPIO_Pin_14;
		port_SCK=GPIOB;
		port_MOSI=GPIOB;
		port_MISO=GPIOB;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	}
	else
	{
		return;
	}

	GPIO_InitTypeDef_SPI.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitTypeDef_SPI.GPIO_Pin=pin_SCK;
	GPIO_InitTypeDef_SPI.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(port_SCK, &GPIO_InitTypeDef_SPI);

	GPIO_InitTypeDef_SPI.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitTypeDef_SPI.GPIO_Pin=pin_MISO;
	GPIO_InitTypeDef_SPI.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(port_MISO, &GPIO_InitTypeDef_SPI);

	GPIO_InitTypeDef_SPI.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitTypeDef_SPI.GPIO_Pin=pin_MOSI;
	GPIO_InitTypeDef_SPI.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(port_MOSI, &GPIO_InitTypeDef_SPI);

	spiInitDef.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_16;
	spiInitDef.SPI_CPHA=SPI_CPHA_1Edge;
	spiInitDef.SPI_CPOL=SPI_CPOL_Low;
	spiInitDef.SPI_DataSize=SPI_DataSize_8b;
	spiInitDef.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	spiInitDef.SPI_FirstBit=SPI_FirstBit_MSB;
	spiInitDef.SPI_Mode=SPI_Mode_Master;
	spiInitDef.SPI_NSS=SPI_NSS_Soft;
	SPI_Init(listOfSPI[inINterface], &spiInitDef);
	if(listOfSPI[inINterface] == SPI1){
		NVIC_EnableIRQ(SPI1_IRQn);
	}
	else if (listOfSPI[inINterface] == SPI2){
		NVIC_EnableIRQ(SPI2_IRQn);
	}
}


void  nrf24l01_ce_set(nrfHeader inNRF)
{
	uint8_t spiIndex;
	if( (spiIndex = getSPIIndex(inNRF)) == 0xFF)
	{
		return ;
	}
	CE_PIN_SET(gpioComState[spiIndex]);
}


void  nrf24l01_ce_clear(nrfHeader inNRF)
{
	uint8_t spiIndex;
	if( (spiIndex = getSPIIndex(inNRF)) == 0xFF)
	{
		return ;
	}
	CE_PIN_RESET(gpioComState[spiIndex]);
}


void  nrf24l01_ce_puls (nrfHeader inNRF)
{
	uint8_t spiIndex;
	if( (spiIndex = getSPIIndex(inNRF)) == 0xFF)
	{
		return ;
	}
	CE_PIN_SET(gpioComState[spiIndex]);
	volatile uint32_t cnt = 0;
	while(cnt++ < (72 * 8)){}
	CE_PIN_RESET(gpioComState[spiIndex]);
}


uint8_t nrf24l01_interface_init(nrfHeader inNRF, NRF_INTERFACE inINterface)
{
	if(inINterface >= MAX_NUM_SPI)
	{
		return PER_ERROR_SPI_NUMBER;
	}
	if(inINterface > 0)
	{
	    remap_out_pin();
	}
	initSPI(inINterface);
	init_gpio(inINterface);
	perephirialInitList.selNrf[perephirialInitList.cnt++] = inNRF;
	return PER_ERROR_OK;
}
