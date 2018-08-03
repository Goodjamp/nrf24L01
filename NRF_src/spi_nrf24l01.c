/*
 * spi_nrf24l01.c
 *
 *  Created on: January 29, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
//#include "NRF24L01.h"
#include "spi_nrf24l01.h"
#include "stm32f10x_exti.h"
#include "string.h"

S_nrf_config *nrf_spi[MAX_NUM_NRF];
static uint8_t counter_init_nrf=0;

PER_ERROR mcu_nrf_init(S_nrf_config *const pNRF){

	if(counter_init_nrf>=MAX_NUM_NRF){return 1;}
	remap_out_pin();
	init_gpio(pNRF);
	init_spi(pNRF);
	nrf_spi[counter_init_nrf]=pNRF;
	counter_init_nrf++;

	return PER_ERROR_OK;
}


//-----------функция processing_TC_init_spi---------------
//функция init_spi - конфигурирую SPI
PER_ERROR init_gpio(const S_nrf_config *pnrf_gpio){
	GPIO_InitTypeDef GPIO_InitTypeDef_NRF;
	EXTI_InitTypeDef EXTI_InitTypeDef_IRQ;

	//  calc num external line from pin definition
	uint8_t take_ext_line(u16 pin){
		uint8_t counter;
		for(counter=0;counter<16;counter++){
			if((1<<counter)==pin){return counter;}
		}
		return 0xFF;
	};

//============CHECK PEREPGIRIAL PARAMITERS============================
	//-----------check pin CE IRQ CSN--------------------------------
	if(!IS_GET_GPIO_PIN(pnrf_gpio->peref.CE_pin)){
		return PER_ERROR_CE_PIN;
	}
	else if(!IS_GET_GPIO_PIN(pnrf_gpio->peref.IRQ_pin)){
		return PER_ERROR_IRQ_PIN;
	}
	else if(!IS_GET_GPIO_PIN(pnrf_gpio->peref.CSN_pin)){
		return PER_ERROR_CSN_PIN;
	}
	// ------------check port CE IRQ CSN-----------------------------
	if(!IS_GPIO_ALL_PERIPH(pnrf_gpio->peref.CE_port)){
		return PER_ERROR_CE_PORT;
	}
	else if(!IS_GPIO_ALL_PERIPH(pnrf_gpio->peref.IRQ_port)){
		return PER_ERROR_IRQ_PORT;
	}
	else if(!IS_GPIO_ALL_PERIPH(pnrf_gpio->peref.CSN_port)){
		return PER_ERROR_CSN_PORT;
	}
//====================================================================

	// ------------config gpio SCN--------------------------
	rcc_gpio_enable(pnrf_gpio->peref.CSN_port);
	GPIO_InitTypeDef_NRF.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeDef_NRF.GPIO_Pin=pnrf_gpio->peref.CSN_pin;
	GPIO_InitTypeDef_NRF.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(pnrf_gpio->peref.CSN_port, &GPIO_InitTypeDef_NRF);
	SPI_CSN_SET(pnrf_gpio);

	// ------------config gpio CE--------------------------
	rcc_gpio_enable(pnrf_gpio->peref.CE_port);
	GPIO_InitTypeDef_NRF.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeDef_NRF.GPIO_Pin=pnrf_gpio->peref.CE_pin;
	GPIO_InitTypeDef_NRF.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(pnrf_gpio->peref.CE_port, &GPIO_InitTypeDef_NRF);
	NRF_CE_RESET(pnrf_gpio);

	// ------------config gpio IRQ--------------------------

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	rcc_gpio_enable(pnrf_gpio->peref.IRQ_port);
	GPIO_InitTypeDef_NRF.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitTypeDef_NRF.GPIO_Pin=pnrf_gpio->peref.IRQ_pin;
	GPIO_InitTypeDef_NRF.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(pnrf_gpio->peref.IRQ_port, &GPIO_InitTypeDef_NRF);

	if(pnrf_gpio->peref.IRQ_port==GPIOA)
	{
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA ,take_ext_line(pnrf_gpio->peref.IRQ_pin));
	}
	else if(pnrf_gpio->peref.IRQ_port==GPIOB)
	{
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB ,take_ext_line(pnrf_gpio->peref.IRQ_pin));
	}
	else if(pnrf_gpio->peref.IRQ_port==GPIOC)
	{
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC ,take_ext_line(pnrf_gpio->peref.IRQ_pin));
	}
	else if(pnrf_gpio->peref.IRQ_port==GPIOD)
	{
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOD ,take_ext_line(pnrf_gpio->peref.IRQ_pin));
	}
	else if(pnrf_gpio->peref.IRQ_port==GPIOE)
	{
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOE ,take_ext_line(pnrf_gpio->peref.IRQ_pin));
	}

	EXTI_InitTypeDef_IRQ.EXTI_Line=pnrf_gpio->peref.IRQ_pin;
	EXTI_InitTypeDef_IRQ.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitTypeDef_IRQ.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitTypeDef_IRQ.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitTypeDef_IRQ);

	// set interupt in vector
	if(pnrf_gpio->peref.IRQ_pin == EXTI_Line0)
	{
		NVIC_EnableIRQ(EXTI0_IRQn);
	}
	else if(pnrf_gpio->peref.IRQ_pin == EXTI_Line1)
	{
		NVIC_EnableIRQ(EXTI1_IRQn);
	}
	else if(pnrf_gpio->peref.IRQ_pin == EXTI_Line2)
	{
		NVIC_EnableIRQ(EXTI2_IRQn);
	}
	else if(pnrf_gpio->peref.IRQ_pin == EXTI_Line3)
	{
		NVIC_EnableIRQ(EXTI3_IRQn);
	}
	else if(pnrf_gpio->peref.IRQ_pin == EXTI_Line4)
	{
		NVIC_EnableIRQ(EXTI4_IRQn);
	}
	else if((pnrf_gpio->peref.IRQ_pin >= EXTI_Line5) && (pnrf_gpio->peref.IRQ_pin <= EXTI_Line9))
	{
		NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
	else if((pnrf_gpio->peref.IRQ_pin >= EXTI_Line10) && (pnrf_gpio->peref.IRQ_pin <= EXTI_Line15))
	{
		NVIC_EnableIRQ(EXTI15_10_IRQn);
	}

	__enable_irq();

	return PER_ERROR_OK;
}


uint8_t rcc_gpio_enable(const GPIO_TypeDef *gpio_enable){
	if(gpio_enable==GPIOA)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	}
	else if(gpio_enable==GPIOB)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	}
	else if(gpio_enable==GPIOC)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	}
	else if(gpio_enable==GPIOD)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	}
	else if(gpio_enable==GPIOE)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	}
	else{return 1;}
	return 0;
}

static inline void spi_nrf_find_irq(u16 pin_input){
	uint8_t counter;
	for(counter=0;counter<counter_init_nrf;counter++){
		if(nrf_spi[counter]->peref.IRQ_pin==pin_input){
			nrf_spi[counter]->f_interrupt=1;
			break;
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


void remap_out_pin(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2,ENABLE);
}


void init_spi(volatile  S_nrf_config *p_nrf){
	GPIO_TypeDef *port_MISO, *port_MOSI, *port_SCK;
	u16 pin_MISO, pin_MOSI, pin_SCK;
	GPIO_InitTypeDef GPIO_InitTypeDef_SPI;
	SPI_InitTypeDef SPI_InitTypeDef_NRF;
	if(p_nrf->peref.SPI==SPI1){
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
	else if (p_nrf->peref.SPI==SPI2){
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
	else {return;}


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

	SPI_InitTypeDef_NRF.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4;
	SPI_InitTypeDef_NRF.SPI_CPHA=SPI_CPHA_1Edge;
	SPI_InitTypeDef_NRF.SPI_CPOL=SPI_CPOL_Low;
	SPI_InitTypeDef_NRF.SPI_DataSize=SPI_DataSize_8b;
	SPI_InitTypeDef_NRF.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPI_InitTypeDef_NRF.SPI_FirstBit=SPI_FirstBit_MSB;
	SPI_InitTypeDef_NRF.SPI_Mode=SPI_Mode_Master;
	SPI_InitTypeDef_NRF.SPI_NSS=SPI_NSS_Soft;
	SPI_Init(p_nrf->peref.SPI,&SPI_InitTypeDef_NRF);
	//SPI_I2S_ITConfig(p_nrf->peref.SPI,SPI_I2S_IT_TXE,ENABLE);

	if(p_nrf->peref.SPI==SPI1){
		//NVIC_EnableIRQ(SPI1_IRQn);
	}
	else if (p_nrf->peref.SPI==SPI2){
		//NVIC_EnableIRQ(SPI2_IRQn);
	}
	SPI_Cmd(p_nrf->peref.SPI,ENABLE);

	SPI_CSN_SET(p_nrf);
}



/*
void SPI1_IRQHandler(void){

	if(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE))// Rx data register no empty
	{
		nrf_spi[0]->buf_g.buf.counter_rx++;

		if(nrf_spi[0]->buf_g.tx_rx_mode==RX_MODE){
			if((nrf_spi[0]->buf_g.buf.counter_rx==1)||((nrf_spi[0]->buf_g.buf.counter_rx-1)>nrf_spi[0]->buf_g.buf.num)){  // firest interrupt - RX STATUS_REGISTER
				(void)SPI1->DR;
			}
			else if((nrf_spi[0]->buf_g.buf.counter_rx>1)&&((nrf_spi[0]->buf_g.buf.counter_rx-1)<=nrf_spi[0]->buf_g.buf.num)){
 *(nrf_spi[0]->buf_g.buf.p_buff_rx+nrf_spi[0]->buf_g.buf.counter_rx-2)=(uint8_t)SPI_I2S_ReceiveData(SPI1);
				if((nrf_spi[0]->buf_g.buf.counter_rx-1)==nrf_spi[0]->buf_g.buf.num){nrf_spi[0]->buf_g.tx_rx_mode=FREE_MODE;}
			}

		}
		else if(nrf_spi[0]->buf_g.tx_rx_mode==TX_MODE){
			(void)SPI1->DR;

			if((nrf_spi[0]->buf_g.buf.counter_rx-1)>=nrf_spi[0]->buf_g.buf.num){  // firest interrupt - RX STATUS_REGISTER
				nrf_spi[0]->buf_g.tx_rx_mode=FREE_MODE;
			}
		}
	}

	else if((SPI1->SR &  SPI_SR_TXE )&&(SPI1->CR2 & SPI_CR2_TXEIE))//Transmit reg empty
	{
		nrf_spi[0]->buf_g.buf.counter_tx++;
		if(nrf_spi[0]->buf_g.tx_rx_mode==RX_MODE){
			if(nrf_spi[0]->buf_g.buf.counter_tx<=nrf_spi[0]->buf_g.buf.num){
				SPI1->DR=(u16)0b10101010;
				if(nrf_spi[0]->buf_g.buf.counter_tx==nrf_spi[0]->buf_g.buf.num){
					DISABLE_TX_INTERUPT(SPI1);
					return;
				}
				return;
			}
			else{
				DISABLE_TX_INTERUPT(SPI1);
				return;
			}
		}
		else if(nrf_spi[0]->buf_g.tx_rx_mode==TX_MODE){
			if(nrf_spi[0]->buf_g.buf.counter_tx<=nrf_spi[0]->buf_g.buf.num){
				SPI1->DR=(u16)nrf_spi[0]->buf_g.buf.buff_tx[nrf_spi[0]->buf_g.buf.counter_tx-1];
				if(nrf_spi[0]->buf_g.buf.counter_tx==nrf_spi[0]->buf_g.buf.num){
					DISABLE_TX_INTERUPT(SPI1);
					return;
				}
				return;
			}
			else{
				DISABLE_TX_INTERUPT(SPI1);
				return;
			}
		}
	}
}

 */

void spi_nrf24l01_Rx(S_nrf_config *pNRF){
	SPI_CSN_RESET(pNRF);
	while(1){
		if((pNRF->peref.SPI->SR | SPI_SR_RXNE)&&((pNRF->buf_g.buf.counter_rx-1)<=pNRF->buf_g.buf.num))
		{
			pNRF->buf_g.buf.counter_rx++;

			if(pNRF->buf_g.tx_rx_mode==RX_MODE)
			{
				if((pNRF->buf_g.buf.counter_rx==1)||((pNRF->buf_g.buf.counter_rx-1)>pNRF->buf_g.buf.num)) // firest interrupt - RX STATUS_REGISTER
				{
					(void)pNRF->peref.SPI->DR;
				}
				else
				{
					*(pNRF->buf_g.buf.p_buff_rx+pNRF->buf_g.buf.counter_rx-2)=(uint8_t)SPI_I2S_ReceiveData(pNRF->peref.SPI);
					if((pNRF->buf_g.buf.counter_rx-1)==pNRF->buf_g.buf.num)
					{
						pNRF->buf_g.tx_rx_mode=FREE_MODE;
						SPI_CSN_SET(pNRF);
						return;
					}
				}

			}
			else if(pNRF->buf_g.tx_rx_mode==TX_MODE)
			{
				(void)pNRF->peref.SPI->DR;
				if((pNRF->buf_g.buf.counter_rx-1) >= pNRF->buf_g.buf.num)
				{
					pNRF->buf_g.tx_rx_mode=FREE_MODE;
					SPI_CSN_SET(pNRF);
					return;
				}
			}
		}
		if((pNRF->peref.SPI->SR | SPI_SR_TXE) && (pNRF->buf_g.buf.counter_tx < pNRF->buf_g.buf.num))
		{
			pNRF->buf_g.buf.counter_tx++;
			pNRF->peref.SPI->DR=(u16)pNRF->buf_g.buf.buff_tx[pNRF->buf_g.buf.counter_tx-1];
		}
	}

}


STATE spi_nrf24l01_TX(S_nrf_config *p_nrf,uint8_t comand, const uint8_t *p_data, uint8_t length){
	if(p_nrf->buf_g.tx_rx_mode!=FREE_MODE){return BUSY_STATE;}
	SPI_CSN_RESET(p_nrf);
	memcpy(&p_nrf->buf_g.buf.buff_tx[0],p_data,length);
	p_nrf->buf_g.buf.num=length;
	p_nrf->buf_g.buf.counter_tx=0;
	p_nrf->buf_g.buf.counter_rx=0;
	p_nrf->buf_g.tx_rx_mode=TX_MODE;
	SPI_I2S_SendData(p_nrf->peref.SPI,(u16)(comand));
	SPI_I2S_ReceiveData(p_nrf->peref.SPI);
	spi_nrf24l01_Rx(p_nrf);
	//ENABLE_TX_INTERUPT(p_nrf->peref.SPI);
	return OK_STATE;
}


STATE spi_nrf24l01_RX(S_nrf_config *p_nrf,uint8_t comand, uint8_t *p_data, uint8_t length){
	if(p_nrf->buf_g.tx_rx_mode!=FREE_MODE){return BUSY_STATE;}
	SPI_CSN_RESET(p_nrf);
	p_nrf->buf_g.buf.p_buff_rx=p_data;
	p_nrf->buf_g.buf.num=length;
	p_nrf->buf_g.buf.counter_rx=0;
	p_nrf->buf_g.buf.counter_tx=0;
	p_nrf->buf_g.tx_rx_mode=RX_MODE;
	SPI_I2S_SendData(p_nrf->peref.SPI,(u16)(comand));
	spi_nrf24l01_Rx(p_nrf);
	//ENABLE_TX_INTERUPT(p_nrf->peref.SPI);
	return OK_STATE;
}

STATE spi_nrf24l01_get_state(S_nrf_config *p_nrf){
	if(p_nrf->buf_g.tx_rx_mode==FREE_MODE){return OK_STATE;}
	else{return BUSY_STATE;};
}


