#include "stdint.h"

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "timProcessing.h"

#define SELL_TIM   TIM4

volatile static uint32_t cntMs = 1;;


uint8_t cnt;
void initDebug(void)
{
	GPIO_InitTypeDef initGPIO;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	initGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
	initGPIO.GPIO_Pin  = GPIO_Pin_2;
	initGPIO.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_Init(GPIOA, &initGPIO);

}


void clockInit(void)
{
	//initDebug();
	RCC_ClocksTypeDef clockFrq;
	RCC_GetClocksFreq(&clockFrq);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);


    TIM_SetAutoreload(SELL_TIM, 1000);
    TIM_ARRPreloadConfig(SELL_TIM, ENABLE);
    TIM_PrescalerConfig(SELL_TIM, clockFrq.PCLK1_Frequency/1000000 - 1, TIM_PSCReloadMode_Immediate);

    TIM_ITConfig(SELL_TIM, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM_Cmd(SELL_TIM, ENABLE);
}


void TIM4_IRQHandler(void)
{
	TIM_ClearFlag(SELL_TIM, TIM_FLAG_Update);
	cntMs++;
}


uint32_t xTaskGetTickCount(void)
{
	return cntMs;
}
