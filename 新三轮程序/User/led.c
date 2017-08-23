#include "led.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

//PA0 PA1 PA2 PA3
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;

	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
}


void LED_A0_ON(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
}
void LED_A0_OFF(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
}

void LED_A1_ON(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}
void LED_A1_OFF(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
}

void LED_A2_ON(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);
}
void LED_A2_OFF(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_2);
}

void LED_A3_ON(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
}
void LED_A3_OFF(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_3);
}