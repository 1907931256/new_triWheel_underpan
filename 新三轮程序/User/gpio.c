/**
  ******************************************************************************
  * @file    gpio.c
  * @author  Calcus Lee
  * @version V1.0.1
  * @date    9-August-2013
  * @brief   functions of gpio
  ******************************************************************************
**/
#include "gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
/**
  * @brief  set the pins of a specific GPIO group to be input or output driver pin.
  * @param  GPIOx: where x can be A-I.
  * @param  GPIO_Pin: The specific pins you want to select in group GPIOX.
			This parameter can be combination of GPIO_Pin_x where x can be (0..15) @ref GPIO_pins_define
  * @param  GPIO_Mode. the value can be one of the following value
		    GPIO_Mode_IN   
		    GPIO_Mode_OUT 
		    GPIO_Mode_AF  
		    GPIO_Mode_AN
  * @retval None
  * @author Calcus Lee
**/
            
void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	  /* Enable GPIOx, clock */  
  switch((uint32_t)GPIOx)
  {
    case GPIOA_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    break;
    }
	case GPIOB_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    break;
    }
	case GPIOC_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    break;
    }
	case GPIOD_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    break;
    }
	case GPIOE_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    break;
    }
	case GPIOF_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    break;
    }
	case GPIOG_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    break;
    }
	case GPIOH_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    break;
    }
	case GPIOI_BASE: 
    {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
    break;
    }	
	default: break;
  }
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode;

	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(GPIOx,&GPIO_InitStructure);	
}




//蜂鸣器初始化
void MyBeefInit(void)
{
	GPIO_Init_Pins(GPIOA,GPIO_Pin_11,GPIO_Mode_OUT);
	GPIO_Init_Pins(GPIOA,GPIO_Pin_12,GPIO_Mode_OUT);
}



//行程开关初始化  拉高响
void MySwitchInit(void)
{
	GPIO_Init_Pins(GPIOA,GPIO_Pin_5,GPIO_Mode_IN);
}


void BeefON(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
	GPIO_ResetBits(GPIOA,GPIO_Pin_12);
}

void BeefOFF(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_11);
	GPIO_SetBits(GPIOA,GPIO_Pin_12);
}



//返回1 代表触发
//返回0 代表未触发
_Bool ReadSwitch(void)
{
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 1)//高电平触发
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


//led闪烁程序，用于程序执行的判断
extern long TIM2_COUNT;
void LedTwinkle(void)
{
	if(TIM2_COUNT>40)
	{
		BeefON();
		if(TIM2_COUNT>80)
		{
			TIM2_COUNT = 0;
		}
	}
	else
	{
		BeefOFF();
	}
}




