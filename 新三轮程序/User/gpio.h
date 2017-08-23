#ifndef __GPIO_H

#define __GPIO_H

#include "stm32f4xx_gpio.h"

void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void MyBeefInit(void);

//行程开关初始化
void MySwitchInit(void);


void BeefON(void);

void BeefOFF(void);

//返回1 代表触发
//返回0 代表未触发
_Bool ReadSwitch(void);

void LedTwinkle(void);

#endif
