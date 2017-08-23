/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   dubug.h
*Author：      Peng Xu
*Date：        2016/10/21
*Description： debug.c的头文件
*
*Version：     V1.0
*
********************************************************************/


#ifndef _DEBUG_H
#define _DEBUG_H


#define SENDUSART  USART2
#include "usart.h"





/*********************************************************************
* @name 	USART_OUT_XYAngle
* @brief  	发送x，y坐标和zangle ，另外全局变量便于debug
* @param  	无
* @retval 	无
********************************************************************/
void USART_OUT_XYAngle(void);

void UsartSendXP3(int a,int b,int c);
void UsartSendXP2(int a,int b);
void UsartSendXP1(int a);

#endif
