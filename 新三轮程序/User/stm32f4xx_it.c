/**
  ******************************************************************************
  * @file    I2C/EEPROM/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "usart.h"
#include "elmo.h"
#include "can.h"
#include "update.h"
#include "stm32f4xx_tim.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup I2C_EEPROM
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

extern _Bool USARTSEND;
long TIM2_COUNT = 0;
void TIM2_IRQHandler(void)
{

   
	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
	{

		 TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
				CaculatePath();
			TIM2_COUNT++;
			USARTSEND=1;
   }

}





//void USART1_IRQHandler(void)
//{

//	 if(USART_GetITStatus(USART1, USART_IT_RXNE)==SET)   
//	 {
//			USART_ClearITPendingBit( USART1,USART_IT_RXNE);
//	 	 
//	 }
//}


	static float pos_x  = 0;
	static float pos_y  = 0;
	static float zangle = 0;
	static float xangle = 0;
	static float yangle = 0;
	static float w_z    = 0;

//串口接收终端
//若标志位为1代表初始化完成，主函数停止等待
extern _Bool GyroscopeFlag;
void USART1_IRQHandler(void)
{


	static uint8_t ch;
	static uint8_t count = 0;
	static uint8_t i = 0;
	static union
    {
		uint8_t data[24];
		float ActVal[6];
    }posture;
		
	 if(USART_GetITStatus(USART1, USART_IT_RXNE)==SET)   
	 {
			USART_ClearITPendingBit( USART1,USART_IT_RXNE);
		    ch=USART_ReceiveData(USART1);
		switch(count)
		{
			case 0:
				if(ch == 0x0d)
					count++;
				else
					count = 0;
				break;
			 
			case 1:
				if(ch == 0x0a)
				{
					i = 0;
					count++;
				}
				else if(ch == 0x0d);
				else
					count = 0;
				break;
			 
			case 2:
				posture.data[i] = ch;
				i++;
				if(i >= 24)
				{
					i = 0;
					count++;
				}
				break;
			 
			case 3:
				if(ch == 0x0a)
					count++;
				else
					count=0;
				break;
			 
			case 4:
				if(ch == 0x0d)
				{
					zangle = posture.ActVal[0];
					xangle = posture.ActVal[1];
					yangle = posture.ActVal[2];
					pos_x  = posture.ActVal[3];
					pos_y  = posture.ActVal[4];
					w_z    = posture.ActVal[5];
					 
					xangle = xangle;
					yangle = yangle;
					pos_x  = pos_x ;
					pos_y  = pos_y ;
					w_z    = w_z   ;
					 
					SetPosx(pos_x);
					SetPosy(pos_y);
					SetAngleZ(zangle);
					GyroscopeFlag = 1;
				}
				count = 0;
				break;

			default:
				count = 0;
				break;		 
		}	 	 
	 }
}



void CAN1_RX0_IRQHandler(void)
{
//	union can_message receive_vel;   //驱动器位置变量
	static uint8_t buffer[8];
	static uint32_t StdId=0;
	if(CAN_MessagePending(CAN1,CAN_FIFO0)==0) return;		//if there is no data, get out of this function
	

	CAN_RxMsg(CAN1, &StdId,buffer,8);			//reveive data	
		
	if((StdId==0x281||StdId==0x282)&&(buffer[0]==0x56)&&(buffer[1]==0x58))     //get pos value
	{
//		receive_vel.data8[0]=buffer[4];
//		receive_vel.data8[1]=buffer[5];
//		receive_vel.data8[2]=buffer[6];
//		receive_vel.data8[3]=buffer[7];  //receive_pos4.data32_t
//		if(StdId==0x281) 
//			SetVelLft(receive_vel.data32_t);
//		if(StdId==0x282) 
//			SetVelRgt(receive_vel.data32_t);
	}
		CAN_ClearFlag(CAN1,CAN_FLAG_EWG);
		CAN_ClearFlag(CAN1,CAN_FLAG_EPV);
		CAN_ClearFlag(CAN1,CAN_FLAG_BOF);
		CAN_ClearFlag(CAN1,CAN_FLAG_LEC);
		
		CAN_ClearFlag(CAN1,CAN_FLAG_FMP0);
		CAN_ClearFlag(CAN1,CAN_FLAG_FF0);
		CAN_ClearFlag(CAN1,CAN_FLAG_FOV0);
		CAN_ClearFlag(CAN1,CAN_FLAG_FMP1);
		CAN_ClearFlag(CAN1,CAN_FLAG_FF1);
		CAN_ClearFlag(CAN1,CAN_FLAG_FOV1);
}
