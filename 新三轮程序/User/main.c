/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-April-2011
  * @brief   Main program body
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
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "update.h"
#include "debug.h"
#include "string.h"
#include <stdlib.h>
#include "elmo.h"
#include "user.h"
#include "update.h"
#include "MotionCard.h"

//����ģʽ
#define NORMAL_MODE 1
//�ɼ�ģʽ
#define SAMPLING_MODE 2



static int  mode = 1;

_Bool GyroscopeFlag = 0;


void Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	

	//CAN1��ʼ�������ڵ������
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9); 
	
	
	//�����ǽ��մ���
	GyroscopeUsartInit(115200);
	

	//��������
	WirelessBluetoothUsartInit(115200);


	
	//��������ʼ��
	MyBeefInit();
	

	
	//�г̿���
	MySwitchInit();
	
	
	//��ʱ����ʼ�� 10ms
	TIM_Init(TIM2, 999, 839, 3, 3);
	
	//��ʼ���ڴ��
	BufferZizeInit(500);
	
	BeefON();
	
  TIM_Delayms(TIM5, 1000);
	
	BeefOFF();
	
	while(GyroscopeFlag == 0)
	{
		//����ȴ������ǳ�ʼ���ڼ䷢������������������в���ģʽ
		if(ReadSwitch() == 1)
		{
				BeefON();
				mode = SAMPLING_MODE;
		}
		else
		{
				BeefOFF();
		}
	}


	/*********************MOTOR DRIVE INTI**************************/

if(mode == NORMAL_MODE)
{
		elmo_Enable(1);
		elmo_Enable(2);
	  elmo_Enable(3);
		
		Vel_cfg(1,13000,13000);
		Vel_cfg(2,13000,13000);
		Vel_cfg(3,13000,13000);

}

//		mode = 4;
	/*********************TIMER INIT*******************************/
		BeefON();

}






extern _Bool USARTSEND;
int main(void)
{
	u32 numCount = 0;
	//��ʼ������
	Init();
	
	while(1)
	{
		if(ReadSwitch() == 1)
		{
			numCount++;
			if(numCount > 1000)
			{
				break;
			}
		}
		else
		{
			numCount = 0;
		}
	}
	BeefOFF();
	while(1)
	{
		if(ReadSwitch() == 0)
		{
			numCount++;
			if(numCount > 2000)
			{
				break;
			}
		}
		else
		{
			numCount = 0;
		}
	}
	
	while(1)
	{
		

	switch(mode)
		{
			
			case NORMAL_MODE:

					ReadFlashPointInformation();
//						numCount = GetCount();
//						USART_OUT(SENDUSART,(uint8_t *)"%d\r\n",numCount);	
//						for(int i = 1;i <= numCount;i++)
//						{
//							USART_OUT(SENDUSART,(uint8_t *)"%d\t %d\t %d\t %d\t %d\t %d\t %d\t\r\n",
//							(int)GetRingBufferPoint(i).x,(int)GetRingBufferPoint(i).y,(int)GetRingBufferPointAngle(i),
//							(int)GetRingBufferPointPoseAngle(i),(int)GetRingBufferPointLen(i),(int)GetRingBufferAverCurvature(i),(int)GetRingBufferPointVell(i));	
//						}
					mode = 3;
				break;
			
			//�ɼ�ģʽ
			case SAMPLING_MODE:
				
				PoseSampling();
			
				if(ReadSwitch() == 1)
				{
					static int timeCount = 0;
					
					timeCount++;
					
					//����г̿��ذ���һ��ʱ�䣬������¼����Ϣд��flash
					if(timeCount > 2000)
					{						

						PoseSamplingDone();
//						for(int i = 1;i <= numCount;i++)
//						{
//							USART_OUT(SENDUSART,(uint8_t *)"%dx%dy%da",																								
//							(int)GetRingBufferPoint(i).x,(int)GetRingBufferPoint(i).y,(int)GetRingBufferPointAngle(i));
//						}
						
//						USART_OUT(SENDUSART,(uint8_t *)"%d\r\n",numCount);	
//						for(int i = 1;i <= numCount;i++)
//						{
//							USART_OUT(SENDUSART,(uint8_t *)"%d\t %d\t %d\t %d\t %d\t %d\t %d\t\r\n",
//							(int)GetRingBufferPoint(i).x,(int)GetRingBufferPoint(i).y,(int)GetRingBufferPointAngle(i),
//							(int)GetRingBufferPointPoseAngle(i),(int)GetRingBufferPointLen(i),(int)GetRingBufferAverCurvature(i),(int)GetRingBufferPointVell(i));	
//						}
							
						BeefON();
						while(1);
					}
				}

				break;
				
				
			case 3:
//					ThreeWheelVellControl(200,90,0);
						PathFollowing(1);
			
//						ActThreeVell vell;
						
			
			
						//vell = GetThreeWheelVell(GetPoseVell().vell,GetPoseVell().angle,GetPoseVell().angularVell);
						
//						USART_OUT(SENDUSART,(uint8_t *)"%d\t%d\t%d\t%d\t%d\t%d\t\r\n",\
//						(int)Pulse2Vel(Get_Speed_Value(1)),(int)Pulse2Vel(Get_Speed_Value(2)),\
//						(int)Pulse2Vel(Get_Speed_Value(3)),(int)vell.v1,(int)vell.v2,(int)vell.v3);
	
			
						if(USARTSEND == 1)
						{
//							ActionRequestMotorInfo(1,VELOCITY);
//							ActionRequestMotorInfo(2,VELOCITY);
//							ActionRequestMotorInfo(3,VELOCITY);
//							USART_OUT_XYAngle();
							USARTSEND = 0;
						}
				break;
			
						
			
//			//����ģʽ
//			case 4:
//					for(int ipoint = 0;ipoint < NUM;ipoint++)
//					{
//						PutRingBuffer(passPose[ipoint]);
//					}
//					
//					SetLength(passPose[NUM - 1].length);
//					
//					mode = 5;
//				break;
//			
//			
//			case 5:
//						PathFollowing(GetPosPresent(),1);
//			
//						if(USARTSEND == 1)
//						{
//							
//							USART_OUT_XYAngle();

//							USARTSEND = 0;
//						}
//				break;
			
			
			default:
				
				break;
		
		}
	
	}


}



