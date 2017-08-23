#include "threeWheelMove.h"
#include "elmo.h"
#include "math.h"
#include "MotionCard.h"
#include "debug.h"
static float Pulse2Vel(float pulse);
static float Vel2Pulse(float vel);

#define AFA 60.0f
#define ROBOT_L   231.5f




void VelControlTriWheel(float v1,float v2,float v3)
{
		VelCrl(1,Vel2Pulse(v3));
		VelCrl(2,Vel2Pulse(v2));
		VelCrl(3,Vel2Pulse(v1));
}


/**
  * @brief  ��׼��λ�ٶ�ת��Ϊ�����ٶ�
  * @param  vel:�ٶ� mm/s
  * @retval velpulse:�ٶ� ����/s
  */
float Vel2Pulse(float vel)
{
	float pulse = 0.0f;
	pulse = vel / (2.0f * PI * WHEELRADIUS) * STDPULSE * REDUCTION;
	return pulse;
}


/**
  * @brief  �����ٶ�ת��Ϊ��׼��λ�ٶ�
  * @param  pulse:�ٶ� ����/s
  * @retval vel:�ٶ� mm/s
  */
float Pulse2Vel(float pulse)
{
	float vel = 0.0f;
	vel = pulse * (2.0f * PI * WHEELRADIUS) / REDUCTION / STDPULSE;
	return vel;
}



float GetAccMax(void)
{
	return 170.0f;
}

float GetVelMax(void)
{
	return 700.0f;
}

float GetRobotRadius(void)
{
	return 231.5f;
}
