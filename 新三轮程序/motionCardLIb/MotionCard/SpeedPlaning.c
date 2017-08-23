#include "SpeedPlaning.h"
#include <stdlib.h>
#include "MotionCard.h"
#include "ringBuffer.h"
#include "math.h"


//通过配置的轮子最大加速度进行降速
//适当比例的降速，算完后记得把最新速度数据放在ringbuffer里
//wheelOne 一号轮速度数组首地址
//wheelTwo 	二号轮速度数组首地址
//wheelThree 三号轮速度数组首地址
void DynamicalAjusting(float* wheelOne, float* wheelTwo, float* wheelThree)
{
	float time = 0.0f;
	int n = GetCount();
	float tempAcc = 0.0f;

	//每次加速度降低至上次的百分值
	float percent = 0.9;

	//先正向削减速度
	for (int i = 2; i < n + 1; i++)
	{

		//粗略计算每两示教点之间的运动的时间
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;
//轮1
		//只处理速度同向的情况
		if (wheelOne[i - 1] * wheelOne[i - 2] > 0)
		{
			tempAcc = (fabs(wheelOne[i - 1]) - fabs(wheelOne[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc > GetAccMax() )
		{
			//每次削减0.05的加速度
			wheelOne[i - 1] = wheelOne[i - 1] > 0 ? wheelOne[i - 2] + tempAcc*percent * time : wheelOne[i - 2] - tempAcc*percent * time;
		}
//轮2
		//只处理速度同向的情况
		if (wheelTwo[i - 1] * wheelTwo[i - 2] > 0)
		{
			tempAcc = (fabs(wheelTwo[i - 1]) - fabs(wheelTwo[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc > GetAccMax())
		{
			//每次削减0.05的加速度
			wheelTwo[i - 1] = wheelTwo[i - 1] > 0 ? wheelTwo[i - 2] + tempAcc*percent * time : wheelTwo[i - 2] - tempAcc*percent * time;
		}
//轮3
		//只处理速度同向的情况
		if (wheelThree[i - 1] * wheelThree[i - 2] > 0)
		{
			tempAcc = (fabs(wheelThree[i - 1]) - fabs(wheelThree[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc > GetAccMax())
		{
			//每次削减0.05的加速度
			wheelThree[i - 1] = wheelThree[i - 1] > 0 ? wheelThree[i - 2] + tempAcc*percent * time : wheelThree[i - 2] - tempAcc*percent * time;
		}
	}

	//反向削减速度
	for (int i = n; i > 0; i--)
	{

		//粗略计算每两示教点之间的运动的时间
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;
		//轮1
		//只处理速度同向的情况
		if (wheelOne[i - 1] * wheelOne[i - 2] > 0)
		{
			tempAcc = (fabs(wheelOne[i - 1]) - fabs(wheelOne[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc < -GetAccMax())
		{
			//每次削减0.05的加速度
			wheelOne[i - 2] = wheelOne[i - 2] > 0 ? wheelOne[i - 1] - tempAcc*percent * time : wheelOne[i - 1] + tempAcc*percent * time;
		}


		//轮2
		//只处理速度同向的情况
		if (wheelTwo[i - 1] * wheelTwo[i - 2] > 0)
		{
			tempAcc = (fabs(wheelTwo[i - 1]) - fabs(wheelTwo[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc < -GetAccMax())
		{
			//每次削减0.05的加速度
			wheelTwo[i - 2] = wheelTwo[i - 2] > 0 ? wheelTwo[i - 1] - tempAcc*percent * time : wheelTwo[i - 1] + tempAcc*percent * time;
		}


		//轮3
		//只处理速度同向的情况
		if (wheelThree[i - 1] * wheelThree[i - 2] > 0)
		{
			tempAcc = (fabs(wheelThree[i - 1]) - fabs(wheelThree[i - 2])) / time;
		}
		else
		{
			tempAcc = 0.0f;
		}

		if (tempAcc < -GetAccMax())
		{
			//每次削减0.05的加速度
			wheelThree[i - 2] = wheelThree[i - 2] > 0 ? wheelThree[i - 1] - tempAcc*percent * time : wheelThree[i - 1] + tempAcc*percent * time;
		}
	}



	for (int i = 0; i < n - 1; i++)
	{
		TriWheelVel_t tempTrueVell;
		tempTrueVell.v1 = wheelOne[i];
		tempTrueVell.v2 = wheelTwo[i];
		tempTrueVell.v3 = wheelThree[i];
		float vellCar;

		float angErr = GetRingBufferPointPoseAngle(i+2) - GetRingBufferPointPoseAngle(i + 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;
		//粗略计算每两示教点之间的运动的时间
		time = (GetRingBufferPointLen(i+2) - GetRingBufferPointLen(i + 1)) / (GetRingBufferPointVell(i+2) + GetRingBufferPointVell(i + 1)) * 2;
		
		//通过DecreseVellByOneWheel函数选择三个轮子中速度最大者进行降速
		if (fabs(tempTrueVell.v1) > fabs(tempTrueVell.v2) && fabs(tempTrueVell.v1) > fabs(tempTrueVell.v3))
		{
			vellCar = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 1, tempTrueVell.v1);
		}
		else if (fabs(tempTrueVell.v2) > fabs(tempTrueVell.v1) && fabs(tempTrueVell.v2) > fabs(tempTrueVell.v3))
		{
			vellCar = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 2, tempTrueVell.v2);
		}
		else
		{
			vellCar = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 3, tempTrueVell.v3);
		}


		//将计算的最新合速度放入缓存池中
		SetRingBufferPointVell(i + 1, vellCar);

	}

}

//通过ringBuffer里的数据计算每一点处三个轮子的速度
//目的更新wheelOne wheelTwo wheelThree这三个数组里的三轮速度，便于下一次的速度削减
//wheelOne 一号轮速度数组首地址
//wheelTwo 	二号轮速度数组首地址
//wheelThree 三号轮速度数组首地址
void CalculateThreeWheelVell(float* wheelOne,float* wheelTwo,float* wheelThree)
{
	//分解到三个轮对全局速度进行规划
	TriWheelVel_t threeVell;
	float n = GetCount();

	wheelOne[0] = wheelTwo[0] = wheelThree[0] = 0; 

	for (int i = 2; i < n + 1; i++)
	{
		float angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;

		float time = 0.0f;

		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

		float rotationVell = angErr / time;


		threeVell = CaculateThreeWheelVel(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell, GetRingBufferPointPoseAngle(i));


		wheelOne[i - 1] = threeVell.v1;

		wheelTwo[i - 1] = threeVell.v2;

		wheelThree[i - 1] = threeVell.v3;

	}
}





//通过降低合速度保证某轮的速度要求
//vellCar 降速前的前进合速度 单位 mm/s
//orientation 速度朝向 单位 度
//rotationalVell 旋转速度 单位 度每秒
//wheelNum  所降速的轮号 
// targetWheelVell   所降速的目标
// 返回所降低后的合速度
float DecreseVellByOneWheel(float vellCar, float orientation, float rotationalVell,float zAngle, int wheelNum, float targetWheelVell)
{
	TriWheelVel_t vell;
	int i;
	switch (wheelNum)
	{
		case 1:
			//每次合速度乘0.9,直到满足一号轮速度降低至目标速度。对于一些不能满足的，循环50次后自动退出
			for (i = 0; i < 50; i++)
			{

				vellCar *= 0.9f;
				vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
				if (fabs(vell.v1) < fabs(targetWheelVell))
				{
					break;
				}
			}
		break;
		
		case 2:
			for (i = 0; i < 50; i++)
			{

				vellCar *= 0.9f;
				vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
				if (fabs(vell.v2) < fabs(targetWheelVell))
				{
					break;
				}
			}
		break;

		case 3:
			for (i = 0; i < 50; i++)
			{

				vellCar *= 0.9f;
				vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
				if (fabs(vell.v3) < fabs(targetWheelVell))
				{
					break;
				}
			}
			break;
	}
	return vellCar;
}



#define MIN_VELL 50
//速度规划函数
void SpeedPlaning()
{
	float* vell = NULL;
	float* curvature = NULL;
	float* wheelOne = NULL;
	float* wheelTwo = NULL;
	float* wheelThree = NULL;


	int n = GetCount();


	vell = (float *)malloc(n*sizeof(float));
	curvature = (float *)malloc(n*sizeof(float));
	wheelOne = (float *)malloc(n*sizeof(float));
	wheelTwo = (float *)malloc(n*sizeof(float));
	wheelThree = (float *)malloc(n*sizeof(float));


	//向左移两个单位，提前反应
	for (int i = 0; i < n-1; i++)
	{
		curvature[i] = GetRingBufferAverCurvature(i + 3);
	}


	curvature[n - 2] = (curvature[n - 3] + curvature[n - 4]) / 2.0f;
	curvature[n - 1] = (curvature[n - 2] + curvature[n - 3]) / 2.0f;


	//电机最大速度能够满足的最小曲率半径
	float curvatureMaxVell = GetVelMax() * GetVelMax() / (2*GetAccMax());


	//将过大的曲率半径全设为能最大速度能满足的最小曲率半径
	for (int i = 0; i < n; i++)
	{
		if (curvature[i] > curvatureMaxVell)
		{
			curvature[i] = curvatureMaxVell;
		}
	}



	//通过曲率半径计算该段能满足的最大速度                                         
	for (int i = 0; i < n; i++)
	{
		vell[i] = sqrt((2*GetAccMax()) * curvature[i]);
	}


	vell[0] = 250;
	vell[n - 1] = 100;


	float tempVell = 0.0f;
	//通过v2^2 - v1^2 = 2*a*s对速度再次规划
	for (int i = 0; i < n - 1; i++)
	{
		if (vell[i + 1] > vell[i])
		{
			tempVell = sqrt(2 * (2*GetAccMax()) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);
			if (tempVell < vell[i + 1])
			{
				vell[i + 1] = tempVell;
			}
		}
	}

	for (int i = n-1; i > 0; i--)
	{
		if (vell[i - 1] > vell[i])
		{
			tempVell = sqrt(2 * (2 * GetAccMax()) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + vell[i] * vell[i]);
			if (tempVell < vell[i - 1])
			{
				vell[i - 1] = tempVell;
			}
		}
	}

	//将暂时规划的速度放入环形数组里
	for (int i = 0; i < n; i++)
	{
		SetRingBufferPointVell(i + 1, vell[i]);
	}


	//计算此时三个轮的速度
	CalculateThreeWheelVell(wheelOne, wheelTwo, wheelThree);

	int jishu = 0;
	//动态的对速度进行平衡
	while (1)
	{
		int ipoint = 0;

		for (ipoint = 2; ipoint < n + 1; ipoint++)
		{
			float time = 0.0f;

			time = (GetRingBufferPointLen(ipoint) - GetRingBufferPointLen(ipoint - 1)) / (GetRingBufferPointVell(ipoint) + GetRingBufferPointVell(ipoint - 1)) * 2;

			//如果判断某一个轮子加速度大于最大加速度时，进行调节
			if ((((wheelOne[ipoint - 1] - wheelOne[ipoint - 2]) / time > GetAccMax()) && (wheelOne[ipoint - 1] * wheelOne[ipoint - 2] > 0)) \
				|| (((wheelTwo[ipoint - 1] - wheelTwo[ipoint - 2]) / time > GetAccMax()) && (wheelTwo[ipoint - 1] * wheelTwo[ipoint - 2] > 0))\
				|| (((wheelThree[ipoint - 1] - wheelThree[ipoint - 2]) / time > GetAccMax()) && (wheelThree[ipoint - 1] * wheelThree[ipoint - 2] > 0)))
			{
				jishu ++;
				//平衡法规划速度
				DynamicalAjusting(wheelOne, wheelTwo, wheelThree);
				//将新获得的三轮速度放在数组里
				CalculateThreeWheelVell(wheelOne, wheelTwo, wheelThree);
				break;
			}
			for (int i = 1; i < n-1; i++)
			{
				wheelOne[i] = (wheelOne[i - 1] + wheelOne[i + 1]) / 2;
				wheelTwo[i] = (wheelTwo[i - 1] + wheelTwo[i + 1]) / 2;
				wheelThree[i] = (wheelThree[i - 1] + wheelThree[i + 1]) / 2;
			}

		}

		if (ipoint == n + 1)
		{
			//将初速度设为200
				SetRingBufferPointVell(1,MIN_VELL);
				//通过v2^2 - v1^2 = 2*a*s对速度再次规划
				for (int i = 1; i < n; i++)
				{
					if (GetRingBufferPointVell(i + 2) > GetRingBufferPointVell(i + 1))
					{
						tempVell = sqrt(2 * (1.0f * GetAccMax()) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
						if (tempVell < GetRingBufferPointVell(i+2))
						{
							SetRingBufferPointVell(i + 2,tempVell);
						}
					}
				}

				for (int i = n - 1; i > 0; i--)
				{
					if (GetRingBufferPointVell(i) > GetRingBufferPointVell(i+1))
					{
						tempVell = sqrt(2.0f * (1.0f * GetAccMax()) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
						if (tempVell < GetRingBufferPointVell(i))
						{
							SetRingBufferPointVell(i, tempVell);
						}
					}
				}
			//将速度小于最小速度的做处理
				for(int i = 2;i < n;i++)
				{
					if(GetRingBufferPointVell(i) < MIN_VELL)		
					{
						SetRingBufferPointVell(i,MIN_VELL);
					}						
				}
				free(wheelOne);
				free(wheelTwo);
				free(wheelThree);
				break;
		}
	}

}






