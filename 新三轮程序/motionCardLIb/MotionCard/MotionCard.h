#ifndef _MOTIONCARD_H
#define _MOTIONCARD_H
#include "stm32f4xx.h"

//注意 示教模式下所存点存储在 0x08040000 地址中  stm32f407单片机



//点的结构体 单位mm
typedef struct
{
	float x;
	float y;
}Point_t;



typedef struct 
{
	float v1;
	float v2;
	float v3;
}TriWheelVel_t;



//点斜式结构体 ，斜率用角度制的角度代替
typedef struct
{
	Point_t point;
	//角度制
	float   direction;
}Pose_t;



//用户需要给定一个三轮的控制程序,具体请看用户手册
//vel       单位mm/s
//direction 速度的方向  单位 度
//rotationVell 旋转速度 单位 度/s 
extern void ThreeWheelVellControl(float vel, float direction, float rotationVell);


//返回三个轮子轮速
//vel       单位mm/s
//direction 速度的方向  单位 度
//rotationVell 旋转速度 单位 度/s 
//posAngle 机器人的姿态  单位 度
extern TriWheelVel_t CaculateThreeWheelVel(float vel, float direction, float rotationVell,float posAngle);


//ACCMAX摩擦力和电机能提供的最大加速度 mm/s^2
extern float GetAccMax(void);
//VELLMAX机器人能达到的最大速度 mm/s  该值是通过比较摩擦力提供的最大加速度与电机能提供给机器人的最大加速度，取最小值。
extern float GetVelMax(void);



//需要外部实现，返回坐标和姿态
extern float GetAngleZ(void);
extern float GetPosx(void);
extern float GetPosy(void);


//开辟内存，使用前必须首先调用  num 最大存取点个数，一个点占28字节  
void BufferZizeInit(int num);
//路径跟随函数 percent: 范围为0~1.1代表规划的最大速度
void PathFollowingNew(float percent);
//示教采点函数，采点时，需要一直调用
void FunSampling(void);
//读取flash里的示教点信息，并添加到ringBuffer里
void ReadFlashPointInformation(void);
//采点完成时需要调用，将所采点信息填入flash里
void FunsamplingOk(void);
//清空ringBuffer,一旦清空，底盘将会停车抱死
void ClearRingBuffer(void);


//本人调试所用
//int GetCount(void);
//float GetRingBufferPointPoseAngle(int num);
//float GetRingBufferPointAngle(int num);
//float GetRingBufferPointVell(int num);
//float GetRingBufferPointLen(int num);
//float GetRingBufferAverCurvature(int num);
//float GetLength(void);

#endif



