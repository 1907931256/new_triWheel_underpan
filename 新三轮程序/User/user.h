#ifndef _USER_H
#define _USER_H

//用户需要给定一个三轮的控制程序,具体请看用户手册
//speed 单位m/s
//angle 速度的方向
//angularVell 旋转速度 单位弧度/s
extern void ThreeWheelVellControl(float speed, float angle, float angularVell);

//ACCMAX摩擦力和电机能提供的最大加速度 mm/s^2
#define ACCMAX 179.0f
//VELLMAX机器人能达到的最大速度 mm/s  该值是通过比较摩擦力提供的最大加速度与电机能提供给机器人的最大加速度，取最小值。
#define VELLMAX  600.0f












#endif
