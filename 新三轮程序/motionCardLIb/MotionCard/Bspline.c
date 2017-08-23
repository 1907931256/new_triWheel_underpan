#include "Bspline.h"
#include "math.h"
#include "ringBuffer.h"



/*********************************************************************************
* @name 	CaculateBsplineLen
* @brief    用于计算两点确定的一条B样条曲线的长度
* @param	point1 起始点				单位 mm
* @param	point2 结束点				单位 mm
* @param	angle1 起始点方向    单位 度
* @param	angle2 结束点方向    单位 度
* @retval   返回该样条曲线长度  单位 mm
*********************************************************************************/
float CaculateBsplineLen(Point_t point1, Point_t point2, float angle1, float angle2)
{

		//常数项
		Point_t P[2];
		//插值点
		//Point_t *passPoint = NULL;
		//系数矩阵的逆矩阵
		float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.0000f, 0.333333333f } };
		//控制到达切矢方向的提前量
		float length = 0.0f;
		//零时控制点坐标
		Point_t dataPoint[2];
		//最终控制点坐标
		Point_t finalDataPoint[6];
		Point_t tempPoint;
		Point_t tempPointOld;
		int i, j;
		//该B样条曲线长度
		float u, b1, b2, b3, b0;



		length = 0.60f*sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));

		//对常数项赋值    angle加90是为了处理陀螺仪角度与坐标系角度不符
		P[0].x = 3 * point1.x + length*cos((angle1) * CHANGE_TO_RADIAN);
		P[0].y = 3 * point1.y + length*sin((angle1) * CHANGE_TO_RADIAN);
		P[1].x = 3 * point2.x - length*cos((angle2) * CHANGE_TO_RADIAN);
		P[1].y = 3 * point2.y - length*sin((angle2) * CHANGE_TO_RADIAN);

		//清零
		for (i = 0; i < 2; i++)
		{
			dataPoint[i].x = 0;
			dataPoint[i].y = 0;
		}


		//d = inv(a).*P  求临时控制点坐标
		for (i = 0; i < 2; i++)
		{
			for (j = 0; j < 2; j++)
			{
				dataPoint[i].x += invA[i][j] * P[j].x;
				dataPoint[i].y += invA[i][j] * P[j].y;
			}
		}

		//复制最终控制点坐标
		finalDataPoint[0] = point1;
		finalDataPoint[1] = point1;
		finalDataPoint[2] = dataPoint[0];
		finalDataPoint[3] = dataPoint[1];
		finalDataPoint[4] = point2;
		finalDataPoint[5] = point2;


		length = 0.0f;
		//将B样条上第一点到首端点的值加上
		tempPointOld = point1;
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 50; j++)
			{
				u = (float)(j)* 0.02f;
				// 基函数b0
				b0 = (1.0f / 6.0f) * pow(1.0f - u, 3);
				// 基函数b1
				b1 = (1.0f / 6.0f) * (3.0f * pow(u, 3) - 6 * pow(u, 2) + 4);
				// 基函数b2；
				b2 = (1.0f / 6.0f) * (-3.0f * pow(u, 3) + 3 * pow(u, 2) + 3 * u + 1);
				// 基函数b3；
				b3 = (1.0f / 6.0f) * pow(u, 3);

				tempPoint.x = b0*finalDataPoint[i].x + b1*finalDataPoint[i + 1].x + b2*finalDataPoint[i + 2].x + b3*finalDataPoint[i + 3].x;
				tempPoint.y = b0*finalDataPoint[i].y + b1*finalDataPoint[i + 1].y + b2*finalDataPoint[i + 2].y + b3*finalDataPoint[i + 3].y;

				length += CalculatePoint2PointDistance(tempPoint,tempPointOld);

				tempPointOld = tempPoint;
			}
		}
		//将B样条上最后一点到末端点的距离加上
		length += CalculatePoint2PointDistance(tempPoint, point2);
		
		return length;
	}


	
	
/*********************************************************************************
* @name 	SerchVirtualPoint
* @brief  根据距离起点的路径长度确定所对应的虚拟点
* @param	robotLen 距离起点的路径长度  单位 mm 
* @retval 返回 虚拟点坐标 和相应在所在样条曲线上的比例系数
*********************************************************************************/
PointU_t SerchVirtualPoint(float robotLen)
{
	int i = 0;
	PointU_t result;

	//确定目标长度在哪一段上
	for (i = 1; i < GetCount(); i++)
	{
		if (robotLen < GetRingBufferPointLen(i))
			break;
	}
	
	
	
	//确定落在第i-1 到 i 上的距离
	robotLen -= GetRingBufferPointLen(i-1);
	
	result = SerchBsplineVirtualPoint(GetRingBufferPoint(i - 1), GetRingBufferPoint(i), GetRingBufferPointAngle(i - 1), GetRingBufferPointAngle(i), robotLen);

	if((i-2) > 0)
	{
			DeleteData(i-2);
	}
	return result;

}



/*********************************************************************************
* @name 	SerchBsplineVirtualPoint
* @brief  根据距离所在样条曲线长度确定所对应的虚拟点
* @param	robotLen 距离所在样条曲线长度  单位 mm
* @retval 返回 虚拟点坐标 和相应在所在样条曲线上的比例系数
*********************************************************************************/
PointU_t SerchBsplineVirtualPoint(Point_t point1, Point_t point2, float angle1, float angle2,float robotlen)
{
	//常数项
	Point_t P[2];
	//系数矩阵的逆矩阵
	float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.0000f, 0.333333333f } };
	//控制到达切矢方向的提前量
	float length = 0.0f;
	//零时控制点坐标
	Point_t dataPoint[2];
	//最终控制点坐标
	Point_t finalDataPoint[6];
	Point_t tempPoint;
	Point_t tempPointOld;
	int i, j;
	//该B样条曲线长度
	float u, b1, b2, b3, b0;
	PointU_t result;



	length = 0.60f*sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));

	//对常数项赋值    angle加90是为了处理陀螺仪角度与坐标系角度不符
	P[0].x = 3 * point1.x + length*cos((angle1) * CHANGE_TO_RADIAN);
	P[0].y = 3 * point1.y + length*sin((angle1) * CHANGE_TO_RADIAN);
	P[1].x = 3 * point2.x - length*cos((angle2) * CHANGE_TO_RADIAN);
	P[1].y = 3 * point2.y - length*sin((angle2) * CHANGE_TO_RADIAN);

	//清零
	for (i = 0; i < 2; i++)
	{
		dataPoint[i].x = 0;
		dataPoint[i].y = 0;
	}


	//d = inv(a).*P  求临时控制点坐标
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			dataPoint[i].x += invA[i][j] * P[j].x;
			dataPoint[i].y += invA[i][j] * P[j].y;
		}
	}


	//复制最终控制点坐标
	finalDataPoint[0] = point1;
	finalDataPoint[1] = point1;
	finalDataPoint[2] = dataPoint[0];
	finalDataPoint[3] = dataPoint[1];
	finalDataPoint[4] = point2;
	finalDataPoint[5] = point2;


	length = 0.0f;
	//将B样条上第一点到首端点的值加上
	tempPointOld = point1;
	int num = 0;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 50; j++)
		{
			//用于计算U值
			num++;

			u = (float)(j)* 0.02f;

			// 基函数b0
			b0 = (1.0f / 6.0f) * pow(1.0f - u, 3);
			// 基函数b1
			b1 = (1.0f / 6.0f) * (3.0f * pow(u, 3) - 6 * pow(u, 2) + 4);
			// 基函数b2；
			b2 = (1.0f / 6.0f) * (-3.0f * pow(u, 3) + 3 * pow(u, 2) + 3 * u + 1);
			// 基函数b3；
			b3 = (1.0f / 6.0f) * pow(u, 3);

			tempPoint.x = b0*finalDataPoint[i].x + b1*finalDataPoint[i + 1].x + b2*finalDataPoint[i + 2].x + b3*finalDataPoint[i + 3].x;
			tempPoint.y = b0*finalDataPoint[i].y + b1*finalDataPoint[i + 1].y + b2*finalDataPoint[i + 2].y + b3*finalDataPoint[i + 3].y;

			length += CalculatePoint2PointDistance(tempPoint, tempPointOld);
			if (robotlen < length)
			{
				result.point = tempPoint;
				result.u = (float)num / 150.0f;
				return result;
			}

			tempPointOld = tempPoint;
		}
	}

	//将B样条上最后一点到末端点的距离加上
	result.point = point2;

	result.u = 1.0f;

	return result;
}

//确定虚拟目标点
PointU_t SerchVirtualPoint2(float robotLen)
{
	int i = 0;
	PointU_t result;

	//确定目标长度在哪一段上
	for (i = 1; i < GetCount(); i++)
	{
		if (robotLen < GetRingBufferPointLen(i))
			break;
	}
	
	
	
	//确定落在第i-1 到 i 上的距离
	robotLen -= GetRingBufferPointLen(i-1);
	
	result = SerchBsplineVirtualPoint(GetRingBufferPoint(i - 1), GetRingBufferPoint(i), GetRingBufferPointAngle(i - 1), GetRingBufferPointAngle(i), robotLen);

	return result;

}

