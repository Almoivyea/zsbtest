#include "ChassisSolution.h"

#include "stdio.h"
#include "math.h"

#include "qaq_user.h"
#include "qaq_lib.h"
#include "can_qaq.h"
#include "uart_qaq.h"
#include "pid_qaq.h"
#include "WitIMU.h"
#include "remote_control.h"

/*
向前为X轴正方向, 向左为Y轴正方向, 底盘旋转逆时针为正

	//轮子布局:
	// 0--3  //头部
	// 1--2 

上坡扭矩补偿: 
	1.低配版,只解算到底盘需要提供的重力分量补偿
	2.高配版,对每个轮向电机进行牵引力6的再分配


*/
float TargetDate[3]; 

float AC_IMU_err ;

#define	MaxTarSpd_X	1.5f //m/s
#define	MaxTarSpd_Y	1.5f //m/s
float TopRate_SET = 30.0f; // 设置小陀螺的转动角速度, 度每秒

float* RC_MapingToChassisTarSpd(void) //将遥控器值映射为底盘目标平动速度,角速度
{	
//	static float TargetDate[3]; 
	 //前X, 左Y, 上Z , 第一象限为0号轮 
	float Vx_target = -(Move_ch1)/660.0f*(ReductionRatio3508)*(MaxTarSpd_X); //最高 (x) m/s, 
	float Vy_target =  (Move_ch2)/660.0f*(ReductionRatio3508)*(MaxTarSpd_Y); 
	float Omega = 0;

	AC_IMU_err = WitIMU_AngErrCal(WitDate[2], C_Board_IMU_Value[8]); 
	
	if(ChassisMode == top) { 
		Omega = TopRate_SET; 
	}else if(ChassisMode == follow)
	{
		
		Omega = AC_IMU_err*15; 
		
	}
	
	TargetDate[0] = Vx_target; // m/s
	TargetDate[1] = Vy_target; // m/s
	TargetDate[2] = Omega; 	   // rad/s 此处为弧度每秒,
	
	return TargetDate; 
}

float* Chassis_InverseSolution (float TarVx, float TarVy, float TarOmege) //将整个底盘被要求的速度等参数, 解算为电机的目标速度 (底盘逆解算) 底盘正解算
{
	static float WheelTarSpd_rpm[4]; 
	float resolved_Vx, resolved_Vy; //根据小陀螺变换后的Vx,Vy
		/*Attention: 此公式计算出的值为角速度值
			a: 轮间长的1/2
			b: 轮间宽的1/2
			s: 轮的半径	 */		
	static float a_m = (1.0f/2.0f) * LengthBetweenWheel*(0.001f); //以底盘的中心为原点, X轴方向的长度, 单位转换为m
	static float b_m = (1.0f/2.0f) * WidthBetweenWheel*(0.001f);	//以底盘的中心为原点, Y轴方向的长度, 单位转换为m
	static float s_m = WheelRadius*(0.001f); 	//轮子的半径, 单位转换为m 
	
	//way1:
	float Imu_Rad = DEG_TO_RAD((double)( AC_IMU_err )); //此处应该输入底盘与云台正方向之间的夹角  AC_IMU_err   WitDate[2] 
	resolved_Vx = TarVx*cos(Imu_Rad) + TarVy*sin(Imu_Rad); 
	resolved_Vy = (-TarVx)*sin(Imu_Rad) + TarVy*cos(Imu_Rad);
//	resolved_Vx = TarVx*cos(0) + TarVy*sin(0); 
//	resolved_Vy = (-TarVx)*sin(0) + TarVy*cos(0);

	
	
	WheelTarSpd_rpm[0] = ((-resolved_Vx + resolved_Vy + TarOmege*(a_m+b_m))/s_m) ; 
	WheelTarSpd_rpm[1] = ((-resolved_Vx - resolved_Vy + TarOmege*(a_m+b_m))/s_m) ; 
	WheelTarSpd_rpm[2] = (( resolved_Vx - resolved_Vy + TarOmege*(a_m+b_m))/s_m) ; 
	WheelTarSpd_rpm[3] = (( resolved_Vx + resolved_Vy + TarOmege*(a_m+b_m))/s_m) ; 
	
	// rad/s转rpm
	for(int i=0;i<4;i++)
	{
		WheelTarSpd_rpm[i] = (float)RADS_TO_RPM(WheelTarSpd_rpm[i]); 
	}
	
	return WheelTarSpd_rpm; //得出的时角速度, 单位rpm, 用于底盘速度环的目标值
	
}







 //下面的目前未用到
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

 //已知每个电机的状态, 我想知道整个底盘到底处于一个什么样的状态
float* Chassis_PositiveSolution (float Omega0, float Omega1, float Omega2, float Omega3) //由底盘四个电机的当前电机速度,得底盘的总平动速度与角速度
{
	static float TheCalculatedData[3]; 
	
	static float a = (1.0f/2.0f) * LengthBetweenWheel*(0.001f);//以底盘的中心为原点, X轴方向的长度, 单位转换为m
	static float b = (1.0f/2.0f) * WidthBetweenWheel*(0.001f);	//以底盘的中心为原点, Y轴方向的长度, 单位转换为m
	static float s = WheelRadius*(0.001f); 	//轮子的半径, 宏定义的单位为mm,此处*(0.001f), 单位转换为m 
	
	float Vx = ((-Omega0) - Omega1 + Omega2 + Omega3)*s/4; 
	float Vy = ( Omega0 - Omega1 - Omega2 + Omega3)*s/4; 
	float W  = ( Omega0 + Omega1 + Omega2 + Omega3)/(a+b)*s/4; 
		
	TheCalculatedData[0] = Vx ; 
	TheCalculatedData[1] = Vy ; 
	TheCalculatedData[2] = W ; 

	
	return TheCalculatedData;
	

}

float* Chassis_Dynamics_Inverse_Solution(float Fx, float Fy, float T) //动力学逆解算
{
	static float TheCalculatedData[4]; 
//	float resolved_Vx, resolved_Vy; //根据小陀螺变换后的Vx,Vy
		/*Attention: 此公式计算出的值为角速度值
			a: 轮间长的1/2
			b: 轮间宽的1/2
			s: 轮的半径	 */		
	static float a_m = (1.0f/2.0f) * LengthBetweenWheel*(0.001f); //以底盘的中心为原点, X轴方向的长度, 单位转换为m
	static float b_m = (1.0f/2.0f) * WidthBetweenWheel*(0.001f);	//以底盘的中心为原点, Y轴方向的长度, 单位转换为m
	static float s_m = WheelRadius*(0.001f); 	//轮子的半径, 单位转换为m 
	
//	float Imu_Rad = DEG_TO_RAD((double)(  User_ImuDate(qaq_Yaw) )); 
//	resolved_Vx = TarVx*cos(Imu_Rad) + TarVy*sin(Imu_Rad); 
//	resolved_Vy = (-TarVx)*sin(Imu_Rad) + TarVy*cos(Imu_Rad); 
//	
	TheCalculatedData[0] = -Fx/4*s_m +  Fy/4*s_m + T/4/(a_m+b_m)*s_m; 
	TheCalculatedData[1] = -Fx/4*s_m + -Fy/4*s_m + T/4/(a_m+b_m)*s_m; 
	TheCalculatedData[2] =  Fx/4*s_m + -Fy/4*s_m + T/4/(a_m+b_m)*s_m; 
	TheCalculatedData[3] =  Fx/4*s_m +  Fy/4*s_m + T/4/(a_m+b_m)*s_m; 
//	
//	// rad/s转rpm
//	for(int i=0;i<4;i++)
//	{                                                                                                                                                  
//		TheCalculatedData[i] = (float)RADS_TO_RPM(TheCalculatedData[i]); 
//	}
	
	return TheCalculatedData; //得出的时角速度, 单位rpm, 用于底盘速度环的目标值
	
}


//float *CompensatingTorqueResolving(void)//补偿力矩解算
//{
//	static float CompensatingTorque[4] = {0, 0, 0, 0}; //底盘各电机的补偿力矩
//	
//	//输入: 底盘imu Pitch, Roll, 整车总重力
//	//输出: 解算后的底盘各需要补偿的力矩, 储存于CompensatingTorque
//	
//	
//	
//	
//	return CompensatingTorque; 
//}


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////










