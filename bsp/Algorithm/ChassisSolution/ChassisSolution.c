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
��ǰΪX��������, ����ΪY��������, ������ת��ʱ��Ϊ��

	//���Ӳ���:
	// 0--3  //ͷ��
	// 1--2 

����Ť�ز���: 
	1.�����,ֻ���㵽������Ҫ�ṩ��������������
	2.�����,��ÿ������������ǣ����6���ٷ���


*/
float TargetDate[3]; 

float AC_IMU_err ;

#define	MaxTarSpd_X	1.5f //m/s
#define	MaxTarSpd_Y	1.5f //m/s
float TopRate_SET = 30.0f; // ����С���ݵ�ת�����ٶ�, ��ÿ��

float* RC_MapingToChassisTarSpd(void) //��ң����ֵӳ��Ϊ����Ŀ��ƽ���ٶ�,���ٶ�
{	
//	static float TargetDate[3]; 
	 //ǰX, ��Y, ��Z , ��һ����Ϊ0���� 
	float Vx_target = -(Move_ch1)/660.0f*(ReductionRatio3508)*(MaxTarSpd_X); //��� (x) m/s, 
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
	TargetDate[2] = Omega; 	   // rad/s �˴�Ϊ����ÿ��,
	
	return TargetDate; 
}

float* Chassis_InverseSolution (float TarVx, float TarVy, float TarOmege) //���������̱�Ҫ����ٶȵȲ���, ����Ϊ�����Ŀ���ٶ� (���������) ����������
{
	static float WheelTarSpd_rpm[4]; 
	float resolved_Vx, resolved_Vy; //����С���ݱ任���Vx,Vy
		/*Attention: �˹�ʽ�������ֵΪ���ٶ�ֵ
			a: �ּ䳤��1/2
			b: �ּ���1/2
			s: �ֵİ뾶	 */		
	static float a_m = (1.0f/2.0f) * LengthBetweenWheel*(0.001f); //�Ե��̵�����Ϊԭ��, X�᷽��ĳ���, ��λת��Ϊm
	static float b_m = (1.0f/2.0f) * WidthBetweenWheel*(0.001f);	//�Ե��̵�����Ϊԭ��, Y�᷽��ĳ���, ��λת��Ϊm
	static float s_m = WheelRadius*(0.001f); 	//���ӵİ뾶, ��λת��Ϊm 
	
	//way1:
	float Imu_Rad = DEG_TO_RAD((double)( AC_IMU_err )); //�˴�Ӧ�������������̨������֮��ļн�  AC_IMU_err   WitDate[2] 
	resolved_Vx = TarVx*cos(Imu_Rad) + TarVy*sin(Imu_Rad); 
	resolved_Vy = (-TarVx)*sin(Imu_Rad) + TarVy*cos(Imu_Rad);
//	resolved_Vx = TarVx*cos(0) + TarVy*sin(0); 
//	resolved_Vy = (-TarVx)*sin(0) + TarVy*cos(0);

	
	
	WheelTarSpd_rpm[0] = ((-resolved_Vx + resolved_Vy + TarOmege*(a_m+b_m))/s_m) ; 
	WheelTarSpd_rpm[1] = ((-resolved_Vx - resolved_Vy + TarOmege*(a_m+b_m))/s_m) ; 
	WheelTarSpd_rpm[2] = (( resolved_Vx - resolved_Vy + TarOmege*(a_m+b_m))/s_m) ; 
	WheelTarSpd_rpm[3] = (( resolved_Vx + resolved_Vy + TarOmege*(a_m+b_m))/s_m) ; 
	
	// rad/sתrpm
	for(int i=0;i<4;i++)
	{
		WheelTarSpd_rpm[i] = (float)RADS_TO_RPM(WheelTarSpd_rpm[i]); 
	}
	
	return WheelTarSpd_rpm; //�ó���ʱ���ٶ�, ��λrpm, ���ڵ����ٶȻ���Ŀ��ֵ
	
}







 //�����Ŀǰδ�õ�
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

 //��֪ÿ�������״̬, ����֪���������̵��״���һ��ʲô����״̬
float* Chassis_PositiveSolution (float Omega0, float Omega1, float Omega2, float Omega3) //�ɵ����ĸ�����ĵ�ǰ����ٶ�,�õ��̵���ƽ���ٶ�����ٶ�
{
	static float TheCalculatedData[3]; 
	
	static float a = (1.0f/2.0f) * LengthBetweenWheel*(0.001f);//�Ե��̵�����Ϊԭ��, X�᷽��ĳ���, ��λת��Ϊm
	static float b = (1.0f/2.0f) * WidthBetweenWheel*(0.001f);	//�Ե��̵�����Ϊԭ��, Y�᷽��ĳ���, ��λת��Ϊm
	static float s = WheelRadius*(0.001f); 	//���ӵİ뾶, �궨��ĵ�λΪmm,�˴�*(0.001f), ��λת��Ϊm 
	
	float Vx = ((-Omega0) - Omega1 + Omega2 + Omega3)*s/4; 
	float Vy = ( Omega0 - Omega1 - Omega2 + Omega3)*s/4; 
	float W  = ( Omega0 + Omega1 + Omega2 + Omega3)/(a+b)*s/4; 
		
	TheCalculatedData[0] = Vx ; 
	TheCalculatedData[1] = Vy ; 
	TheCalculatedData[2] = W ; 

	
	return TheCalculatedData;
	

}

float* Chassis_Dynamics_Inverse_Solution(float Fx, float Fy, float T) //����ѧ�����
{
	static float TheCalculatedData[4]; 
//	float resolved_Vx, resolved_Vy; //����С���ݱ任���Vx,Vy
		/*Attention: �˹�ʽ�������ֵΪ���ٶ�ֵ
			a: �ּ䳤��1/2
			b: �ּ���1/2
			s: �ֵİ뾶	 */		
	static float a_m = (1.0f/2.0f) * LengthBetweenWheel*(0.001f); //�Ե��̵�����Ϊԭ��, X�᷽��ĳ���, ��λת��Ϊm
	static float b_m = (1.0f/2.0f) * WidthBetweenWheel*(0.001f);	//�Ե��̵�����Ϊԭ��, Y�᷽��ĳ���, ��λת��Ϊm
	static float s_m = WheelRadius*(0.001f); 	//���ӵİ뾶, ��λת��Ϊm 
	
//	float Imu_Rad = DEG_TO_RAD((double)(  User_ImuDate(qaq_Yaw) )); 
//	resolved_Vx = TarVx*cos(Imu_Rad) + TarVy*sin(Imu_Rad); 
//	resolved_Vy = (-TarVx)*sin(Imu_Rad) + TarVy*cos(Imu_Rad); 
//	
	TheCalculatedData[0] = -Fx/4*s_m +  Fy/4*s_m + T/4/(a_m+b_m)*s_m; 
	TheCalculatedData[1] = -Fx/4*s_m + -Fy/4*s_m + T/4/(a_m+b_m)*s_m; 
	TheCalculatedData[2] =  Fx/4*s_m + -Fy/4*s_m + T/4/(a_m+b_m)*s_m; 
	TheCalculatedData[3] =  Fx/4*s_m +  Fy/4*s_m + T/4/(a_m+b_m)*s_m; 
//	
//	// rad/sתrpm
//	for(int i=0;i<4;i++)
//	{                                                                                                                                                  
//		TheCalculatedData[i] = (float)RADS_TO_RPM(TheCalculatedData[i]); 
//	}
	
	return TheCalculatedData; //�ó���ʱ���ٶ�, ��λrpm, ���ڵ����ٶȻ���Ŀ��ֵ
	
}


//float *CompensatingTorqueResolving(void)//�������ؽ���
//{
//	static float CompensatingTorque[4] = {0, 0, 0, 0}; //���̸�����Ĳ�������
//	
//	//����: ����imu Pitch, Roll, ����������
//	//���: �����ĵ��̸���Ҫ����������, ������CompensatingTorque
//	
//	
//	
//	
//	return CompensatingTorque; 
//}


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////










