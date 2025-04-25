#include "qaq_lib.h"

#include "main.h"
//#include "stdlib.h"
#include "math.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "tim.h"

#include "qaq_user.h"
#include "qaq_uart.h"
#include "qaq_lib.h"
#include "pid.h"
#include "qaq_pid.h"
#include "qaq_can.h"
#include "WitIMU.h"








float turn_ang_err = 0;
float add_spd  = 0;




///////////////////////////////////////////////
/////////////////////////////////////////////


#define MotorTest	4

float PhaseTarAngle = 0; 
float WheelTarEcd[4] = {0};
void App_ChassisTask(void *argument)
{
	pid_init_chassis(); 
	short ChassisCur_Set[4] = {0}; // 用于最后的CAN通信赋值
	
	
	#define MaxSpd  100*Radio_2006  //rpm  限制一下底盘的最大速度
	float ChassisTarPos[4] = {0,0,0,0}; 
	float add_pos = 0;
	ChassisTarPos[0] =  mm2ecd(800);
	ChassisTarPos[1] =  mm2ecd(800);
	ChassisTarPos[2] = -mm2ecd(800);
	ChassisTarPos[3] = -mm2ecd(800);
	
	
  for(;;)
  {	 TickType_t PreviousTick=xTaskGetTickCount();
	  
#if MotorTest == 0 // 最初的实现想法
{
		PosPID_Calculation(&ChassisCasPos[LF], WheelTarEcd[LF], Chassis_Date[LF].total_ecd); 
		PosPID_Calculation(&ChassisCasPos[LB], WheelTarEcd[LB], Chassis_Date[LB].total_ecd);
		PosPID_Calculation(&ChassisCasPos[RB], WheelTarEcd[RB], Chassis_Date[RB].total_ecd);
		PosPID_Calculation(&ChassisCasPos[RF], WheelTarEcd[RF], Chassis_Date[RF].total_ecd);
	    
		PosPID_Calculation(&ChassisAng, PhaseTarAngle, Chassis_Date[LF].total_ecd); 
	    
		PosPID_Calculation(&ChassisPosSpd[LF], ChassisCasPos[LF].PIDout[Now] + ChassisAng.PIDout[Now], Chassis_Date[LF].spd_rpm); 
		PosPID_Calculation(&ChassisPosSpd[LB], ChassisCasPos[LB].PIDout[Now] + ChassisAng.PIDout[Now], Chassis_Date[LB].spd_rpm);
		PosPID_Calculation(&ChassisPosSpd[RB], ChassisCasPos[RB].PIDout[Now] - ChassisAng.PIDout[Now], Chassis_Date[RB].spd_rpm);
		PosPID_Calculation(&ChassisPosSpd[RF], ChassisCasPos[RF].PIDout[Now] - ChassisAng.PIDout[Now], Chassis_Date[RF].spd_rpm);
		ChassisCur_Set[LF] = ChassisPosSpd[LF].PIDout[Now]; 
		ChassisCur_Set[LB] = ChassisPosSpd[LB].PIDout[Now]; 
		ChassisCur_Set[RB] = ChassisPosSpd[RB].PIDout[Now]; 
		ChassisCur_Set[RF] = ChassisPosSpd[RF].PIDout[Now]; 
}
#elif MotorTest == 1  //速度环调试
{
		PosPID_Calculation(&ChassisPosSpd[LF],  260, Chassis_Date[LF].spd_rpm); // 120*Radio_2006
		PosPID_Calculation(&ChassisPosSpd[LB],  260, Chassis_Date[LB].spd_rpm);
		PosPID_Calculation(&ChassisPosSpd[RF], -260, Chassis_Date[RF].spd_rpm);
		PosPID_Calculation(&ChassisPosSpd[RB], -260, Chassis_Date[RB].spd_rpm);
		ChassisCur_Set[LF] = ChassisPosSpd[LF].PIDout[Now]; // + ( (ChassisPosSpd[LF].PIDout[Now]>0)?120:-120 );   // 120
		ChassisCur_Set[LB] = ChassisPosSpd[LB].PIDout[Now]; // + ( (ChassisPosSpd[LF].PIDout[Now]>0)?110:-110 );   // 110
		ChassisCur_Set[RB] = ChassisPosSpd[RB].PIDout[Now]; // - ( (ChassisPosSpd[LF].PIDout[Now]>0)?145:-145 );   // 145
		ChassisCur_Set[RF] = ChassisPosSpd[RF].PIDout[Now]; // - ( (ChassisPosSpd[LF].PIDout[Now]>0)?160:-160 );   // 160
		
//		printf("%.3f,%.3f\r\n", ChassisPosSpd[LF].tar[0], Chassis_Date[LF].spd_rpm); 
//		printf("%.3f,%.3f\r\n", ChassisPosSpd[LB].tar[0], Chassis_Date[LB].spd_rpm); 
//		printf("%.3f,%.3f\r\n", ChassisPosSpd[RB].tar[0], Chassis_Date[RB].spd_rpm); 
//		printf("%.3f,%.3f\r\n", ChassisPosSpd[RF].tar[0], Chassis_Date[RF].spd_rpm); 
	printf("%.3f,%.3f,%.3f,%.3f\r\n", Chassis_Date[LF].current, Chassis_Date[LB].current, Chassis_Date[RF].current, Chassis_Date[RB].current); 
	
	
}
#elif MotorTest == 2    //位置环调试
{
		PosPID_Calculation(&ChassisCasPos[LF], mm2ecd(180), Chassis_Date[LF].total_ecd);  //mm2ecd(800) mm2ecd(251.327)  8192*Radio_2006  
		PosPID_Calculation(&ChassisCasPos[LB], mm2ecd(180), Chassis_Date[LB].total_ecd);
		PosPID_Calculation(&ChassisCasPos[RB], mm2ecd(180), Chassis_Date[RB].total_ecd);
		PosPID_Calculation(&ChassisCasPos[RF], mm2ecd(180), Chassis_Date[RF].total_ecd);
		ChassisCur_Set[LF] = ChassisCasPos[LF].PIDout[Now]; 
		ChassisCur_Set[LB] = ChassisCasPos[LB].PIDout[Now]; 
		ChassisCur_Set[RB] = ChassisCasPos[RB].PIDout[Now]; 
		ChassisCur_Set[RF] = ChassisCasPos[RF].PIDout[Now]; 


//		printf("%.3f,%.3f,%.3f,%.3f\r\n", ChassisCasPos[LF].tar[0], Chassis_Date[LF].total_ecd, ChassisCasPos[LF].PIDout[0]
//		, ChassisCasPos[LF].dout); 
}
#elif MotorTest == 3  //没有IMU
{
		#define MaxSpd  100*Radio_2006  //rpm
		PosPID_Calculation(&ChassisCasPos[LF],  mm2ecd(800), Chassis_Date[LF].total_ecd); // WheelTarEcd[LF]
		PosPID_Calculation(&ChassisCasPos[LB],  mm2ecd(800), Chassis_Date[LB].total_ecd);
		PosPID_Calculation(&ChassisCasPos[RB], -mm2ecd(800), Chassis_Date[RB].total_ecd);
		PosPID_Calculation(&ChassisCasPos[RF], -mm2ecd(800), Chassis_Date[RF].total_ecd);
		
		//变速处理后期再做
		PosPID_Calculation(&ChassisPosSpd[LF],  CasMaxSpd_Lim(ChassisCasPos[LF].PIDout[Now], MaxSpd), Chassis_Date[LF].spd_rpm); 
		PosPID_Calculation(&ChassisPosSpd[LB],  CasMaxSpd_Lim(ChassisCasPos[LB].PIDout[Now], MaxSpd), Chassis_Date[LB].spd_rpm); 
		PosPID_Calculation(&ChassisPosSpd[RB],  CasMaxSpd_Lim(ChassisCasPos[RB].PIDout[Now], MaxSpd), Chassis_Date[RB].spd_rpm); 
		PosPID_Calculation(&ChassisPosSpd[RF],  CasMaxSpd_Lim(ChassisCasPos[RF].PIDout[Now], MaxSpd), Chassis_Date[RF].spd_rpm); 
		
		ChassisCur_Set[LF] = ChassisPosSpd[LF].PIDout[Now] + ( (ChassisPosSpd[LF].PIDout[Now]>0)?120:-120 ); //空载情况的补偿
		ChassisCur_Set[LB] = ChassisPosSpd[LB].PIDout[Now] + ( (ChassisPosSpd[LF].PIDout[Now]>0)?110:-110 ); 
		ChassisCur_Set[RB] = ChassisPosSpd[RB].PIDout[Now] - ( (ChassisPosSpd[LF].PIDout[Now]>0)?145:-145 ); 
		ChassisCur_Set[RF] = ChassisPosSpd[RF].PIDout[Now] - ( (ChassisPosSpd[LF].PIDout[Now]>0)?160:-160 ); 
		
//		printf("%.3f,%.3f\r\n", ChassisCasPos[LF].tar[0], Chassis_Date[LF].total_ecd); 
		
}	
#elif MotorTest == 4  //有IMU + 尝试通过改变位置来改变底盘姿态
{ 
	
		// 213+321
		//PosPID_Calculation(&ChassisAng, PhaseTarAngle, WitDate[2]); 
//		turn_ang_err = WitIMU_AngErrCal(0, WitDate[2]); // PhaseTarAngle
//		add_pos += turn_ang_err*25;  // (turn_ang_err/90.0f)*422430; 

//		
//		ChassisTarPos[0] += add_pos; 
//		ChassisTarPos[1] += add_pos; 
//		ChassisTarPos[2] += add_pos; 
//		ChassisTarPos[3] += add_pos; 
		
//		if(turn_ang_err>0 && ) {
//			
//			
//		}

//用户自定义函数
		
		PosPID_Calculation(&ChassisCasPos[LF], ChassisTarPos[0], Chassis_Date[LF].total_ecd); // WheelTarEcd[LF]  800
		PosPID_Calculation(&ChassisCasPos[LB], ChassisTarPos[1], Chassis_Date[LB].total_ecd); // (422430*2)+1
		PosPID_Calculation(&ChassisCasPos[RB], ChassisTarPos[2], Chassis_Date[RB].total_ecd); // mm2ecd(360)
		PosPID_Calculation(&ChassisCasPos[RF], ChassisTarPos[3], Chassis_Date[RF].total_ecd);
		
		// 因为每一次都是更新完成员值,才执行这个,所以根本就没有作用
//		Chassis_Acc_Lim(&ChassisCasPos[LF]); //对低底盘的目标加速度进行限制 
//		Chassis_Acc_Lim(&ChassisCasPos[LB]); 
//		Chassis_Acc_Lim(&ChassisCasPos[RB]); 
//		Chassis_Acc_Lim(&ChassisCasPos[RF]); 
		
		PosPID_Calculation(&ChassisPosSpd[LF],  ChassisCasPos[LF].PIDout[Now], Chassis_Date[LF].spd_rpm); 
		PosPID_Calculation(&ChassisPosSpd[LB],  ChassisCasPos[LB].PIDout[Now], Chassis_Date[LB].spd_rpm); 
		PosPID_Calculation(&ChassisPosSpd[RB],  ChassisCasPos[RB].PIDout[Now], Chassis_Date[RB].spd_rpm); 
		PosPID_Calculation(&ChassisPosSpd[RF],  ChassisCasPos[RF].PIDout[Now], Chassis_Date[RF].spd_rpm); 		
		
//		//第二个参数做了最大速度限制, 当完成加速度限制后, 是否不再需要速度限制
//		PosPID_Calculation(&ChassisPosSpd[LF],  CasMaxSpd_Lim(ChassisCasPos[LF].PIDout[Now], MaxSpd), Chassis_Date[LF].spd_rpm); 
//		PosPID_Calculation(&ChassisPosSpd[LB],  CasMaxSpd_Lim(ChassisCasPos[LB].PIDout[Now], MaxSpd), Chassis_Date[LB].spd_rpm); 
//		PosPID_Calculation(&ChassisPosSpd[RB],  CasMaxSpd_Lim(ChassisCasPos[RB].PIDout[Now], MaxSpd), Chassis_Date[RB].spd_rpm); 
//		PosPID_Calculation(&ChassisPosSpd[RF],  CasMaxSpd_Lim(ChassisCasPos[RF].PIDout[Now], MaxSpd), Chassis_Date[RF].spd_rpm); 
		
															// 阻力补偿
		ChassisCur_Set[LF] = ChassisPosSpd[LF].PIDout[Now] + ( (ChassisPosSpd[LF].PIDout[Now]>0)?176:-176 );   // 120 180 165
		ChassisCur_Set[LB] = ChassisPosSpd[LB].PIDout[Now] + ( (ChassisPosSpd[LF].PIDout[Now]>0)?180:-180 );   // 110 180 169
		ChassisCur_Set[RB] = ChassisPosSpd[RB].PIDout[Now] - ( (ChassisPosSpd[LF].PIDout[Now]>0)?261:-261 );   // 145 260 261
		ChassisCur_Set[RF] = ChassisPosSpd[RF].PIDout[Now] - ( (ChassisPosSpd[LF].PIDout[Now]>0)?267:-267 );   // 160 270 267
		
		
		
//		ChassisCur_Set[LF] = Chassis_Acc_Lim(&ChassisPosSpd[LF]) + ( (ChassisPosSpd[LF].PIDout[Now]>0)?120:-120 ); //空载情况的补偿
//		ChassisCur_Set[LB] = Chassis_Acc_Lim(&ChassisPosSpd[LB]) + ( (ChassisPosSpd[LF].PIDout[Now]>0)?115:-115 ); 
//		ChassisCur_Set[RB] = Chassis_Acc_Lim(&ChassisPosSpd[RB]) - ( (ChassisPosSpd[LF].PIDout[Now]>0)?145:-145 ); 
//		ChassisCur_Set[RF] = Chassis_Acc_Lim(&ChassisPosSpd[RF]) - ( (ChassisPosSpd[LF].PIDout[Now]>0)?160:-160 );
		
		
		
//		printf("%.3f,%.3f\r\n", ChassisCasPos[LF].tar[0], Chassis_Date[LF].total_ecd); 
//		printf("%.3f,%.3f\r\n", ChassisCasPos[LF].PIDout[0], ChassisPosSpd[LF].PIDout[0]); 
		
}	
#elif MotorTest == 5  //有IMU + 尝试通过改变速度来改变底盘姿态
{
/*
第二, 内环的最大速度衰减问题

*/
		#define MaxSpd  100*Radio_2006  //rpm
		
		
		turn_ang_err = WitIMU_AngErrCal(0, WitDate[2]); // PhaseTarAngle
		add_spd = turn_ang_err*180;
		
		PosPID_Calculation(&ChassisCasPos[LF],  mm2ecd(0), Chassis_Date[LF].total_ecd); // WheelTarEcd[LF]  800
		PosPID_Calculation(&ChassisCasPos[LB],  mm2ecd(0), Chassis_Date[LB].total_ecd); // (422430*2)+1
		PosPID_Calculation(&ChassisCasPos[RB], -mm2ecd(0), Chassis_Date[RB].total_ecd); // mm2ecd(360)
		PosPID_Calculation(&ChassisCasPos[RF], -mm2ecd(0), Chassis_Date[RF].total_ecd); 
		
		PosPID_Calculation(&ChassisAng, 0, WitDate[2]); // PhaseTarAngle
		
		//变速处理后期再做 
		// 角度环的输出, 极性的正确 (全加或者全减)
		PosPID_Calculation(&ChassisPosSpd[LF],  CasMaxSpd_Lim(ChassisCasPos[LF].PIDout[Now], MaxSpd)+add_spd, Chassis_Date[LF].spd_rpm); 
		PosPID_Calculation(&ChassisPosSpd[LB],  CasMaxSpd_Lim(ChassisCasPos[LB].PIDout[Now], MaxSpd)+add_spd, Chassis_Date[LB].spd_rpm); 
		PosPID_Calculation(&ChassisPosSpd[RB],  CasMaxSpd_Lim(ChassisCasPos[RB].PIDout[Now], MaxSpd)+add_spd, Chassis_Date[RB].spd_rpm); 
		PosPID_Calculation(&ChassisPosSpd[RF],  CasMaxSpd_Lim(ChassisCasPos[RF].PIDout[Now], MaxSpd)+add_spd, Chassis_Date[RF].spd_rpm); 
		
		ChassisCur_Set[LF] = ChassisPosSpd[LF].PIDout[Now] + ( (ChassisPosSpd[LF].PIDout[Now]>0)?120:-120 ); //空载情况的补偿
		ChassisCur_Set[LB] = ChassisPosSpd[LB].PIDout[Now] + ( (ChassisPosSpd[LF].PIDout[Now]>0)?110:-110 ); 
		ChassisCur_Set[RB] = ChassisPosSpd[RB].PIDout[Now] - ( (ChassisPosSpd[LF].PIDout[Now]>0)?145:-145 ); 
		ChassisCur_Set[RF] = ChassisPosSpd[RF].PIDout[Now] - ( (ChassisPosSpd[LF].PIDout[Now]>0)?160:-160 ); 
		
//		printf("%.3f,%.3f\r\n", ChassisCasPos[LF].tar[0], Chassis_Date[LF].total_ecd); 
}
#endif
	  
	  CAN_SetChassisValue(ChassisCur_Set[0], ChassisCur_Set[1], ChassisCur_Set[2], ChassisCur_Set[3]); 
//	  CAN_SetChassisValue(170, 180, -260, -270); 
	                
	  
    vTaskDelayUntil(&PreviousTick, pdMS_TO_TICKS(PID_SamplingTime));
  }
}


 
 // TrayTaskHandle 
void AppTrayTask(void *argument)
{
	uint32_t Tray_Flag = 0;
	
  for(;;)
  { xTaskNotifyWait(0,0xFF, &Tray_Flag, portMAX_DELAY); 

	taskENTER_CRITICAL();
	  if(Tray_Flag == 1) {
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 500);
		  
	  }
	  else if(Tray_Flag == 2) {
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 2500);
		  
	  }
	  else if(Tray_Flag == 3) {
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 2500);
		  
	  }
	  
	taskEXIT_CRITICAL();
  }
}






