#include "qaq_lib.h"

#include "main.h"
#include "tim.h"
#include "stdio.h"
#include "cmsis_os.h"

#include "qaq_user.h"
#include "qaq_pid.h"
#include "MarkBase42C.h"
#include "qaq_can.h"


uint8_t Yaw_TurnCmd = 0;
			  
void App_GimbalTask(void *argument)
{
	short GimbalVol_Set[3]  = {0};  
	float YawOrientation[2] = {4048, 7391}; 
	pid_init_gimbal(); 
	
  for(;;)
  {	 TickType_t PreviousTick=xTaskGetTickCount();
	  
//	  PosPID_Calculation(&YawEcd, YawOrientation[1], Gimbal_Date[0].ecd); 
//	  GimbalVol_Set[0] = YawEcd.PIDout[Now];  
//	  printf("%.3f,%.3f,%.3f\r\n", YawEcd.tar[0], Gimbal_Date[0].ecd, YawEcd.iout); 
	  
	  // 默认 云台朝外  (0是朝外,  1是朝内(拿到东西了就是1))
	  if(Yaw_TurnCmd == 1) {
		  PosPID_Calculation(&YawEcd, YawOrientation[1], Gimbal_Date[0].ecd); 
		  GimbalVol_Set[0] = YawEcd.PIDout[Now];   
	  }
	  else {
		  PosPID_Calculation(&YawEcd, YawOrientation[0], Gimbal_Date[0].ecd); 
		  GimbalVol_Set[0] = YawEcd.PIDout[Now];  
	  }
	   
	  
	  CAN_SetGimbalValue( GimbalVol_Set[0] ); 
	  
    vTaskDelayUntil(&PreviousTick, pdMS_TO_TICKS(PID_SamplingTime));
  }
  
}



// // LiftTaskHandle
//void App_LiftTask(void *argument)
//{
//	typedef struct __Lift {
//		unsigned int get_raw;
//		unsigned int harvest;
//		unsigned int put_gnd;
//		unsigned int put_stack; 
//	}__Lift; // MB_SumPos
//	
//	#define LiftRunSpd    20
//	__Lift Lift_Pos = {.get_raw = 1500,   //相对于0点的绝对脉冲数
//					   .harvest = 5000,
//					   .put_gnd = 500,
//					   .put_stack = 1000,
//					  }; 
//	
//	uint32_t Lift_Flag = 0;
//					  
//  for(;;)
//  { xTaskNotifyWait(0,0xFF, &Lift_Flag, portMAX_DELAY); 
//	  
//	  if(Lift_Flag == 1) { //从原料区获取物料的高度
//		  
//		  StMoRunTarPos(M0, LiftRunSpd, Lift_Pos.get_raw); 
//	  }
//	  else if(Lift_Flag == 2) {
//		  StMoRunTarPos(M0, LiftRunSpd, Lift_Pos.harvest); 		  
//	  }
//	  else if(Lift_Flag == 3) {
//		  StMoRunTarPos(M0, LiftRunSpd, Lift_Pos.put_gnd); 		  
//	  }
//	  else if(Lift_Flag == 4) {
//		  StMoRunTarPos(M0, LiftRunSpd, Lift_Pos.put_stack); 		  
//	  }
//	  
//    osDelay(1);
//  }
//}



// // CaptureTaskHandle 
//void App_CaptureTask(void *argument)
//{
//	uint32_t Capture_Flag = 0;
//	
//  for(;;)
//  { xTaskNotifyWait(0,0xFF, &Capture_Flag, portMAX_DELAY); 

//	taskEXIT_CRITICAL();
//	  if(Capture_Flag == 0) { //夹爪打开
//		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 1000);
// 
//	  }
//	  else { //夹爪闭合
//		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 1500);
//		  
//	  }
//	taskENTER_CRITICAL();
//	  
//  
//    osDelay(1);
//  }
//}






