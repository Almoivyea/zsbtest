#include "qaq_user.h"

#include "main.h"
#include "cmsis_os.h"

#include "WitIMU.h"
#include "seekfree.h"
#include "qaq_can.h"
#include "qaq_pid.h" 

 //Virtual_USB
#include "usbd_cdc_if.h" 
extern uint32_t CDC_Recv_dlen; 				 	 
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
 //Virtual_USB ////////////////////////////////////////////////////////////////

/* 	
WitIMU_GetDate(qaq_Yaw); 


xTaskNotify(TrayTaskHandle,    0, eSetValueWithOverwrite); 
xTaskNotify(LiftTaskHandle,    0, eSetValueWithOverwrite); 
xTaskNotify(CaptureTaskHandle, 0, eSetValueWithOverwrite); 

*/
extern osThreadId_t TrayTaskHandle;
extern osThreadId_t LiftTaskHandle;
extern osThreadId_t CaptureTaskHandle;


__ChassisDate ChassisDate;
short View_DecodeDate[10]; 



void View_RawDateHandler(void); 


void App_ControlTask(void *argument)
{	
	ChassisDate.IMU_InitialValue = WitIMU_GetDate(qaq_Yaw); 
	
	int Action_Flag = 0;
	
	xTaskNotify(TrayTaskHandle,    2, eSetValueWithOverwrite); 
	
  for(;;)
  {

	  
//	  Chassis_TarPos_Reset(); 
	  
	  
	  
	  
	  
    osDelay(5);
  }
}


void App_VIewTask(void *argument)
{
  for(;;)
  {
	  View_RawDateHandler(); 
	  
	  
    osDelay(5);
  }
  
}



//void App_DebugTask(void *argument)
//{
//	seekfree_assistant_interface_init(); 
//	
////	portTASK_USES_FLOATING_POINT();
//	
//	for(;;)
//	{
//		seekfree_assistant_data_analysis();
//		if(seekfree_assistant_parameter_update_flag[0]  || seekfree_assistant_parameter_update_flag[1] || 
//			seekfree_assistant_parameter_update_flag[2] || seekfree_assistant_parameter_update_flag[3] || 
//			seekfree_assistant_parameter_update_flag[4] || seekfree_assistant_parameter_update_flag[5] )
//		{
//			seekfree_assistant_parameter_update_flag[0]=0; 
//			seekfree_assistant_parameter_update_flag[1]=0; 
//			seekfree_assistant_parameter_update_flag[2]=0; 
//			seekfree_assistant_parameter_update_flag[3]=0; 
//			seekfree_assistant_parameter_update_flag[4]=0; 
//			seekfree_assistant_parameter_update_flag[5]=0; 
//			
//			ChassisPosSpd[LF].basic.kp=seekfree_assistant_parameter[0]; //Params Update
//			ChassisPosSpd[LF].basic.ki=seekfree_assistant_parameter[1];
//			ChassisPosSpd[LF].basic.kd=seekfree_assistant_parameter[2]; 
//			ChassisPosSpd[LF].err[0]=0; //Reset Status
//			ChassisPosSpd[LF].err[1]=0; 
//			ChassisPosSpd[LF].err[2]=0; 
//			ChassisPosSpd[LF].PIDout[0]=0; 
//			ChassisPosSpd[LF].PIDout[1]=0; 
//			ChassisPosSpd[LF].PIDout[2]=0; 
//			ChassisPosSpd[LF].fout=0; 
//			ChassisPosSpd[LF].pout=0; 
//			ChassisPosSpd[LF].iout=0; 
//			ChassisPosSpd[LF].dout=0; 
//			       	
//	
////			static float p,i,d; 
////			p=seekfree_assistant_parameter[0];
////			i=seekfree_assistant_parameter[1];
////			d=seekfree_assistant_parameter[2];
////				printf("%.3f, %.3f, %.3f", p, i, d); 
//			

//		}
//	osDelay(5);
//	}
//}



void View_RawDateHandler(void) {
	const static uint8_t FrameHead = 0xFE; 
	const static uint8_t FrameTail = 0xFF; 
	const static uint8_t RawDateSumLength= 11;
	
	if( (UserRxBufferFS[0] == FrameHead) && (UserRxBufferFS[RawDateSumLength-1] == FrameTail) ){
//		View_DecodeDate[0] = UserRxBufferFS[4] | UserRxBufferFS[5]<<8; 
//		View_DecodeDate[1] = UserRxBufferFS[4] | UserRxBufferFS[5]<<8; //注意这里数据读取未更改
//		View_DecodeDate[2] = UserRxBufferFS[4] | UserRxBufferFS[5]<<8; 
//		View_DecodeDate[3] = UserRxBufferFS[4] | UserRxBufferFS[5]<<8; 
//		View_DecodeDate[4] = UserRxBufferFS[4] | UserRxBufferFS[5]<<8; 
	}
	else {
		memset(UserRxBufferFS, 0, RawDateSumLength); 
	}
	
}










