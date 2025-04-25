#include "qaq_can.h"

#include "can.h"
#include "math.h"
#include "stdarg.h" 
#include "stdint.h" 

#include "qaq_lib.h" 

Motor_Date_t Chassis_Date[4]; 
Motor_Date_t Gimbal_Date[3]; 


#define CAN_GetDateHandle(ptr, data)                                \
{                                                                   \
	(ptr)->last_ecd = (ptr)->ecd;									\
	(ptr)->ecd	 	= (int16_t)((data)[0] << 8 | (data)[1]);        \
	(ptr)->spd_rpm  = (int16_t)((data)[2] << 8 | (data)[3]);        \
	(ptr)->current  = (int16_t)((data)[4] << 8 | (data)[5]);  		\
	(ptr)->temperate= (int8_t)(data)[6];                            \
	(ptr)->ecd_err = (ptr)->ecd - (ptr)->last_ecd; 					\
	if ( fabs ((float)(ptr)->ecd_err ) >6553 ) 						\
	{																\
	(ptr)->total_round = ((ptr)->ecd_err >0)?( --(ptr)->total_round ):( ++(ptr)->total_round );	\
	}																\
	(ptr)->total_ecd = (ptr)->total_round *8192.0f + (ptr)->ecd;	\
	(ptr)->total_angle = (ptr)->total_ecd/8192.0f*360.0f;			\
} 


void CAN_FilterAndInteruptInit(void)
{
	CAN_FilterTypeDef		CAN_FilterConfigStructure;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 	0x0000; 
	CAN_FilterConfigStructure.FilterIdLow  =	0x0000; 
	CAN_FilterConfigStructure.FilterMaskIdHigh= 0x0000; 
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000; 
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0; 
	CAN_FilterConfigStructure.FilterBank = 0;	
	CAN_FilterConfigStructure.FilterActivation = ENABLE; 
	HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure); 
	
	
	HAL_CAN_Start(&hcan1); 
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); 
	
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14; 
    CAN_FilterConfigStructure.FilterBank = 14; 
    HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure); 
	
    HAL_CAN_Start(&hcan2); 
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); 
	
} 

void CAN_SetChassisValue (int16_t LFcur, int16_t LBcur, int16_t RFcur, int16_t RBcur) 
{ 
	static CAN_TxHeaderTypeDef  TxHeader; 
	static uint8_t Send_Date[8]; 
	uint32_t TxMailbox;		
	
	TxHeader.StdId = CAN_Send_ID_01; 
    TxHeader.IDE = CAN_ID_STD;		
    TxHeader.RTR = CAN_RTR_DATA;	
    TxHeader.DLC = 0x08;			
	TxHeader.TransmitGlobalTime = DISABLE;	
	
	Send_Date[0] = LFcur >> 8;
    Send_Date[1] = LFcur;
    Send_Date[2] = LBcur >> 8;
    Send_Date[3] = LBcur;
    Send_Date[4] = RFcur >> 8;
    Send_Date[5] = RFcur;
    Send_Date[6] = RBcur >> 8;
    Send_Date[7] = RBcur;
	
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Send_Date, &TxMailbox);

} 

void CAN_SetGimbalValue (int16_t Yaw_val ) 
{ 
	static CAN_TxHeaderTypeDef  TxHeader; 
	static uint8_t Send_Date[8]; 
	uint32_t TxMailbox;		
	
	TxHeader.StdId = CAN_Send_ID_02; 
    TxHeader.IDE = CAN_ID_STD;		
    TxHeader.RTR = CAN_RTR_DATA;	
    TxHeader.DLC = 0x08;			
	TxHeader.TransmitGlobalTime = DISABLE;	
	
	Send_Date[0] = Yaw_val >> 8;
    Send_Date[1] = Yaw_val;

	
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Send_Date, &TxMailbox);

} 


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef CAN_RxHeader; 
    uint8_t CAN_RxDate[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_RxDate);

    switch (CAN_RxHeader.StdId)
    {
        case CAN_Chassis_LF:
		{
			CAN_GetDateHandle(&Chassis_Date[0], CAN_RxDate);
			break;
		}
        case CAN_Chassis_LB:
		{
			CAN_GetDateHandle(&Chassis_Date[1], CAN_RxDate);
			break;
		}
        case CAN_Chassis_RB:
		{
			CAN_GetDateHandle(&Chassis_Date[2], CAN_RxDate);
			break;
		}
        case CAN_Chassis_RF:
		{
			CAN_GetDateHandle(&Chassis_Date[3], CAN_RxDate);
			break;
		}
		

		case CAN_Gimbal_Yaw:
		{
			CAN_GetDateHandle(&Gimbal_Date[0], CAN_RxDate);
			break;
		}
//		case CAN_Gimbal_Pitch:
//		{
//			CAN_GetDateHandle(&Gimbal_Date[1], CAN_RxDate);
//			break;
//		}
		
//		case CAN_Gimbal_Trig:
//		{
//			CAN_GetDateHandle(&Trig_Date, CAN_RxDate);
//			break;
//		}
		
        default:
        {
            break;
        }
    }
}









