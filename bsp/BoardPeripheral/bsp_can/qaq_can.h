#ifndef QAQ_CAN_H 
#define QAQ_CAN_H 

#include "stdint.h"

typedef enum
{
	CAN_Send_ID_01   = 0x200,
	CAN_Chassis_LF   = 0x201,	//LeftFront
	CAN_Chassis_LB   = 0x202,	//LeftBack
	CAN_Chassis_RB   = 0x203,	
	CAN_Chassis_RF   = 0x204,	
	
	CAN_Send_ID_02	 = 0x1FF,
	CAN_Gimbal_Yaw	 = 0x205, 
	CAN_Gimbal_Pitch = 0x206, 
	CAN_Gimbal_Trig  = 0x207, 
	
	CAN_Send_ID_03	 = 0x2FF,
	CAN_6020_ID1 = 0x205, 
    CAN_6020_ID2 = 0x206, 
    CAN_6020_ID3 = 0x207, 
	CAN_6020_ID4 = 0x208, 
	CAN_6020_ID5 = 0x209, 
    CAN_6020_ID6 = 0x20A, 
    CAN_6020_ID7 = 0x20B, 
	
//	CAN_RubL = 0x201,
//	CAN_RubM = 0x202,
//	CAN_RubR = 0x203,
	
} CAN_DeviceName_t;

typedef struct _motor_measure_t 
{
	float ecd; 
	float last_ecd;
	float spd_rpm;
	float current;
	uint8_t temperate;
	
	float ecd_err;
	float total_ecd; 
	float total_round; 
	float total_angle; 
	
} Motor_Date_t; 


extern Motor_Date_t Chassis_Date[4]; 
extern Motor_Date_t Gimbal_Date[3]; 


void CAN_FilterAndInteruptInit(void);
void CAN_SetChassisValue (int16_t LFcur, int16_t LBcur, int16_t RFcur, int16_t RBcur); 
void CAN_SetGimbalValue (int16_t Yaw_val); 



#endif 
