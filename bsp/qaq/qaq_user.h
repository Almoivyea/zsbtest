#ifndef _QAQ_USER_H
#define _QAQ_USER_H

#include "stdint.h" 
#include "math.h" 



enum{
	LF=0,
	LB=1, 
	RB=2, 
	RF=3, 
	
	Yaw=0,
	Pitch=1,
	Trig=2,
	
//	RubL=0,
//	RubM=1,
//	RubR=2,
	
};  


typedef struct __ChassisDate {
	float IMU_InitialValue; 
	
	float Vx;
	float Vy;
	float Vw;
	
	
}__ChassisDate;






#endif
