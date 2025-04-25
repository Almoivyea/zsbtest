#ifndef __QAQ_PID_H 
#define __QAQ_PID_H 


#include "pid.h"






extern PosPID_t ChassisCasPos[4];  extern PosPID_t ChassisPosSpd[4]; 
extern PosPID_t ChassisAng; 

extern PosPID_t YawEcd; 



float CasMaxSpd_Lim(float CasOut, float MaxSpd); 


void pid_init_chassis(void); 
void pid_init_gimbal(void); 


float ErrGet_ChassisAngle(PosPID_t* pid, float target, float measure);




#endif

