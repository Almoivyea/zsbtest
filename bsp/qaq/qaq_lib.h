#ifndef _QAQ_LIB_H
#define _QAQ_LIB_H


#include "pid.h"


#define Pi 3.141592653589f // 3.14159265358979323846  
#define Radio_2006	36
#define WheelPerimeter_mm	251.327408f  // 2*Pi*40


#define DEG_TO_RAD(deg) ((deg) * (float)Pi / 180.0f) 
#define RADS_TO_RPM(rads) ((rads) * 60.0f / (2.0f * (float)Pi)) // rad/s ת rpm 
#define mm2ecd(mm)    (mm/WheelPerimeter_mm)*(8192.0f*Radio_2006)



extern void usb_printf(const char *format, ...); 


float GM6020_EcdErrCal(float des, float NowEncode ); 
float WitIMU_AngErrCal(float des, float now ); 

float Chassis_Acc_Lim(PosPID_t* pid); 







#endif
