#include "qaq_lib.h"

#include <math.h>
#include "stdint.h"

#include "usbd_cdc_if.h" 
#include <stdarg.h>
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
void usb_printf(const char *format, ...)
{
    va_list args;
    uint32_t length;
 
    va_start(args, format);
    length = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, (char *)format, args);
    va_end(args);
    CDC_Transmit_FS(UserTxBufferFS, length); 
}


float GM6020_EcdErrCal(float des, float NowEncode ) {
	static float MaxValue = 8192.0f; 
	
	float Target = fmod(des, MaxValue); 
	float current = fmod(NowEncode, MaxValue); 
	float clockwise  = (Target >= current) ? (Target - current) : (MaxValue - current + Target);
    float counterclockwise= (current >= Target) ? (current - Target) : (MaxValue  - Target + current);

	return (clockwise < counterclockwise) ? clockwise : -counterclockwise;
}


float WitIMU_AngErrCal(float des, float now ) {
	static float MaxValue = 360.0f; 
	
	des = ( (des<0)?(des+360.0f):des ); 
	now = ( (now<0)?(now+360.0f):now );  
	
	float Target = fmod(des, MaxValue); 
	float current = fmod(now, MaxValue); 
	float clockwise  = (Target >= current) ? (Target - current) : (MaxValue - current + Target);
    float counterclockwise= (current >= Target) ? (current - Target) : (MaxValue  - Target + current);

	return (clockwise < counterclockwise) ? -clockwise : counterclockwise;
}

const float max_acc = 60;
float Chassis_Acc_Lim(PosPID_t* pid) {
	
	float delta_spd = pid->PIDout[0] - pid->PIDout[1];
	
//	pid->PIDout[0] = (fabs(delta_spd)>max_acc)?( (delta_spd>0)?max_acc:-max_acc ):delta_spd;
	
	if(fabs(delta_spd)>max_acc) {
		if(pid->PIDout[0]>pid->PIDout[1]) {
			pid->PIDout[0] = pid->PIDout[1] + max_acc;
		}
		else if(pid->PIDout[0]<pid->PIDout[1]) {
			pid->PIDout[0] = pid->PIDout[1] - max_acc;
			
			
		}		
	}

	
	return pid->PIDout[0]; 
	
}














