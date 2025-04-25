#include "pid.h"
dsoikghndsikjbgojkls;hbgnoujkdshgojuikshoujghsioujghsoduij
#include <math.h>
#include <string.h>

#include "qaq_lib.h"


const uint8_t PID_SamplingTime = 2; //ms

void PosPID_UpdateMemberVariables(PosPID_t* pid)  
{
	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];
	pid->tar[2] = pid->tar[1];
	pid->tar[1] = pid->tar[0];
	pid->PIDout[2] = pid->PIDout[1]; 
	pid->PIDout[1] = pid->PIDout[0];
	
}


void PosPID_Init(PosPID_t* pid, 
//			 PID_DeviceName_t name,
             PosPID_BasicParam basic,
             PosPID_AdvancedParam adv,
			 PosPID_FilterConfig filter_cfg,
			 float (*err_cb)(PosPID_t*, float, float)) 
{
    pid->basic = basic;
	pid->advanced = adv;
	pid->filter = filter_cfg; 
//	pid->name = name; 

	pid->calculate_error = err_cb ? err_cb : PosErrGet_Default; //CallBack: cb 
	
    memset(pid->err, 0, sizeof(pid->err));
    memset(pid->tar, 0, sizeof(pid->tar));
    memset(pid->PIDout, 0, sizeof(pid->PIDout));
    pid->fout = 0.0;
    pid->pout = 0.0;
    pid->iout = 0.0;
    pid->dout = 0.0;
}

float PosPID_Calculation(PosPID_t* pid, float target, float input) {
    pid->tar[0] = target; 
	
	if(pid->err_insert == 0)
	{
		float filtered_input = input;
		if(pid->filter.filter_apply == FILTER_INPUT) {
			switch(pid->filter.filter_type) {
				case FILTER_LPF: {
						float alpha = pid->filter.filter_params.lpf.alpha;
						filtered_input = (1-alpha)*filtered_input+alpha* input;
					break; 
				}
				case FILTER_KALMAN: {
	//				static float est_err = 1.0;
	//				float kg = est_err / (est_err + pid->advanced.filter_param);
	//				filtered_input += kg * (input - filtered_input);
	//				est_err *= (1 - kg);
					
					break;
				}
				default: break;
			}
		}

		pid->err[0] = pid->calculate_error(pid, target, filtered_input); 
	}
	pid->err_insert = 0; //
	
	if(pid->advanced.deadband) {
		if(fabs(pid->err[0]) < pid->advanced.deadband) {
//			pid->fout = 0;
//			pid->pout = 0;
//			pid->iout = 0;
//			pid->dout = 0;
			pid->PIDout[0]=0;
			
			PosPID_UpdateMemberVariables(pid);  
			
			return 0;
		}
	}
	
    if( pid->basic.kf) {
		pid->fout = pid->basic.kf * (pid->tar[0] - pid->tar[1]);
	}
	
	if(pid->basic.kp) {
		pid->pout = pid->basic.kp * pid->err[0];
	}
	
	if(pid->basic.ki) {
		
		if(pid->advanced.separation_thres > 0 && 
		  fabs(pid->err[0]) > pid->advanced.separation_thres) {
			pid->iout = 0;  
		}else {
			float this_ki = (pid->advanced.variable_thres > 0 && fabs(pid->err[0]) > pid->advanced.variable_thres) 
							? pid->advanced.ki_variable : pid->basic.ki;
			pid->iout += this_ki * pid->err[0];
		}
		
		if(pid->basic.integral_lim > 0) {
			pid->iout = fmaxf(fminf(pid->iout, pid->basic.integral_lim), -pid->basic.integral_lim);
		}
		
	}
	if(pid->advanced.anti_WindUp > 0) 
	{
		if (pid->iout*pid->err[0]<0)
		{
			pid->iout=0; 
		}
	}

    if(pid->basic.kd) {
		pid->dout = pid->err[0] - pid->err[1]; 
		
		if(pid->filter.filter_apply == FILTER_DERIVATIVE) {
			switch(pid->filter.filter_type) {
				case FILTER_LPF: {
					float alpha = pid->filter.filter_params.lpf.alpha;
					pid->dout = (1-alpha) * pid->dout + alpha * (pid->err[1] - pid->err[2]);
					break;
				}
				case FILTER_KALMAN: { 
					
				
					break;
				}
				default: break;
			}
		}
	}
	
	pid->PIDout[0] = pid->fout + pid->pout + pid->iout + pid->dout;
    
    pid->PIDout[0] = fmaxf(fminf(pid->PIDout[0], pid->basic.output_lim), -pid->basic.output_lim);
    
	if(pid->user_func != NULL) {
		pid->user_func(pid);
		
	}
	
	
	PosPID_UpdateMemberVariables(pid); 
    
    return pid->PIDout[0];
}




//===============================================================
//Îó²î»ñÈ¡º¯Êý
//===============================================================

 // Default
float PosErrGet_Default(PosPID_t* pid, float target, float measure) {
    pid->tar[Now] = target; 
	pid->err[Now] = pid->tar[Now] - measure;
	
	return pid->err[Now];
}








///////////





