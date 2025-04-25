#include "qaq_pid.h" 
#include "pid.h" 

#include "qaq_user.h"
#include "qaq_lib.h"


float ErrGet_6020Ecd(PosPID_t* pid, float target, float NowEncode ); 
float ErrGet_ChassisAngle(PosPID_t* pid, float target, float measure);
//float ErrGet_ChassisSpd(PosPID_t* pid, float target, float measure);



PosPID_t ChassisCasPos[4];  PosPID_t ChassisPosSpd[4]; 
PosPID_t ChassisAng; 
void pid_init_chassis(void) 
{
  //底盘位置环	
	PosPID_BasicParam ChassisCP_basic = { 
		.kp = 0.013,  .ki = 0.0,  .kd = 0.0, .kf = 0.0,
		.integral_lim = 5000,
		.output_lim = 4000
		// 36*
	}; PosPID_AdvancedParam ChassisCP_adv = {
		.deadband = 0.0,  // 2000
		.separation_thres = 0.0,
		.ki_variable = 0.0,
		.variable_thres = 0.0,
		.anti_WindUp = 0,
    }; PosPID_FilterConfig ChassisCP_filter = {
		.filter_params.lpf.alpha = 0.15, 
		.filter_type = FILTER_LPF,
		.filter_apply = FILTER_INPUT
    }; 
	for(int i=0; i<4; i++) {
		PosPID_Init(&ChassisCasPos[i], ChassisCP_basic, ChassisCP_adv, ChassisCP_filter, PosErrGet_Default); 
	}
	
 //底盘速度环		
	PosPID_BasicParam ChassisPS_basic = { 
		.kp = 0.8,  .ki = 0.02  ,  .kd = 0.0, .kf = 0.0,
		.integral_lim = 2000,
		.output_lim = 1500   // 对内环最终电流值的输出进行限幅,其实就是限制住加速度了
	}; PosPID_AdvancedParam ChassisPS_adv = {
		.deadband = 0.0,
		.separation_thres = 0.0,
		.ki_variable = 0.0,
		.variable_thres = 0.0,
		.anti_WindUp = 0,
    }; PosPID_FilterConfig ChassisPS_filter = {
		.filter_params.lpf.alpha = 0.15, 
		.filter_type = FILTER_LPF,
		.filter_apply = FILTER_INPUT
    }; 
	for(int i=0; i<4; i++) {
		PosPID_Init(&ChassisPosSpd[i], ChassisPS_basic, ChassisPS_adv, ChassisPS_filter, PosErrGet_Default); 
	}
	
//	ChassisPosSpd[2].basic.output_lim = 2000;
//	ChassisPosSpd[3].basic.output_lim = 2000;
	
	
 //		ChassisPosSpd[i].user_func = Chassis_Acc_Lim;

	
 //底盘角度环 
	PosPID_BasicParam ChassisAng_basic = { 
		.kp = 14.0,  .ki = 0.0,  .kd = 0.0, .kf = 0.0,
		.integral_lim = 5000,
		.output_lim = 1000
	}; PosPID_AdvancedParam ChassisAng_adv = {
		.deadband = 0.0,
		.separation_thres = 0.0,
		.ki_variable = 0.0,
		.variable_thres = 0.0,
		.anti_WindUp = 0,
    }; PosPID_FilterConfig ChassisAng_filter = {
		.filter_params.lpf.alpha = 0.15, 
		.filter_type = FILTER_TYPE_NONE,
		.filter_apply = FILTER_APPLY_NONE
    }; 
	PosPID_Init(&ChassisAng, ChassisAng_basic, ChassisAng_adv, ChassisAng_filter, ErrGet_ChassisAngle); 
	
}


PosPID_t YawEcd; 
void pid_init_gimbal(void) 
{
		PosPID_BasicParam YE_basic = { 
		.kp = 1.5,  .ki = 0.1,  .kd = 0.0, .kf = 0.0,
		.integral_lim = 1800,
		.output_lim = 20000
	}; PosPID_AdvancedParam YE_adv = {
		.deadband = 0.0,
		.separation_thres = 0.0,
		.ki_variable = 0.0,
		.variable_thres = 0.0,
		.anti_WindUp = 1
    }; PosPID_FilterConfig YE_filter = {
		.filter_params.lpf.alpha = 0.15, 
		.filter_type = FILTER_LPF,
		.filter_apply = FILTER_DERIVATIVE 
    }; PosPID_Init(&YawEcd, YE_basic, YE_adv, YE_filter, ErrGet_6020Ecd); 
	
	
}




float CasMaxSpd_Lim(float CasOut, float MaxSpd) {
	return ( (fabs(CasOut)>MaxSpd)?((CasOut>0)?MaxSpd:-MaxSpd):CasOut ); 
	
}

void Chassis_TarPos_Reset(PosPID_t* pid, float target) {
	
	
	
	
}

//float ResistanceCompensation(float OutPut ) {
//	
//	
//	
//}


float ErrGet_6020Ecd(PosPID_t* pid, float target, float NowEncode ) {
    pid->tar[Now] = target; 
	pid->err[Now] = GM6020_EcdErrCal(target, NowEncode);
	
	return pid->err[Now];
	
}

float ErrGet_ChassisAngle(PosPID_t* pid, float target, float measure) {
    pid->tar[Now] = target; 
	pid->err[Now] = WitIMU_AngErrCal(target , measure);
	
	return pid->err[Now];
}

//float ErrGet_ChassisSpd(PosPID_t* pid, float target, float measure) {
//	
//    pid->tar[Now] = target; 
//	pid->err[Now] = pid->tar[Now] - measure;
//	
//	return pid->err[Now];
//	
//}







