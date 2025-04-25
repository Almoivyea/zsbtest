#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include <stdbool.h>


extern const uint8_t PID_SamplingTime;

enum {
	Now = 0,
	Last = 1,
	LLast = 2,
};


typedef enum {
	FILTER_TYPE_NONE=0,
	FILTER_LPF,
	FILTER_KALMAN
} FilterType;

typedef enum {
	FILTER_APPLY_NONE,
	FILTER_DERIVATIVE,
	FILTER_INPUT
} FilterApply;
	

typedef struct {
    double kp;           
    double ki;           
    double kd;           
    double kf;           
    double integral_lim; 
    double output_lim;   
} PosPID_BasicParam; 

typedef struct {
    float deadband;         // 死区阈值
    float separation_thres; // 积分分离阈值 threshold
    float variable_thres;   // 变积分阈值
	float ki_variable; 
	float anti_WindUp;  //实际上积分分离,变积分,积分限幅都属于抗积分饱和, 但这个参数控制的处理方法在位置控制时很好用,但没有专业名词, 姑且先这么称呼
	
} PosPID_AdvancedParam;

typedef struct {
	union {
		struct {                
			float alpha;        
		} lpf;
		struct {                
			float q;            
			float r;            
		} kalman;
	} filter_params;
	
	FilterType filter_type;
	FilterApply filter_apply;

} PosPID_FilterConfig; 

typedef struct PosPID_t  PosPID_t;
struct PosPID_t {
    PosPID_BasicParam basic;
    PosPID_AdvancedParam advanced;
    PosPID_FilterConfig filter;
//	PID_DeviceName_t name;
	
    float tar[3];       
	float err[3];       
    float PIDout[3];    
    
    float fout;         
    float pout;         
    float iout;         
    float dout;         
    
	float (*calculate_error)(PosPID_t*, float, float);
	float (*user_func)(PosPID_t*);
	
	uint8_t err_insert;
}; 


/*************
== 接口函数
*************/ 
void PosPID_Init(PosPID_t* pid, 
//			 PID_DeviceName_t name,
             PosPID_BasicParam basic,
             PosPID_AdvancedParam adv,
			 PosPID_FilterConfig filter_cfg,
			 float (*err_cb)(PosPID_t*, float, float));
float PosPID_Calculation(PosPID_t* pid, float target, float input);

void PosPID_ResetRunStatus(PosPID_t* pid); 



/*************
== 误差获取函数
*************/
/* ==位置式PID== */
/* Default */
float PosErrGet_Default(PosPID_t* pid, float target, float input);













#endif 
