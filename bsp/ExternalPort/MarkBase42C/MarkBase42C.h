#ifndef __MARKBASE42C_H
#define __MARKBASE42C_H

#include "stdint.h"

typedef enum{
	M0 = 0XE0,
	M1 = 0XE1,
	M2 = 0XE2,
	M3 = 0XE3,
	
} StMoName_t; 

typedef enum{
	Kp = 0XA1,
	Ki = 0XA2,
	Kd = 0XA3,
	Acc= 0XA4,
	MaxT= 0XA5,
	
} StMoPara_t; 

typedef enum{
	TotalEnCnts  =0X30,	//记录上电后使能或不使能编码器记录的电机转动范围[编码值]（经过线性化校准和插值后的值）
	TotalInputPul=0X33,	//控制器发送过来的累计脉冲数
	ToTalAngle   =0X36,	//电机自上电/使能起所转过的[角度]  转一圈是 65536，十圈就是 655360,AndSoOn
	AngleErr     =0X39,	//想要控制的位置角度减去电机的实时角度位置得到的差值，单位：0~65535 表示 0~360°，
	//8, add+int32+uint16+chk
	//6, add+int32+chk
	//6, add+int32+chk
	//4, add+int16+chk
	
}  StMoinfo_t;

typedef enum{
	MotType_09, //0.9度
	MotType_18, //1.8度
	
}  MotType_t; 

#define huartStMo	huart8 
//extern unsigned char FrameHead42c ;	
//extern unsigned char len42c  ; 		
//extern unsigned char UartStMoFlag ;	
//extern unsigned char StMoRxData ; 	



 //电机设置
void StMoCalibrate(void); //所有地址的电机都矫正一遍 
void StMoSetMotType(StMoName_t StMoAddress, MotType_t Type);
void StMoSetCurrent(StMoName_t StMoAddress, unsigned short Current); 
void StMoSetMStep(StMoName_t StMoAddress, unsigned char MStep); 
void StMoSetPositiveDir(StMoName_t StMoAddress, unsigned char CW); 
void StMoSetUartBaud(StMoName_t StMoAddress, unsigned int UartBaud); 
void StMoSetPara(StMoName_t StMoAddress, StMoPara_t ParaType, unsigned short Value); 


 //数据读取
void StMoReadInfo(StMoName_t StMoAddress, StMoinfo_t InfoType); 
	
 //串口直接控制指令
extern int MB_SumPos; // MarkBase 
void StMoRunTarPos(StMoName_t StMoAddress, short Speed,  int Position); 
void StMoRunTarSpd(StMoName_t StMoAddress, short Speed ); 


#endif


