#include "MarkBase42C.h"

#include "stdio.h"
#include "stdint.h"
#include "usart.h"

//#include "qaq_Commute.h"
//#include "WitIMU.h"
#include "stdlib.h"

// StMo: StepMotor
#define  StMoUART	huart8


static uint8_t GetCRC(uint8_t *buffer,uint8_t size) 
{ 
	buffer[size-1] = 0;
	uint16_t sum=0; 
	for(uint8_t i=0;i<size;i++)
	{ 
	  sum += buffer[i]; 
	} 
	return(sum&0xFF);     //返回校验和
} 

unsigned char FrameHead42c = 0;	
unsigned char len42c  = 8; 		
unsigned char UartStMoFlag = 0;	
unsigned char StMoRxData = 0; 	
void StMoReadInfo(StMoName_t StMoAddress, StMoinfo_t InfoType) 
{  
	if(UartStMoFlag == 1) //串口是否被占用, 如果有什么读取操作没有完成, 那在读取完成前所有的读取操作都是无效的
	{ // 串口被占用, 跳出函数, 读取无效
		return; 
		
	} 
	
	//串口未被占用, 更新需要发送的指令
	uint8_t temp[3]; temp[2] = 0; 
	
	temp[0] = StMoAddress; 
	temp[1] = InfoType; 
	temp[2] = GetCRC(temp, sizeof(temp)); 
	
	FrameHead42c = StMoAddress;
	switch(InfoType)
	{
		case TotalEnCnts:
		{
			len42c = 8;
			break;
		}
		case TotalInputPul:
		{
			len42c = 6;
			break;
		}
		case ToTalAngle:
		{ 
			len42c = 6;
			break;
		} 
		case AngleErr:
		{  
			len42c = 4; 
			break; 
		} 
		
	} 
	
	HAL_UART_Transmit(&StMoUART, temp, sizeof(temp), HAL_MAX_DELAY); 
	UartStMoFlag = 1;  
	HAL_UART_Receive_IT(&StMoUART ,&StMoRxData, 1); 

}


int MB_SumPos = 0; //用于记录通过此指令,总共走了多少Position


void StMoRunTarPos(StMoName_t StMoAddress, short Speed,  int Position)
{ 
	uint8_t temp[8]; 
	temp[0] = StMoAddress;
	temp[1] = 0XFD;
	
	if(Speed*Position > 0)//正转 
	{
		temp[2] = 0X00; 
		MB_SumPos += Position;
	} 
	else //if(Position<0)//反转 
	{ 
		temp[2] = 0X80; 
		MB_SumPos -= Position;
	}
//	else 
//	{ return; } 
	
	if(Speed<128) 
	{
		temp[2] = temp[2] | Speed;
	}else
	{ return; }
	
	temp[3] = Position >> 24;
	temp[4] = Position >> 16;
	temp[5] = Position >> 8;
	temp[6] = Position;
	
	temp[7] = GetCRC(temp, sizeof(temp)); 
	
	HAL_UART_Transmit(&StMoUART, temp, sizeof(temp), HAL_MAX_DELAY); //为什么中断方式发送不能用??
}

void StMoRunTarSpd(StMoName_t StMoAddress, short Speed ) //速度有128个档位
{ 
	uint8_t temp[4]; 
	temp[0] = StMoAddress;
	temp[1] = 0XF6;
	
	if(Speed>0)//正转 
	{
		temp[2] = 0X08; 
	} 
	else if(Speed<0)//反转 
	{ 
		temp[2] = 0X00; 
	}else
	{ return; }
	if(Speed<128)
	{
		temp[2] = temp[2] | Speed;
	}else
	{ return; } 
	
	temp[3] = GetCRC(temp, sizeof(temp)); 
	HAL_UART_Transmit(&StMoUART, temp, sizeof(temp), HAL_MAX_DELAY);
	
}

void StMoCalibrate(void) //所有地址的电机都矫正一遍
{
// 校准编码器前，请确保电机没带负载！！！建议校准好以后再装进机器。
//校准成功，返回 e0 01 e1 
//校准失败，返回 e0 02 e2 
	uint8_t temp[] = {0XE0, 0X80, 0X00, 0X60};
	for(int i=0; i<10; i++)
	{ 
		temp[0]+=i;
		temp[3]+=i;
		HAL_UART_Transmit(&StMoUART, temp, sizeof(temp), HAL_MAX_DELAY);
		
	}  
	return;
}
 
void StMoSetMotType(StMoName_t StMoAddress, MotType_t Type)
{
//设置成功，返回 e0 01 e1；
//设置失败，返回 e0 00 e0。
//设置电机类型后，需要重新校准编码器，校准前要断开电机负载。
	uint8_t temp[4]; 
	temp[0] = StMoAddress; 
	temp[1] = 0X81; 
	
	if(Type == MotType_09 ) 
	{
		temp[2] = 0X00; 
	}
	else if(Type == MotType_18 )
	{
		temp[2] = 0X01; 
	}
	else
	{ return; }
	temp[3] = GetCRC(temp, sizeof(temp)); 
	HAL_UART_Transmit(&StMoUART, temp, sizeof(temp), HAL_MAX_DELAY);
	
} 

void StMoSetCurrent(StMoName_t StMoAddress, unsigned short Current)
{
//00 对应 0ma
//01 对应 200ma
//…
//0E 对应 2800ma
//0F 对应 3000ma
//比如：发送 e0 83 06 69，设置电流 1200ma。
//设置成功，返回 e0 01 e1；
//设置失败，返回 e0 00 e0。
	uint8_t temp[4]; 
	temp[0] = StMoAddress; 
	temp[1] = 0X83; 
	if(Current>0 && Current<=3000)
	{ 
		temp[2] = Current/200; 
		temp[3] = GetCRC(temp, sizeof(temp)); 
		HAL_UART_Transmit(&StMoUART, temp, sizeof(temp), HAL_MAX_DELAY); 
		
	} 
}

void StMoSetMStep(StMoName_t StMoAddress, unsigned char MStep)
{ 
//设置成功，返回 e0 01 e1；
//设置失败，返回 e0 00 e0。
	uint8_t temp[4]; 
	temp[0] = StMoAddress; 
	temp[1] = 0X84; 
	temp[2] = MStep; 
	temp[3] = GetCRC(temp, sizeof(temp)); 
	HAL_UART_Transmit(&StMoUART, temp, sizeof(temp), HAL_MAX_DELAY); 
	
} 



void StMoSetPositiveDir(StMoName_t StMoAddress, unsigned char CW)
{ 
//设置成功，返回 e0 01 e1；
//设置失败，返回 e0 00 e0。
	uint8_t temp[4]; 
	temp[0] = StMoAddress; 
	temp[1] = 0X86;
	if(CW == 0 || CW == 1) //0:顺时针旋转为正方向, 1:逆时针旋转为正方向
	{
		temp[2] = CW; 
	}
	else
	{ return; }
	
	temp[3] = GetCRC(temp, sizeof(temp)); 
	HAL_UART_Transmit(&StMoUART, temp, sizeof(temp), HAL_MAX_DELAY); 
	
} 


void StMoSetUartBaud(StMoName_t StMoAddress, unsigned int UartBaud)
{ 
//设置成功，返回 e0 01 e1；
//设置失败，返回 e0 00 e0。
	uint8_t temp[4]; 
	temp[0] = StMoAddress; 
	temp[1] = 0X8A; 
	switch(UartBaud) 
	{
		case 9600:
		{	temp[2] = 0X01; 
			break;
		}
		case 19200:
		{	temp[2] = 0X02; 
			break;
		}
		case 25000:
		{	temp[2] = 0X03; 
			break;
		}
		case 38400:
		{	temp[2] = 0X04; 
			break;
		}
		case 57600:
		{	temp[2] = 0X05; 
			break;
		}
		case 115200:
		{	temp[2] = 0X06; 
			break;
		}
		default:break;
	}
	temp[3] = GetCRC(temp, sizeof(temp)); 
	HAL_UART_Transmit(&StMoUART, temp, sizeof(temp), HAL_MAX_DELAY); 
	
} 

void StMoSetPara(StMoName_t StMoAddress, StMoPara_t ParaType, unsigned short Value)
{ 
//Kp 默认值为 0x650(1616) 
//Ki 默认值为 1
//Kd 默认值为 0x650(1616) 
	//PID 参数设置不当，电机可能震动，请谨慎设置参数！！！
	
//ACC 默认值为 0x11E(286)
	//ACC 设置过大，可能损坏驱动板，请谨慎设置参数！！！
	
//MaxT 默认值为 0x4B0(1200); MaxT 取值范围（0 ~ 0x4B0）
	uint8_t temp[5]; 
	temp[0] = StMoAddress; 
	temp[1] = ParaType; 
	
	temp[2] = Value>>8; 
	temp[3] = Value; 
	
	temp[4] = GetCRC(temp, sizeof(temp)); 
	HAL_UART_Transmit(&StMoUART, temp, sizeof(temp), HAL_MAX_DELAY);
	
//设置成功，返回 e0 01 e1；
//设置失败，返回 e0 00 e0。
} 



