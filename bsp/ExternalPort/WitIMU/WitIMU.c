#include "WitIMU.h"
#include "WitIMU_REG.h"

#include "string.h"

/* 串口中断接收流程:
单字节的串口中断, 调用 WitIMU_CopeRawData();//处理串口初始数据
	将巉口单字节数据转化为需要的short数据
轮询任务 WitIMU_CopeRegData(); //处理 以寄存器值为索引的数组(sReg) 的数据 

*/


#define RAW_DATA_BUFF_SIZE 11   // 原 22 ?

uint8_t IMU_RxDate=0;

static uint8_t RawDataBuff[RAW_DATA_BUFF_SIZE];// 用于储存从串口新收到的原始数据
static uint32_t RawDataCnt = 0;// 用于给上面这个数组计数
	
int16_t sReg[REGSIZE];// 以寄存器地址为索引的数组

uint8_t DateCategoryIndex = 0;
uint16_t usRegDataBuff[4] = {0};
uint32_t uiRegDataLen = 0;

float User_ImuDate(IMU_DateType DateType)
{
	switch(DateType)
	{
		case qaq_Yaw: 
		{
			float WitIMU = WitIMU_GetDate(DateType); 
			
			return ( (WitIMU<0)?(WitIMU+360.0f):WitIMU ); 

		}
		
		
		
		
		
		
		default: return WitIMU_GetDate(DateType); 
	}
}




float WitIMU_GetDate(IMU_DateType DateType)
{
	float fAcc[3], fGyro[3], fAngle[3];
	for(int i = 0; i < 3; i++)
	{
		fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
		fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
		fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
	}
	switch(DateType)
	{
		case qaq_Roll:
			return fAngle[0];
		case qaq_Pitch:
			return fAngle[1];
		case qaq_Yaw:
			return fAngle[2];
		case qaq_AccX:
			return fAcc[0];
		case qaq_AccY:
			return fAcc[1];
		case qaq_AccZ:
			return fAcc[2];
		case qaq_GyroX:
			return fGyro[0];
		case qaq_GyroY:
			return fGyro[1];
		case qaq_GyroZ:
			return fGyro[2];
			
//		case qaq_Height:
			
		default :
			return -120; //错误码
	}
}

	// RegDate就是将接收到的原始数据的高低位融合(初步处理), 得到的数据
void WitIMU_CopeRegData(void)// 把[初步处理的值] 储存在 sReg数组的对应位置
{
    uint32_t uiReg1 = 0, uiReg2 = 0, uiReg1Len = 0, uiReg2Len = 0; //一帧数据内,最多包含两种数据. 这里的变量记录对应的索引值,及其数据长度
    uint16_t *p_usReg1Val = usRegDataBuff; //通过指针来把刚刚处理好的串口数据,放入sReg数组
    uint16_t *p_usReg2Val = usRegDataBuff+3; 
	
    uiReg1Len = 4; //默认为4,因为很多类的数据不需要分离
    switch(DateCategoryIndex)//通过协议头,分离处理好的串口数据
    {
        case WIT_ANGLE:	uiReg1 = Roll;  	uiReg1Len = 3;  uiReg2 = VERSION;  	uiReg2Len = 1;
			break;// 协议头为角度
		case WIT_ACC:	uiReg1 = AX;    	uiReg1Len = 3;  uiReg2 = TEMP;  	uiReg2Len = 1;
			break;// 协议头为角加速度
		case WIT_GYRO:  uiReg1 = GX;  		uiReg1Len = 3;
			break;// 协议头为角速度
		case WIT_MAGNETIC: uiReg1 = HX;  	uiReg1Len = 3;
			break;// 协议头为磁力计
        case WIT_PRESS: uiReg1 = PressureL; uiReg1Len = 2;	uiReg2 = HeightL;  	uiReg2Len = 2;
			break;// 协议头为气压, 高度
        case WIT_QUATER:    uiReg1 = q0; 
			break;// 协议头为四元数
//        case WIT_TIME:  uiReg1 = YYMM; //若需要使用, 徐修改其接收到的原始数据的初处理
//			break;// 协议头为时间
		
//        case WIT_REGVALUE:  uiReg1 = s_uiReadRegIndex;
//			break;// 协议头为 读取寄存器
		default:
			return ;
    }
if(uiRegDataLen == 3)
{
	uiReg1Len = 3;
	uiReg2Len = 0;
}
    if(uiReg1Len)//将处理好的值,储存在sReg数组中
	{
		memcpy(&sReg[uiReg1], p_usReg1Val, uiReg1Len<<1);

	}
    if(uiReg2Len)
	{
		memcpy(&sReg[uiReg2], p_usReg2Val, uiReg2Len<<1);

	}
}



static uint8_t CalSum(uint8_t *data, uint32_t len)// 和校验
{ uint8_t sum = 0; for(int i=0; i<len; i++){sum += *(data + i);} return sum; }

void WitIMU_CopeRawData(uint8_t input)//把8位的串口原始数据处理成16位的数据
{
	uint8_t VerifySum;
    static uint16_t ProcessedDate[4];
	
    RawDataBuff[RawDataCnt++] = input;

	if(RawDataBuff[0] != 0x55) ////////////////判断 串口原始数据接收数组 的第一个数据(陀螺仪ID号)
	{
		RawDataCnt--; //退位,因为接收到input后+1了
		memcpy(RawDataBuff, &RawDataBuff[1], RawDataCnt); //掩盖掉不对的数据
		return ; 
	}
	
	if(RawDataCnt == 11)//成功接收到了11字节合法数据 //原来是: if(RawDataCnt >= 11)   why?
	{
		VerifySum = CalSum(RawDataBuff, 10);// 计算校验位
		if(VerifySum != RawDataBuff[10])//判断校验位
		{
			RawDataCnt--; //退位,因为接收到input后+1了. 退位后的值才可用于下面的数据掩盖
			memcpy(RawDataBuff, &RawDataBuff[1], RawDataCnt); 
			return ;
		}
		//校验位合法, 进行数据处理
		ProcessedDate[0] = ((uint16_t)RawDataBuff[3] << 8) | (uint16_t)RawDataBuff[2];
		ProcessedDate[1] = ((uint16_t)RawDataBuff[5] << 8) | (uint16_t)RawDataBuff[4];
		ProcessedDate[2] = ((uint16_t)RawDataBuff[7] << 8) | (uint16_t)RawDataBuff[6];
		ProcessedDate[3] = ((uint16_t)RawDataBuff[9] << 8) | (uint16_t)RawDataBuff[8];
		RawDataCnt = 0;
		
		DateCategoryIndex = RawDataBuff[1]; //记录刚刚接收到的是哪类的数据
		memcpy(usRegDataBuff,ProcessedDate,8); //储存处理好的串口数据(short数据)
		uiRegDataLen = 4; //算是标志位的用法吧
	}
	
    if(RawDataCnt == RAW_DATA_BUFF_SIZE){ RawDataCnt = 0;} 
}



void WitIMU_CopeRxedRawData_DMA(uint8_t* Rxed_Date) //把8位的原始数据处理成16位的数据
{
	uint8_t VerifySum;
    static uint16_t ProcessedDate[4];
	
	if(Rxed_Date[0]==0x55)/////////////////////////
	{
		for(int i=0; i<10; i++)
		{
			VerifySum += Rxed_Date[i]; 
			
		}
		if(VerifySum != Rxed_Date[10])
		{
			memset(Rxed_Date, 0, 15); 
			return;; 
			
		}
		ProcessedDate[0] = ((uint16_t)Rxed_Date[3] << 8) | (uint16_t)Rxed_Date[2];
		ProcessedDate[1] = ((uint16_t)Rxed_Date[5] << 8) | (uint16_t)Rxed_Date[4];
		ProcessedDate[2] = ((uint16_t)Rxed_Date[7] << 8) | (uint16_t)Rxed_Date[6];
		ProcessedDate[3] = ((uint16_t)Rxed_Date[9] << 8) | (uint16_t)Rxed_Date[8];
	RawDataCnt = 0;
		
		DateCategoryIndex = Rxed_Date[1];
		memcpy(usRegDataBuff,ProcessedDate,8);
		uiRegDataLen = 4;

	}
		
}


