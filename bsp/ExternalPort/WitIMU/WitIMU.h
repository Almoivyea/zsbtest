#ifndef __WITIMU_H
#define __WITIMU_H

#include "stdint.h"
#include "usart.h"


#define IMU_UART	UART7
#define IMU_huart	huart7

extern DMA_HandleTypeDef hdma_uart7_rx;
#define hdma_IMU_rx	hdma_uart7_rx



//extern uint8_t IMU_RxDate; //串口中断单个字节接收 所用变量

typedef enum
{
	qaq_Roll=0,
	qaq_Pitch,
	qaq_Yaw,
	
	qaq_AccX,  // ?g, (g可取9.8m/s2)
	qaq_AccY,
	qaq_AccZ,
	
	qaq_GyroX, // 度每秒
	qaq_GyroY,
	qaq_GyroZ,
	
	/* 以下功能还没实现, 有需要再写 (Doge
//	qaq_MagneticX,
//	qaq_MagneticY,
//	qaq_MagneticZ,
//	
//	qaq_Height,
	
//	qaq_Time,
//	qaq_Quater,
//	qaq_Temperature,*/
	
	
}IMU_DateType;


//DMA+空闲模式 接口
float User_ImuDate(IMU_DateType DateType); //读取用户处理过的数据
float WitIMU_GetDate(IMU_DateType DateType);// 用于读取陀螺仪报文数据
void WitIMU_CopeRegData(void);// 用于更新数据
void WitIMU_CopeRxedRawData_DMA(uint8_t* Rxed_Date); //接收完成 处理函数


//串口中断模式 接口 
//float WitIMU_GetDate(IMU_DateType DateType);// 用于读取陀螺仪报文数据
//void WitIMU_CopeRawData(uint8_t input);// 用于串口, 接收原始数据
//void WitIMU_CopeRegData(void);// 用于更新数据


/* 空闲DMA为例

#include "WitIMU.h"

开启对应事件中断,失能半中断
uint8_t IMU_DMA_Cplt_Array[22]; 
	HAL_UARTEx_ReceiveToIdle_DMA(&IMU_huart,IMU_DMA_Cplt_Array,sizeof(IMU_DMA_Cplt_Array) ); 
	__HAL_DMA_DISABLE_IT(&IMU_dma_rx,DMA_IT_HT);

编写对应事件回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{ 
	if(huart == &IMU_huart)
	{ 
		WitIMU_CopeRxedRawData_DMA( IMU_DMA_Cplt_Array );	
		WitIMU_CopeRegData(); 	

		//再次开启
		HAL_UARTEx_ReceiveToIdle_DMA(&IMU_huart,IMU_DMA_Cplt_Array,sizeof(IMU_DMA_Cplt_Array) ); 
		__HAL_DMA_DISABLE_IT(&hdma_uart7_rx,DMA_IT_HT);
		
	}
} 

此后要读取数据调用:
ImuDate(IMU_DateType DateType); //读取用户处理过的数据 (要怎么处理,看需求,这里转化为0~360度)
或
WitIMU_GetDate(IMU_DateType DateType);// 用于读取陀螺仪报文数据 (0~180, 0~-180)

*/











#endif

