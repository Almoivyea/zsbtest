#include "qaq_uart.h"

#include "usart.h"

#include "WitIMU.h" 
#include "WitIMU_REG.h"
#include "seekfree.h"

uint8_t IMU_DMA_Array[11];
float WitDate[10]; 


void qaq_IMU_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&IMU_huart,IMU_DMA_Array,sizeof(IMU_DMA_Array) ); 
	__HAL_DMA_DISABLE_IT(&hdma_IMU_rx,DMA_IT_HT); 
	
//	HAL_UARTEx_ReceiveToIdle_DMA(&ViewChat_huart,ViewChatArray,sizeof(ViewChatArray) ); 
//	__HAL_DMA_DISABLE_IT(&hdma_ViewChat_rx,DMA_IT_HT); 
	
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{ 
	if(huart == &IMU_huart)
	{ 
		WitIMU_CopeRxedRawData_DMA( IMU_DMA_Array );	
		WitIMU_CopeRegData(); 	
		WitDate[0] = WitIMU_GetDate(qaq_Roll);
		WitDate[1] = WitIMU_GetDate(qaq_Pitch);
		WitDate[2] = WitIMU_GetDate(qaq_Yaw);

		//Restore
		HAL_UARTEx_ReceiveToIdle_DMA(&IMU_huart,IMU_DMA_Array,sizeof(IMU_DMA_Array) ); 
		__HAL_DMA_DISABLE_IT(&hdma_IMU_rx, DMA_IT_HT);
		
	}

} 



//uint8_t IMU_DMA_Array[11];
//void qaq_IMU_Init(void)
//{
//	HAL_UARTEx_ReceiveToIdle_DMA(&IMU_huart,IMU_DMA_Array,sizeof(IMU_DMA_Array) ); 
//	__HAL_DMA_DISABLE_IT(&hdma_IMU_rx,DMA_IT_HT); 
//	
//} 


//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{ 
//	if(huart == &IMU_huart)
//	{ 
//		WitIMU_CopeRxedRawData_DMA( IMU_DMA_Array );	
//		WitIMU_CopeRegData(); 	
//		
//		//function_test
//// 		HAL_UART_Transmit_DMA(&IMU_huart,IMU_DMA_Array,Size); 

//		//Restore
//		HAL_UARTEx_ReceiveToIdle_DMA(&IMU_huart,IMU_DMA_Array,sizeof(IMU_DMA_Array) ); 
//		__HAL_DMA_DISABLE_IT(&hdma_IMU_rx, DMA_IT_HT);

//	}
//} 




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
	if(huart->Instance==DEBUG_USART_IT)	// SeekFree	
    { 
        Debug_Receive(); 
		
    } 
}














