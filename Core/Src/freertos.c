/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for LiftTask */
osThreadId_t LiftTaskHandle;
const osThreadAttr_t LiftTask_attributes = {
  .name = "LiftTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TrayTask */
osThreadId_t TrayTaskHandle;
const osThreadAttr_t TrayTask_attributes = {
  .name = "TrayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CaptureTask */
osThreadId_t CaptureTaskHandle;
const osThreadAttr_t CaptureTask_attributes = {
  .name = "CaptureTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for VIewTask */
osThreadId_t VIewTaskHandle;
const osThreadAttr_t VIewTask_attributes = {
  .name = "VIewTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void App_ControlTask(void *argument);
void App_ChassisTask(void *argument);
void App_GimbalTask(void *argument);
void App_DebugTask(void *argument);
void App_LiftTask(void *argument);
void AppTrayTask(void *argument);
void App_CaptureTask(void *argument);
void App_VIewTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(App_ControlTask, NULL, &ControlTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(App_ChassisTask, NULL, &ChassisTask_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(App_GimbalTask, NULL, &GimbalTask_attributes);

  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(App_DebugTask, NULL, &DebugTask_attributes);

  /* creation of LiftTask */
  LiftTaskHandle = osThreadNew(App_LiftTask, NULL, &LiftTask_attributes);

  /* creation of TrayTask */
  TrayTaskHandle = osThreadNew(AppTrayTask, NULL, &TrayTask_attributes);

  /* creation of CaptureTask */
  CaptureTaskHandle = osThreadNew(App_CaptureTask, NULL, &CaptureTask_attributes);

  /* creation of VIewTask */
  VIewTaskHandle = osThreadNew(App_VIewTask, NULL, &VIewTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  
//	  xSemaphoreTake();  portMAX_DELAY 
	  
//	  vTaskSuspend(); 
//	  vTaskResume(); 
	  
	 uint32_t QAQticks = 88;
	  	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_0); 
		osDelay(QAQticks);
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1); 
		osDelay(QAQticks); 
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_2); 
		osDelay(QAQticks);
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_3); 
		osDelay(QAQticks);
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_4); 
		osDelay(QAQticks);
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_5); 
		osDelay(QAQticks);
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_6); 
		osDelay(QAQticks);  
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_7); 
		osDelay(QAQticks);
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_8); 
		osDelay(QAQticks);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_App_ControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_App_ControlTask */
__weak void App_ControlTask(void *argument)
{
  /* USER CODE BEGIN App_ControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END App_ControlTask */
}

/* USER CODE BEGIN Header_App_ChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_App_ChassisTask */
__weak void App_ChassisTask(void *argument)
{
  /* USER CODE BEGIN App_ChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END App_ChassisTask */
}

/* USER CODE BEGIN Header_App_GimbalTask */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_App_GimbalTask */
__weak void App_GimbalTask(void *argument)
{
  /* USER CODE BEGIN App_GimbalTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END App_GimbalTask */
}

/* USER CODE BEGIN Header_App_DebugTask */
/**
* @brief Function implementing the DebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_App_DebugTask */
__weak void App_DebugTask(void *argument)
{
  /* USER CODE BEGIN App_DebugTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END App_DebugTask */
}

/* USER CODE BEGIN Header_App_LiftTask */
/**
* @brief Function implementing the LiftTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_App_LiftTask */
__weak void App_LiftTask(void *argument)
{
  /* USER CODE BEGIN App_LiftTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END App_LiftTask */
}

/* USER CODE BEGIN Header_AppTrayTask */
/**
* @brief Function implementing the TrayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTrayTask */
__weak void AppTrayTask(void *argument)
{
  /* USER CODE BEGIN AppTrayTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppTrayTask */
}

/* USER CODE BEGIN Header_App_CaptureTask */
/**
* @brief Function implementing the CaptureTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_App_CaptureTask */
__weak void App_CaptureTask(void *argument)
{
  /* USER CODE BEGIN App_CaptureTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END App_CaptureTask */
}

/* USER CODE BEGIN Header_App_VIewTask */
/**
* @brief Function implementing the VIewTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_App_VIewTask */
__weak void App_VIewTask(void *argument)
{
  /* USER CODE BEGIN App_VIewTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END App_VIewTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

