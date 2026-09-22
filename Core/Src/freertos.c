/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "BMI088driver.h"
#include "imu_temp_ctrl.h"
#include "Buzzer.h"
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
osThreadId IMUTaskHandle;
osThreadId MotorTaskHandle;
osThreadId ErrorTaskHandle;
osThreadId ChassisHandle;
osThreadId GimbalHandle;
osThreadId RcTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void OS_IMUCallback(void const * argument);
void OS_MotorCallback(void const * argument);
void OS_ErrorCallback(void const * argument);
void OS_ChassisCallback(void const * argument);
void OS_GimbalCallback(void const * argument);
void OS_RcCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, OS_IMUCallback, osPriorityNormal, 0, 1024);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, OS_MotorCallback, osPriorityNormal, 0, 512);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of ErrorTask */
  osThreadDef(ErrorTask, OS_ErrorCallback, osPriorityRealtime, 0, 256);
  ErrorTaskHandle = osThreadCreate(osThread(ErrorTask), NULL);

  /* definition and creation of Chassis */
  osThreadDef(Chassis, OS_ChassisCallback, osPriorityNormal, 0, 256);
  ChassisHandle = osThreadCreate(osThread(Chassis), NULL);

  /* definition and creation of Gimbal */
  osThreadDef(Gimbal, OS_GimbalCallback, osPriorityNormal, 0, 256);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

  /* definition and creation of RcTask */
  /* 遥控输入层要抢在控制环前拿到最新帧，优先级高于 Chassis/Gimbal */
  osThreadDef(RcTask, OS_RcCallback, osPriorityAboveNormal, 0, 256);
  RcTaskHandle = osThreadCreate(osThread(RcTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_OS_IMUCallback */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_IMUCallback */
__weak void OS_IMUCallback(void const * argument)
{
  /* USER CODE BEGIN OS_IMUCallback */
  (void)argument;
  for(;;)
  {
    vTaskSuspend(NULL);
  }
  /* USER CODE END OS_IMUCallback */
}

/* USER CODE BEGIN Header_OS_MotorCallback */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_MotorCallback */
__weak void OS_MotorCallback(void const * argument)
{
  /* USER CODE BEGIN OS_MotorCallback */
  (void)argument;
  for(;;)
  {
    vTaskSuspend(NULL);
  }
  /* USER CODE END OS_MotorCallback */
}

/* USER CODE BEGIN Header_OS_ErrorCallback */
/**
* @brief Function implementing the ErrorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_ErrorCallback */
__weak void OS_ErrorCallback(void const * argument)
{
  /* USER CODE BEGIN OS_ErrorCallback */
  (void)argument;
  for(;;)
  {
    vTaskSuspend(NULL);
  }
  /* USER CODE END OS_ErrorCallback */
}

/* USER CODE BEGIN Header_OS_ChassisCallback */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_ChassisCallback */
__weak void OS_ChassisCallback(void const * argument)
{
  /* USER CODE BEGIN OS_ChassisCallback */
  (void)argument;
  for(;;)
  {
    vTaskSuspend(NULL);
  }
  /* USER CODE END OS_ChassisCallback */
}

/* USER CODE BEGIN Header_OS_GimbalCallback */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_GimbalCallback */
__weak void OS_GimbalCallback(void const * argument)
{
  /* USER CODE BEGIN OS_GimbalCallback */
  (void)argument;
  for(;;)
  {
    vTaskSuspend(NULL);
  }
  /* USER CODE END OS_GimbalCallback */
}

/* USER CODE BEGIN Header_OS_RcCallback */
/**
* @brief Function implementing the RcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_RcCallback */
__weak void OS_RcCallback(void const * argument)
{
  /* USER CODE BEGIN OS_RcCallback */
  (void)argument;
  for(;;)
  {
    vTaskSuspend(NULL);
  }
  /* USER CODE END OS_RcCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void vApplicationStackOverflowHook(TaskHandle_t task,
                                   char *task_name)
{
  (void)task;
  (void)task_name;

  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

/* USER CODE END Application */
