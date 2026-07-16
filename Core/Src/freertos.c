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
#include "imu_attitude.h"
#include "motor.h"
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
extern fp32 gyro[3];
extern fp32 accel[3];
extern fp32 temp;

/* Minimum unused stack observed since each task started, in 32-bit words. */
volatile UBaseType_t motor_stack_free = 0;
volatile UBaseType_t comm_stack_free = 0;

/* Same values converted to bytes for convenient inspection in Keil Watch. */
volatile uint32_t motor_stack_free_bytes = 0;
volatile uint32_t comm_stack_free_bytes = 0;
/* USER CODE END Variables */
osThreadId MotorCtrlTaskHandle;
osThreadId CommTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void MotorCtrl(void const * argument);
void Common(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
  (void)xTask;
  (void)pcTaskName;

  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}
/* USER CODE END 5 */

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
  /* definition and creation of MotorCtrlTask */
  osThreadDef(MotorCtrlTask, MotorCtrl, osPriorityHigh, 0, 256);
  MotorCtrlTaskHandle = osThreadCreate(osThread(MotorCtrlTask), NULL);

  /* definition and creation of CommTask */
  osThreadDef(CommTask, Common, osPriorityNormal, 0, 256);
  CommTaskHandle = osThreadCreate(osThread(CommTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_MotorCtrl */
/**
  * @brief  Function implementing the MotorCtrlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MotorCtrl */
__weak void MotorCtrl(void const * argument)
{
  /* USER CODE BEGIN MotorCtrl */
  TickType_t xLastWakeTime;

  (void)argument;

  /* BMI088 may not be ready immediately after power-up. */
  while (BMI088_init()) {
    osDelay(10);
  }

  IMU_Attitude_Init();
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    BMI088_read(gyro, accel, &temp);
    IMU_Attitude_Update(gyro, accel, MOTOR_CONTROL_PERIOD_S);

    Motor_Control_Update_2ms();
    Buzzer_Update_2ms();

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
  }
  /* USER CODE END MotorCtrl */
}

/* USER CODE BEGIN Header_Common */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Common */
__weak void Common(void const * argument)
{
  /* USER CODE BEGIN Common */
  (void)argument;

  for (;;) {
    motor_stack_free = uxTaskGetStackHighWaterMark(MotorCtrlTaskHandle);
    comm_stack_free = uxTaskGetStackHighWaterMark(CommTaskHandle);

    motor_stack_free_bytes = (uint32_t)motor_stack_free * sizeof(StackType_t);
    comm_stack_free_bytes = (uint32_t)comm_stack_free * sizeof(StackType_t);

    /* Reserved for telemetry and CAN/remote/motor supervision. */
    osDelay(1000);
  }
  /* USER CODE END Common */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
