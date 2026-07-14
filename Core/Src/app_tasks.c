#include "app_tasks.h"
#include "main.h"
#include "tim.h"
#include "motor.h"
#include "BMI088driver.h"
#include "Remote.h"

/* Task handles */
osThreadId MotorCtrlTaskHandle;
osThreadId CommTaskHandle;

/* Task function declarations - CMSIS-RTOS v1 style */
void MotorCtrlTask(void const *argument);
void CommTask(void const *argument);

osThreadDef(MotorCtrlTask, MotorCtrlTask, osPriorityHigh, 1, MOTOR_CTRL_TASK_STACK);
osThreadDef(CommTask, CommTask, osPriorityNormal, 1, COMM_TASK_STACK);

/* Global sensor data */
extern fp32 gyro[3], accel[3], temp;

/* FreeRTOS stack overflow hook (required by configCHECK_FOR_STACK_OVERFLOW=2) */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    /* Debugger breakpoint: inspect xTask and pcTaskName here. */
    __disable_irq();
    for (;;) {}
}

/*
 * MotorCtrlTask - 2ms cycle control task
 * Uses RTOS vTaskDelayUntil for precise timing (replaces TIM6 flag polling)
 */
void MotorCtrlTask(void const *argument)
{
    TickType_t xLastWakeTime;

    /* Initialize sensor - retry until success */
    while (BMI088_init()) {
        osDelay(10);
    }

    /* Initialize the wake time */
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        /* Read BMI088 gyro + accel + temperature */
        BMI088_read(gyro, accel, &temp);

        /* Run motor control update (2ms cycle) */
        Motor_Control_Update_2ms();

        /* Precise 2ms period - compensates for execution time */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
    }
}

/*
 * CommTask - Communication / debug monitoring (100ms cycle)
 * Reads CAN status, remote status, sensor data for debug
 */
void CommTask(void const *argument)
{
    for (;;) {
        osDelay(100);  /* 100ms debug/supervision period */

        /* TODO: Add telemetry/debug output here:
         * - printf("gyro: %.2f,%.2f,%.2f  accel: %.2f,%.2f,%.2f  temp:%.1f\r\n",
         *          gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], temp);
         * - CAN bus error monitoring
         * - Motor state monitoring
         */
    }
}

void AppTasks_Init(void)
{
    /* Start TIM6 as base timer (no interrupt needed for task sync) */
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
        Error_Handler();
    }

    /* Create RTOS tasks */
    MotorCtrlTaskHandle = osThreadCreate(osThread(MotorCtrlTask), NULL);
    CommTaskHandle = osThreadCreate(osThread(CommTask), NULL);

    if (MotorCtrlTaskHandle == NULL || CommTaskHandle == NULL) {
        Error_Handler();
    }
}