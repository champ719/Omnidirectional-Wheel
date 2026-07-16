#include "Motor_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Buzzer.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Motor_Drv.h"
#include "Remote.h"
#include "Robot_Control.h"
#include "USER_CAN.h"

static void Task_Motor_Output_Callback(void)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) {
        Motor_Drv_SetWheelTorque(i, chassis_control.wheel_output[i]);
    }
    Motor_Drv_SetGimbalTorque(0U, gimbal_control.yaw_output);
    Motor_Drv_SetGimbalTorque(1U, gimbal_control.pitch_output);
    (void)Motor_Drv_Send_All();
}

void Motor_Task_Init(void)
{
    Motor_Drv_Init();
    CAN_Init();
    Remote_Init();
    Buzzer_Init();
    Robot_Control_Init();
}

void OS_MotorCallback(void const *argument)
{
    TickType_t last_wake;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        Robot_Control_Update_2ms();
        Task_Motor_Output_Callback();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2U));
    }
}
