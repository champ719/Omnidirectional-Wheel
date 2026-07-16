#include "Error.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Chassis.h"
#include "Motor_Drv.h"
#include "imu_attitude.h"
#include "Remote.h"

static volatile Error_Result_t error_result = ERROR_RESULT_REMOTE_OFFLINE;

static void Error_EnterEmergencyState(void)
{
    Motor_Drv_StopAll();

    for (;;) {
        (void)Motor_Drv_Send_All();

        if (rc_ctrl.rc.s1 == RC_SW_MID) {
            HAL_NVIC_SystemReset();
        }

        HAL_Delay(1U);
    }
}

void Error_Init(void)
{
    error_result = ERROR_RESULT_REMOTE_OFFLINE;
}

Error_Result_t Error_GetResult(void)
{
    return error_result;
}

void Error_Task_Update_2ms(void)
{
    error_result = Error_Update(
        Remote_IsOnline(),
        rc_ctrl.rc.s1,
        (rc_ctrl.rc.s2 == RC_SW_DOWN) ? 1U : 0U,
        IMU_Attitude_IsReady(),
        Chassis_Ctrl_DirectionsCalibrated());
}

Error_Result_t Error_Update(uint8_t remote_online,
                            uint8_t remote_switch,
                            uint8_t rotate_requested,
                            uint8_t imu_ready,
                            uint8_t directions_calibrated)
{
    if (remote_online == 0U) {
        return ERROR_RESULT_REMOTE_OFFLINE;
    }

    if (remote_switch == RC_SW_DOWN) {
        return ERROR_RESULT_EMERGENCY_STOP;
    }

    if (remote_switch != RC_SW_MID) {
        return ERROR_RESULT_STOPPED;
    }

    if (directions_calibrated == 0U) {
        return ERROR_RESULT_DIRECTION_UNCALIBRATED;
    }

    if ((rotate_requested != 0U) && (imu_ready == 0U)) {
        return ERROR_RESULT_IMU_NOT_READY;
    }

    return ERROR_RESULT_NONE;
}

void OS_ErrorCallback(void const *argument)
{
    TickType_t last_wake;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        Error_Task_Update_2ms();
        if (Error_GetResult() == ERROR_RESULT_EMERGENCY_STOP) {
            Error_EnterEmergencyState();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2U));
    }
}
