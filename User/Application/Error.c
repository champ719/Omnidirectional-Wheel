#include "Error.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Buzzer.h"
#include "Chassis.h"
#include "USER_CAN.h"
#include "motor.h"
#include "imu_attitude.h"
#include "Remote.h"

static volatile Error_Result_t error_result = ERROR_RESULT_REMOTE_OFFLINE;

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
    RC_Ctrl_t remote;
    Error_Result_t result;

    Remote_GetSnapshot(&remote);
    result = Error_Update(
        Remote_IsOnline(),
        remote.rc.s1,
        IMU_Attitude_IsReady(),
        1U);

    if (result == ERROR_RESULT_NONE) {
        if (CAN_IsHealthy() == 0U) {
            result = ERROR_RESULT_CAN_FAULT;
        } else if (Motor_FeedbackHealthy() == 0U) {
            result = ERROR_RESULT_MOTOR_FEEDBACK_TIMEOUT;
        }
    }
    error_result = result;
}

Error_Result_t Error_Update(uint8_t remote_online,
                            uint8_t remote_switch,
                            uint8_t imu_ready,
                            uint8_t directions_calibrated)
{
    if (remote_online == 0U) {
        return ERROR_RESULT_REMOTE_OFFLINE;
    }

    if (remote_switch == RC_SW_DOWN) {
        return ERROR_RESULT_EMERGENCY_STOP;
    }

    if ((remote_switch != RC_SW_MID) &&
        (remote_switch != RC_SW_UP)) {
        return ERROR_RESULT_STOPPED;
    }

    if (directions_calibrated == 0U) {
        return ERROR_RESULT_DIRECTION_UNCALIBRATED;
    }

    if (imu_ready == 0U) {
        return ERROR_RESULT_IMU_NOT_READY;
    }

    return ERROR_RESULT_NONE;
}

void OS_ErrorCallback(void const *argument)
{
    TickType_t last_wake;
    Error_Result_t previous_result = ERROR_RESULT_NONE;
    Error_Result_t current_result;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        Error_Task_Update_2ms();
        current_result = Error_GetResult();
        if ((current_result == ERROR_RESULT_EMERGENCY_STOP) &&
            (previous_result != ERROR_RESULT_EMERGENCY_STOP)) {
            Buzzer_PlayEmergencyDoubleBeep();
        }
        previous_result = current_result;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2U));
    }
}
