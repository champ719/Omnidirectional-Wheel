#include "Error.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Buzzer.h"
#include "Chassis.h"
#include "USER_CAN.h"
#include "motor.h"
#include "imu_temp_ctrl.h"
#include "Remote.h"

static volatile Error_Result_t error_result = ERROR_RESULT_REMOTE_OFFLINE;
static volatile uint8_t emergency_stop_triggered;

extern osThreadId ErrorTaskHandle;

/**
 * @brief 初始化故障监控状态。
 * @note 上电时默认认为遥控器离线，急停任务保持未触发状态。
 */
void Error_Init(void)
{
    error_result = ERROR_RESULT_REMOTE_OFFLINE;
    emergency_stop_triggered = 0U;
}

/**
 * @brief 获取最近一次故障检测结果。
 * @return 当前故障状态，ERROR_RESULT_NONE 表示系统允许正常控制。
 */
Error_Result_t Error_GetResult(void)
{
    return error_result;
}

/**
 * @brief 更新遥控器、IMU、CAN 和电机反馈的健康状态。
 * @note 该函数由电机控制任务周期调用，只更新结果，不直接发送电机指令。
 */
void Error_MonitorUpdate(void)
{
    RC_Ctrl_t remote;
    Error_Result_t result;

    Remote_GetSnapshot(&remote);
    result = Error_Update(
        Remote_IsOnline(),
        remote.rc.s1,
        IMU_Attitude_IsReady());

    if (result == ERROR_RESULT_NONE) {
        if (CAN_IsHealthy() == 0U) {
            result = ERROR_RESULT_CAN_FAULT;
        } else if (Motor_FeedbackHealthy() == 0U) {
            result = ERROR_RESULT_MOTOR_FEEDBACK_TIMEOUT;
        }
    }
    error_result = result;
}

/**
 * @brief 锁存急停触发标志并唤醒最高优先级 ErrorTask。
 * @note 重复调用不会重复唤醒；ErrorTask 唤醒后会持续发送零电流。
 */
void Error_TriggerEmergencyStop(void)
{
    if (emergency_stop_triggered != 0U) {
        return;
    }

    emergency_stop_triggered = 1U;
    if (ErrorTaskHandle != NULL) {
        (void)osThreadResume(ErrorTaskHandle);
    }
}

/**
 * @brief 按安全优先级判断基础故障状态。
 * @param remote_online 遥控器在线标志，1 表示在线。
 * @param remote_switch 遥控器右拨杆 S1 的当前位置。
 * @param imu_ready IMU 姿态解算就绪标志，1 表示就绪。
 * @return 当前基础故障结果；遥控离线的优先级高于拨杆急停。
 */
Error_Result_t Error_Update(uint8_t remote_online,
                            uint8_t remote_switch,
                            uint8_t imu_ready)
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

    if (imu_ready == 0U) {
        return ERROR_RESULT_IMU_NOT_READY;
    }

    return ERROR_RESULT_NONE;
}

/**
 * @brief 急停专用最高优先级任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 * @note 任务上电后先挂起自身；被急停触发唤醒后不再让出 CPU，持续维护
 *       CAN 并发送零电流。S1 回到中档或上档时执行 MCU 软件复位。
 */
void OS_ErrorCallback(void const *argument)
{
    RC_Ctrl_t remote;
    uint32_t buzzer_update_tick;

    (void)argument;
    (void)osThreadSuspend(ErrorTaskHandle);

    error_result = ERROR_RESULT_EMERGENCY_STOP;
    Chassis_ResetControl();
    Motor_STOP();
    Buzzer_PlayEmergencyDoubleBeep();
    buzzer_update_tick = HAL_GetTick();

    for (;;)
    {
        CAN_Service();
        Motor_STOP();

        if ((HAL_GetTick() - buzzer_update_tick) >= 2U) {
            buzzer_update_tick += 2U;
            Buzzer_Update_2ms();
        }

        Remote_GetSnapshot(&remote);
        if ((remote.rc.s1 == RC_SW_MID) ||
            (remote.rc.s1 == RC_SW_UP)) {
            NVIC_SystemReset();
        }

        HAL_Delay(1U);
    }
}
