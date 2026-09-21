#include "Error.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Buzzer.h"
#include "chassis.h"
#include "gimbal.h"
#include "USER_CAN.h"
#include "motor.h"
#include "imu_temp_ctrl.h"
#include "Remote.h"

static volatile Error_Result_t error_result = ERROR_RESULT_REMOTE_OFFLINE;
volatile Error_Result_t error_result_debug = ERROR_RESULT_REMOTE_OFFLINE;
volatile Error_Debug_t error_debug = {
    .result = ERROR_RESULT_REMOTE_OFFLINE,
    .stage = ERROR_STAGE_BOOT
};
static volatile uint8_t emergency_stop_triggered;
static volatile uint8_t controls_enabled;
static uint32_t arming_imu_sequence;

extern osThreadId ErrorTaskHandle;

#define ERROR_MOTOR_ONLINE_LF   (1U << 0)
#define ERROR_MOTOR_ONLINE_RF   (1U << 1)
#define ERROR_MOTOR_ONLINE_LB   (1U << 2)
#define ERROR_MOTOR_ONLINE_RB   (1U << 3)
#define ERROR_MOTOR_ONLINE_YAW  (1U << 4)
#define ERROR_MOTOR_ONLINE_PITCH (1U << 5)

static void Error_SetState(Error_Result_t result, Error_DebugStage_t stage)
{
    error_result = result;
    error_result_debug = result;
    error_debug.result = result;
    error_debug.stage = stage;
}

static uint8_t Error_GetMotorOnlineMask(void)
{
    uint8_t mask = 0U;

    if (Motor_IsOnline(&chassis.motor_3508[LF]) != 0U) {
        mask |= ERROR_MOTOR_ONLINE_LF;
    }
    if (Motor_IsOnline(&chassis.motor_3508[RF]) != 0U) {
        mask |= ERROR_MOTOR_ONLINE_RF;
    }
    if (Motor_IsOnline(&chassis.motor_3508[LB]) != 0U) {
        mask |= ERROR_MOTOR_ONLINE_LB;
    }
    if (Motor_IsOnline(&chassis.motor_3508[RB]) != 0U) {
        mask |= ERROR_MOTOR_ONLINE_RB;
    }
    if (Motor_IsOnline(&gimbal.yaw_motor) != 0U) {
        mask |= ERROR_MOTOR_ONLINE_YAW;
    }
    if (Motor_IsOnline(&gimbal.pitch_motor) != 0U) {
        mask |= ERROR_MOTOR_ONLINE_PITCH;
    }
    return mask;
}

static void Error_PrepareArming(void)
{
    controls_enabled = 0U;
    arming_imu_sequence = IMU_Attitude_GetUpdateSequence();
    Remote_ResetValidFrameCount();
    error_debug.controls_enabled = 0U;
    error_debug.valid_frames_ready = 0U;
    error_debug.arming_imu_sequence = arming_imu_sequence;
}

/**
 * @brief 初始化故障监控状态。
 * @note 上电时默认认为遥控器离线，急停任务保持未触发状态。
 */
void Error_Init(void)
{
    Error_SetState(ERROR_RESULT_REMOTE_OFFLINE,
                   ERROR_STAGE_REMOTE_OFFLINE);
    emergency_stop_triggered = 0U;
    Error_PrepareArming();
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
    uint8_t remote_online;
    uint8_t imu_ready;

    Remote_GetSnapshot(&remote);
    remote_online = Remote_IsOnline();
    imu_ready = IMU_Attitude_IsReady();

    error_debug.remote_online = remote_online;
    error_debug.remote_s1 = remote.rc.s1;
    error_debug.remote_s2 = remote.rc.s2;
    error_debug.controls_centered = Remote_ControlsAreCentered(&remote);
    error_debug.valid_frames_ready = Remote_HasFiveValidFrames();
    error_debug.imu_ready = imu_ready;
    error_debug.chassis_initialized = Chassis_IsInitialized();
    error_debug.gimbal_initialized = Gimbal_IsInitialized();
    error_debug.can_healthy = CAN_IsHealthy();
    error_debug.motor_online_mask = Error_GetMotorOnlineMask();
    error_debug.imu_sequence = IMU_Attitude_GetUpdateSequence();
    error_debug.arming_imu_sequence = arming_imu_sequence;
    error_debug.controls_enabled = controls_enabled;

    result = Error_Update(
        remote_online,
        remote.rc.s1,
        imu_ready);

    if (result != ERROR_RESULT_NONE) {
        Error_PrepareArming();
        if (result == ERROR_RESULT_REMOTE_OFFLINE) {
            Error_SetState(result, ERROR_STAGE_REMOTE_OFFLINE);
        } else if (result == ERROR_RESULT_EMERGENCY_STOP) {
            Error_SetState(result, ERROR_STAGE_EMERGENCY_STOP);
        } else if (result == ERROR_RESULT_STOPPED) {
            Error_SetState(result, ERROR_STAGE_SWITCH_STOPPED);
        } else {
            Error_SetState(result, ERROR_STAGE_IMU_NOT_READY);
        }
        return;
    }

    if (error_debug.chassis_initialized == 0U) {
        Error_PrepareArming();
        Error_SetState(ERROR_RESULT_ARMING,
                       ERROR_STAGE_WAIT_CHASSIS_INIT);
        return;
    }

    if (error_debug.gimbal_initialized == 0U) {
        Error_PrepareArming();
        Error_SetState(ERROR_RESULT_ARMING,
                       ERROR_STAGE_WAIT_GIMBAL_INIT);
        return;
    }

    if (error_debug.can_healthy == 0U) {
        Error_PrepareArming();
        Error_SetState(ERROR_RESULT_CAN_FAULT,
                       ERROR_STAGE_CAN_FAULT);
        return;
    }

    if (Motor_FeedbackHealthy() == 0U) {
        Error_PrepareArming();
        Error_SetState(ERROR_RESULT_MOTOR_FEEDBACK_TIMEOUT,
                       ERROR_STAGE_MOTOR_FEEDBACK_TIMEOUT);
        return;
    }

    if (controls_enabled == 0U) {
        if (remote.rc.s2 != RC_SW_MID) {
            Error_PrepareArming();
            Error_SetState(ERROR_RESULT_ARMING,
                           ERROR_STAGE_WAIT_S2_MID);
            return;
        }

        if (error_debug.controls_centered == 0U) {
            Error_PrepareArming();
            Error_SetState(ERROR_RESULT_ARMING,
                           ERROR_STAGE_WAIT_STICK_CENTER);
            return;
        }

        if (Remote_HasFiveValidFrames() == 0U) {
            Error_SetState(ERROR_RESULT_ARMING,
                           ERROR_STAGE_WAIT_REMOTE_FRAMES);
            return;
        }

        if (IMU_Attitude_GetUpdateSequence() == arming_imu_sequence) {
            Error_SetState(ERROR_RESULT_ARMING,
                           ERROR_STAGE_WAIT_IMU_UPDATE);
            return;
        }

        controls_enabled = 1U;
        error_debug.controls_enabled = 1U;
    }

    Error_SetState(ERROR_RESULT_NONE, ERROR_STAGE_READY);
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

    Error_PrepareArming();
    Error_SetState(ERROR_RESULT_EMERGENCY_STOP,
                   ERROR_STAGE_EMERGENCY_STOP);
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
 *       CAN 并发送零电流。S1 回到中档或上档时只清控制器历史量，保留
 *       IMU 连续角和云台目标角，然后重新挂起以等待下一次急停。
 */
void OS_ErrorCallback(void const *argument)
{
    RC_Ctrl_t remote;
    uint32_t buzzer_update_tick;

    (void)argument;
    for (;;)
    {
        /* 上电或上一次急停解除后挂起，由 Error_TriggerEmergencyStop 唤醒。 */
        (void)osThreadSuspend(ErrorTaskHandle);

        Error_SetState(ERROR_RESULT_EMERGENCY_STOP,
                       ERROR_STAGE_EMERGENCY_STOP);
        Error_PrepareArming();
        Chassis_ResetControl();
        gimbal.yaw_motor.give_current = 0.0f;
        gimbal.pitch_motor.give_current = 0.0f;
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
            error_debug.remote_online = Remote_IsOnline();
            error_debug.remote_s1 = remote.rc.s1;
            error_debug.remote_s2 = remote.rc.s2;
            if ((remote.rc.s1 == RC_SW_MID) ||
                (remote.rc.s1 == RC_SW_UP)) {
                Chassis_ResetControl();

                PID_Clear(&gimbal.yaw_motor.pid_position);
                PID_Clear(&gimbal.yaw_motor.pid_speed);
                PID_Clear(&gimbal.pitch_motor.pid_position);
                PID_Clear(&gimbal.pitch_motor.pid_speed);

                gimbal.yaw.target_yaw_w = 0.0f;
                gimbal.pitch.target_pitch_w = 0.0f;
                gimbal.yaw_motor.give_current = 0.0f;
                gimbal.pitch_motor.give_current = 0.0f;
                Gimbal_HoldCurrentYawAfterEmergencyStop();

                Error_PrepareArming();
                emergency_stop_triggered = 0U;
                Error_MonitorUpdate();

                break;
            }

        
            HAL_Delay(1U);
        }
    }
}
