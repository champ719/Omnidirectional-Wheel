#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

typedef enum
{
    ERROR_RESULT_NONE = 0,
    ERROR_RESULT_STOPPED,
    ERROR_RESULT_REMOTE_OFFLINE,
    ERROR_RESULT_EMERGENCY_STOP,
    ERROR_RESULT_ARMING,
    ERROR_RESULT_IMU_NOT_READY,
    ERROR_RESULT_MOTOR_FEEDBACK_TIMEOUT,
    ERROR_RESULT_CAN_FAULT
} Error_Result_t;

/* 比 Error_Result_t 更细的调试阶段，用于定位解锁流程卡点。 */
typedef enum
{
    ERROR_STAGE_BOOT = 0,
    ERROR_STAGE_REMOTE_OFFLINE,
    ERROR_STAGE_EMERGENCY_STOP,
    ERROR_STAGE_SWITCH_STOPPED,
    ERROR_STAGE_IMU_NOT_READY,
    ERROR_STAGE_WAIT_CHASSIS_INIT,
    ERROR_STAGE_WAIT_GIMBAL_INIT,
    ERROR_STAGE_CAN_FAULT,
    ERROR_STAGE_MOTOR_FEEDBACK_TIMEOUT,
    ERROR_STAGE_WAIT_S2_MID,
    ERROR_STAGE_WAIT_STICK_CENTER,
    ERROR_STAGE_WAIT_REMOTE_FRAMES,
    ERROR_STAGE_WAIT_IMU_UPDATE,
    ERROR_STAGE_READY
} Error_DebugStage_t;

typedef struct
{
    Error_Result_t result;
    Error_DebugStage_t stage;
    uint8_t controls_enabled;
    uint8_t remote_online;
    uint8_t remote_s1;
    uint8_t remote_s2;
    uint8_t controls_centered;
    uint8_t valid_frames_ready;
    uint8_t imu_ready;
    uint8_t chassis_initialized;
    uint8_t gimbal_initialized;
    uint8_t can_healthy;
    uint8_t motor_online_mask;
    uint32_t imu_sequence;
    uint32_t arming_imu_sequence;
} Error_Debug_t;

/* 可直接加入 Watch/Live Expressions 的全局调试符号。 */
extern volatile Error_Result_t error_result_debug;
extern volatile Error_Debug_t error_debug;

void Error_Init(void);
void Error_MonitorUpdate(void);
Error_Result_t Error_GetResult(void);
void Error_TriggerEmergencyStop(void);
Error_Result_t Error_Update(uint8_t remote_online, uint8_t remote_switch, uint8_t imu_ready);

#endif
