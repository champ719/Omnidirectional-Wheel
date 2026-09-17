#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

typedef enum
{
    ERROR_RESULT_NONE = 0,
    ERROR_RESULT_STOPPED,
    ERROR_RESULT_REMOTE_OFFLINE,
    ERROR_RESULT_EMERGENCY_STOP,
    ERROR_RESULT_IMU_NOT_READY,
    ERROR_RESULT_MOTOR_FEEDBACK_TIMEOUT,
    ERROR_RESULT_CAN_FAULT
} Error_Result_t;

void Error_Init(void);
void Error_MonitorUpdate(void);
Error_Result_t Error_GetResult(void);
void Error_TriggerEmergencyStop(void);
Error_Result_t Error_Update(uint8_t remote_online,
                            uint8_t remote_switch,
                            uint8_t imu_ready);

#endif
