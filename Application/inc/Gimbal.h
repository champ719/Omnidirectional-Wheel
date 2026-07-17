#ifndef GIMBAL_H
#define GIMBAL_H

#include "USER_PID.h"
#include <stdint.h>

#define GIMBAL_YAW_TARGET_COUNT              1400.0f  /* 云台正前方对应的 yaw 编码器计数。 */
#define GIMBAL_COMMAND_MAX                   163.84f   /* 云台电机控制输出的绝对值上限。 */
#define GIMBAL_YAW_SPEED_MAX_RADPS           10.0f    /* yaw 目标角速度上限，单位：rad/s。 */
#define GIMBAL_PITCH_SPEED_MAX_RADPS         3.0f     /* pitch 目标角速度上限，单位：rad/s。 */
#define GIMBAL_PITCH_START_COUNT             5380.0f  /* pitch 上电初始目标编码器计数。 */
#define GIMBAL_PITCH_MIN_COUNT               4900.0f  /* pitch 机械软限位最小编码器计数。 */
#define GIMBAL_PITCH_MAX_COUNT               6076.0f  /* pitch 机械软限位最大编码器计数。 */
#define GIMBAL_PITCH_RATE_COUNT_S            3000.0f  /* 满量程遥控输入对应的 pitch 计数变化率，单位：count/s。 */
#define GIMBAL_ENCODER_RANGE                 8192.0f  /* 云台电机编码器一圈的总计数。 */
#define GIMBAL_ENCODER_COUNT_PER_RAD         (GIMBAL_ENCODER_RANGE / 6.28318530718f) /* 弧度到编码器计数的换算系数。 */

typedef struct
{
    volatile float yaw_target;
    volatile float yaw_feedback;
    volatile float yaw_error;
    volatile float yaw_speed_target;
    volatile float yaw_speed_feedback;
    volatile float yaw_output;
    volatile float pitch_target;
    volatile float pitch_feedback;
    volatile float pitch_error;
    volatile float pitch_speed_target;
    volatile float pitch_speed_feedback;
    volatile float pitch_output;
    volatile float inertial_yaw_target;
    volatile float inertial_yaw_feedback;
    volatile float inertial_yaw_error;
    volatile uint8_t inertial_hold_active;
    volatile uint8_t initialized;
    USER_PID_t yaw_position_pid;
    USER_PID_t yaw_speed_pid;
    USER_PID_t inertial_yaw_position_pid;
    USER_PID_t pitch_position_pid;
    USER_PID_t pitch_speed_pid;
} Gimbal_Control_t;

extern Gimbal_Control_t gimbal_control;
void Gimbal_Ctrl_Init(void);
void Gimbal_Ctrl_Stop(void);
void Gimbal_Ctrl_UpdateNormal(float remote_pitch,
                              float yaw_feedback,
                              float yaw_speed_feedback,
                              float pitch_feedback,
                              float pitch_speed_feedback,
                              float dt);
void Gimbal_Ctrl_UpdateRotate(float remote_yaw,
                              float remote_pitch,
                              float yaw_feedback,
                              float yaw_speed_feedback,
                              float pitch_feedback,
                              float pitch_speed_feedback,
                              float imu_yaw_continuous,
                              float dt);
float Gimbal_Ctrl_CircularError(float target, float feedback);

#endif
