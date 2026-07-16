#ifndef GIMBAL_H
#define GIMBAL_H

#include "USER_PID.h"
#include <stdint.h>

#define GIMBAL_YAW_TARGET_COUNT              1400.0f
#define GIMBAL_YAW_DEADBAND_COUNT            30.0f
#define GIMBAL_INERTIAL_YAW_DEADBAND_RAD     0.0035f
#define GIMBAL_COMMAND_MAX                   163.84f
#define GIMBAL_YAW_SPEED_MAX_RADPS           4.0f
#define GIMBAL_PITCH_SPEED_MAX_RADPS         3.0f
#define GIMBAL_PITCH_START_COUNT             5500.0f
#define GIMBAL_PITCH_MIN_COUNT               5000.0f
#define GIMBAL_PITCH_MAX_COUNT               6000.0f
#define GIMBAL_PITCH_DEADBAND_COUNT          5.0f
#define GIMBAL_PITCH_RATE_COUNT_S            1000.0f
#define GIMBAL_ENCODER_RANGE                 8192.0f
#define GIMBAL_ENCODER_COUNT_PER_RAD         (GIMBAL_ENCODER_RANGE / 6.28318530718f)
#define GIMBAL_IMU_YAW_TO_ENCODER_DIRECTION  (+1.0f)
#define GIMBAL_YAW_DIRECTION                 (+1.0f)
#define GIMBAL_PITCH_DIRECTION               (+1.0f)

/* Position loop: position error -> target speed. */
#define GIMBAL_YAW_POSITION_KP               0.010f
#define GIMBAL_YAW_POSITION_KI               0.0f
#define GIMBAL_YAW_POSITION_KD               0.0f
#define GIMBAL_INERTIAL_YAW_POSITION_KP      6.0f
#define GIMBAL_INERTIAL_YAW_POSITION_KI      0.0f
#define GIMBAL_INERTIAL_YAW_POSITION_KD      0.0f
#define GIMBAL_PITCH_POSITION_KP             0.010f
#define GIMBAL_PITCH_POSITION_KI             0.0f
#define GIMBAL_PITCH_POSITION_KD             0.0f

/* Speed loop: speed error -> motor current command. */
#define GIMBAL_YAW_SPEED_KP                  10.0f
#define GIMBAL_YAW_SPEED_KI                  0.0f
#define GIMBAL_YAW_SPEED_KD                  0.0f
#define GIMBAL_PITCH_SPEED_KP                10.0f
#define GIMBAL_PITCH_SPEED_KI                0.0f
#define GIMBAL_PITCH_SPEED_KD                0.0f

#ifndef GIMBAL_IMU_MOUNTED_ON_GIMBAL
#define GIMBAL_IMU_MOUNTED_ON_GIMBAL        1U
#endif

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
    volatile float inertial_encoder_reference;
    volatile uint8_t inertial_hold_active;
    volatile uint8_t initialized;
    USER_PID_t yaw_position_pid;
    USER_PID_t yaw_speed_pid;
    USER_PID_t inertial_yaw_position_pid;
    USER_PID_t pitch_position_pid;
    USER_PID_t pitch_speed_pid;
} Gimbal_Control_t;

extern Gimbal_Control_t gimbal_control;
extern volatile float motor_inertial_yaw_feedforward_gain;

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
                              uint8_t yaw_command_enabled,
                              float yaw_feedback,
                              float yaw_speed_feedback,
                              float pitch_feedback,
                              float pitch_speed_feedback,
                              float imu_yaw_continuous,
                              float chassis_wz_target,
                              float dt);
float Gimbal_Ctrl_CircularError(float target, float feedback);

#endif
