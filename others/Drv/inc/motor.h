#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "USER_PID.h"

#define MOTOR_CONTROL_PERIOD_S       0.002f
#define MOTOR_WHEEL_RADIUS_M         0.075f
#define MOTOR_HALF_LENGTH_M          0.300f
#define MOTOR_HALF_WIDTH_M           0.300f
#define MOTOR_MAX_VX_MPS             6.0f
#define MOTOR_MAX_VY_MPS             6.0f
#define MOTOR_MAX_WZ_RADPS           5.0f
#define MOTOR_REMOTE_FULL_SCALE      660.0f
#define MOTOR_REMOTE_DEADBAND        0

#define MOTOR_YAW_TARGET_COUNT       1400.0f
#define MOTOR_YAW_DEADBAND_COUNT     30.0f
#define MOTOR_YAW_D_FILTER_ALPHA     0.8f
#define MOTOR_PITCH_START_COUNT      5500.0f
#define MOTOR_PITCH_MIN_COUNT        5000.0f
#define MOTOR_PITCH_MAX_COUNT        6000.0f
#define MOTOR_PITCH_RATE_COUNT_S     1000.0f
#define MOTOR_ENCODER_RANGE          8192.0f

/* Set to +1.0f or -1.0f after checking each gimbal motor direction. */
#define MOTOR_YAW_DIRECTION          1.0f
#define MOTOR_PITCH_DIRECTION        1.0f

/*
 * Direction calibration is intentionally incomplete. Set each item to
 * +1.0f or -1.0f only after checking the corresponding wheel direction.
 * A zero direction keeps every motor stopped.
 */
#define MOTOR_DIRECTION_FL          (+1.0f)
#define MOTOR_DIRECTION_FR          (-1.0f)
#define MOTOR_DIRECTION_RL          (+1.0f)
#define MOTOR_DIRECTION_RR          (-1.0f)

typedef enum
{
    MOTOR_WHEEL_FL = 0,
    MOTOR_WHEEL_FR,
    MOTOR_WHEEL_RL,
    MOTOR_WHEEL_RR,
    MOTOR_WHEEL_COUNT
} Motor_WheelIndex_t;

typedef enum
{
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_REMOTE_OFFLINE,
    MOTOR_STATE_EMERGENCY_STOP,
    MOTOR_STATE_DIRECTION_UNCALIBRATED
} Motor_State_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float output_max;
    float integral_max;
} Motor_PID_Parameters_t;

typedef struct
{
    volatile float yaw_target;
    volatile float yaw_feedback;
    volatile float yaw_error;
    volatile float yaw_output;
    volatile float pitch_target;
    volatile float pitch_feedback;
    volatile float pitch_error;
    volatile float pitch_output;
    volatile uint8_t initialized;
    USER_PID_t yaw_pid;
    USER_PID_t pitch_pid;
} Motor_GimbalControl_t;

typedef struct
{
    volatile Motor_State_t state;
    volatile uint32_t update_count;
    volatile float vx_target;
    volatile float vy_target;
    volatile float wz_target;
    volatile float wheel_target[MOTOR_WHEEL_COUNT];
    volatile float wheel_feedback[MOTOR_WHEEL_COUNT];
    volatile float wheel_output[MOTOR_WHEEL_COUNT];
    USER_PID_t speed_pid[MOTOR_WHEEL_COUNT];
    Motor_GimbalControl_t gimbal;
} Motor_Control_t;

extern Motor_Control_t motor_control;
extern volatile Motor_PID_Parameters_t motor_speed_pid_parameters;
extern volatile Motor_PID_Parameters_t motor_yaw_pid_parameters;
extern volatile Motor_PID_Parameters_t motor_pitch_pid_parameters;

extern volatile uint8_t motor_update_flag;

void Motor_Control_Init(void);
void Motor_Control_Update_2ms(void);
void Motor_Control_StopAll(Motor_State_t reason);
uint8_t Motor_Control_DirectionsCalibrated(void);

#endif
