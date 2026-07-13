#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "USER_PID.h"

#define MOTOR_CONTROL_PERIOD_S       0.002f
#define MOTOR_WHEEL_RADIUS_M         0.075f
#define MOTOR_MAX_VX_MPS             3.0f
#define MOTOR_MAX_VY_MPS             3.0f
#define MOTOR_REMOTE_FULL_SCALE      660.0f
#define MOTOR_REMOTE_DEADBAND        20

/* Low-speed, wheels-off-ground commissioning values. */
#define MOTOR_SPEED_PID_KP           0.50f
#define MOTOR_SPEED_PID_KI           0.80f
#define MOTOR_SPEED_PID_KD           0.00f
#define MOTOR_SPEED_PID_OUT_MAX      20.0f
#define MOTOR_SPEED_PID_I_MAX        10.0f

/*
 * Direction calibration is intentionally incomplete. Set each item to
 * +1.0f or -1.0f only after checking the corresponding wheel direction.
 * A zero direction keeps every motor stopped.
 */
#define MOTOR_DIRECTION_FL           0.0f
#define MOTOR_DIRECTION_FR           0.0f
#define MOTOR_DIRECTION_RL           0.0f
#define MOTOR_DIRECTION_RR           0.0f

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
    volatile Motor_State_t state;
    volatile uint32_t update_count;
    volatile float vx_target;
    volatile float vy_target;
    volatile float wheel_target[MOTOR_WHEEL_COUNT];
    volatile float wheel_feedback[MOTOR_WHEEL_COUNT];
    volatile float wheel_output[MOTOR_WHEEL_COUNT];
    USER_PID_t speed_pid[MOTOR_WHEEL_COUNT];
} Motor_Control_t;

extern Motor_Control_t motor_control;

void Motor_Control_Init(void);
void Motor_Control_Update_2ms(void);
void Motor_Control_StopAll(Motor_State_t reason);
uint8_t Motor_Control_DirectionsCalibrated(void);

#endif
