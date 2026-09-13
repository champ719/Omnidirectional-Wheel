#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>

#define ROBOT_CONTROL_PERIOD_S 0.01f

typedef enum
{
    ROBOT_CONTROL_SOURCE_NONE = 0,
    ROBOT_CONTROL_SOURCE_JOYSTICK,
    ROBOT_CONTROL_SOURCE_KEYBOARD_MOUSE
} Robot_ControlSource_t;

typedef enum
{
    ROBOT_CHASSIS_GYRO_CCW = -1,
    ROBOT_CHASSIS_FOLLOW = 0,
    ROBOT_CHASSIS_GYRO_CW = 1
} Robot_ChassisMode_t;

typedef enum
{
    ROBOT_STATE_STOPPED = 0,
    ROBOT_STATE_RUNNING,
    ROBOT_STATE_REMOTE_OFFLINE,
    ROBOT_STATE_EMERGENCY_STOP,
    ROBOT_STATE_DIRECTION_UNCALIBRATED,
    ROBOT_STATE_SMALL_GYRO,
    ROBOT_STATE_IMU_NOT_READY,
    ROBOT_STATE_MOTOR_OFFLINE,
    ROBOT_STATE_CAN_FAULT
} Robot_ControlState_t;

typedef struct
{
    Robot_ControlSource_t source;
    Robot_ChassisMode_t chassis_mode;
    float forward;
    float right;
    float gimbal_yaw_delta_rad;
    float gimbal_pitch_delta_rad;
    float speed_scale;
    uint32_t input_tick;
    uint32_t input_sequence;
} Robot_Command_t;

typedef struct
{
    volatile Robot_ControlState_t state;
    Robot_Command_t command;
} Robot_Control_t;

extern Robot_Control_t robot_control;

void Robot_Control_Init(void);
void Robot_Control_Update(void);
void OS_MotorCallback(void const *argument);

#endif
