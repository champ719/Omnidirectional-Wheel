#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>

#define ROBOT_CONTROL_PERIOD_S 0.002f

typedef enum
{
    ROBOT_STATE_STOPPED = 0,
    ROBOT_STATE_RUNNING,
    ROBOT_STATE_REMOTE_OFFLINE,
    ROBOT_STATE_EMERGENCY_STOP,
    ROBOT_STATE_DIRECTION_UNCALIBRATED,
    ROBOT_STATE_SMALL_GYRO,
    ROBOT_STATE_IMU_NOT_READY,
    ROBOT_STATE_GYRO_EXIT_ALIGN
} Robot_Control_State_t;

typedef struct
{
    volatile Robot_Control_State_t state;
} Robot_Control_t;

extern Robot_Control_t robot_control;

void Robot_Control_Init(void);
void Robot_Control_Update_2ms(void);

#endif
