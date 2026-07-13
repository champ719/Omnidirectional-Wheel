#ifndef USER_PID_H
#define USER_PID_H

#include "main.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float output;
    float output_max;
    float integral_max;
} USER_PID_t;

void USER_PID_Init(USER_PID_t *pid,
                   float kp,
                   float ki,
                   float kd,
                   float output_max,
                   float integral_max);
void USER_PID_Reset(USER_PID_t *pid);
float USER_PID_Calculate(USER_PID_t *pid,
                         float target,
                         float feedback,
                         float dt);

#endif
