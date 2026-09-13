#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct
{
  float kp;
  float ki;
  float kd;
  float kf;
  float last_target;
  float integral;
  float last_err;
  float out_limit;
  float integral_limit;
  float out_put;
} PID_t;

void PID_Set(volatile PID_t *pid, float kp, float ki, float kd, float kf,
             float out_limit, float integral_limit);
void PID_Position_Calc(volatile PID_t *pid, float target, float feedback);
void PID_Speed_Calc(volatile PID_t *pid, float target, float feedback);
void PID_Clear(volatile PID_t *pid);

#endif
