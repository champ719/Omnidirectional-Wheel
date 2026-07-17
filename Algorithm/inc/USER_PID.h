#ifndef USER_PID_H
#define USER_PID_H

#include <stddef.h>
#include <stdint.h>

#define USER_PID_PI 3.14159265358979323846f

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kf;
    float target;
    float integral;
    float last_error;
    float derivative;
    float derivative_filter_alpha;
    float output;
    float integral_max;
    float integral_step;
    float integral_dead_zone;
    float dead_zone;
    float error;
} USER_PID_t;

void USER_PID_Init(USER_PID_t *pid,
                   float kp,
                   float ki,
                   float kd,
                   float kf,
                   float integral_max,
                   float integral_step,
                   float integral_dead_zone,
                   float dead_zone);
void USER_PID_Reset(USER_PID_t *pid);
void USER_PID_SetDerivativeFilter(USER_PID_t *pid, float alpha);
void USER_PID_ClearIntegral(USER_PID_t *pid);
void USER_PID_SetError(USER_PID_t *pid, float target, float feedback);
void USER_PID_SetAngleError(USER_PID_t *pid,
                            float target_angle,
                            float feedback_angle);
float USER_PID_NormalizeAngle(float angle);
float USER_PID_ShortestAngleDelta(float target_angle,
                                  float feedback_angle);
float USER_PID_CalculateFromError(USER_PID_t *pid, float dt);
float USER_PID_Calculate(USER_PID_t *pid,
                         float target,
                         float feedback,
                         float dt);

#endif
