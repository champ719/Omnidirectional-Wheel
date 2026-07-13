#include "USER_PID.h"

static float USER_PID_Limit(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void USER_PID_Init(USER_PID_t *pid,
                   float kp,
                   float ki,
                   float kd,
                   float output_max,
                   float integral_max)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_max = output_max;
    pid->integral_max = integral_max;
    USER_PID_Reset(pid);
}

void USER_PID_Reset(USER_PID_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
}

float USER_PID_Calculate(USER_PID_t *pid,
                         float target,
                         float feedback,
                         float dt)
{
    float error;
    float derivative;

    if ((pid == NULL) || (dt <= 0.0f)) {
        return 0.0f;
    }

    error = target - feedback;
    pid->integral += error * dt;
    pid->integral = USER_PID_Limit(pid->integral, pid->integral_max);
    derivative = (error - pid->last_error) / dt;

    pid->output = pid->kp * error
                + pid->ki * pid->integral
                + pid->kd * derivative;
    pid->output = USER_PID_Limit(pid->output, pid->output_max);
    pid->last_error = error;

    return pid->output;
}
