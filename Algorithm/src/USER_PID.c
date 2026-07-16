#include "USER_PID.h"
#include <math.h>

static float USER_PID_Limit(float value, float limit)
{
    if (limit < 0.0f) {
        limit = -limit;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float USER_PID_ApplyDeadZone(const USER_PID_t *pid, float error)
{
    if (pid->dead_zone <= 0.0f) {
        return error;
    }
    if (fabsf(error) <= pid->dead_zone) {
        return 0.0f;
    }

    return (error > 0.0f) ?
           (error - pid->dead_zone) :
           (error + pid->dead_zone);
}

void USER_PID_Init(USER_PID_t *pid,
                   float kp,
                   float ki,
                   float kd,
                   float integral_max,
                   float integral_step,
                   float integral_dead_zone,
                   float dead_zone)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_max = fabsf(integral_max);
    pid->integral_step = fabsf(integral_step);
    pid->integral_dead_zone = fabsf(integral_dead_zone);
    pid->dead_zone = fabsf(dead_zone);
    pid->derivative_filter_alpha = 1.0f;
    USER_PID_Reset(pid);
}

void USER_PID_Reset(USER_PID_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->error = 0.0f;
}

void USER_PID_SetDerivativeFilter(USER_PID_t *pid, float alpha)
{
    if (pid == NULL) {
        return;
    }
    pid->derivative_filter_alpha = alpha;
    if (pid->derivative_filter_alpha < 0.0f) {
        pid->derivative_filter_alpha = 0.0f;
    } else if (pid->derivative_filter_alpha > 1.0f) {
        pid->derivative_filter_alpha = 1.0f;
    }
}

void USER_PID_ClearIntegral(USER_PID_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
}

void USER_PID_SetError(USER_PID_t *pid, float target, float feedback)
{
    if (pid == NULL) {
        return;
    }
    pid->error = target - feedback;
}

float USER_PID_NormalizeAngle(float angle)
{
    angle = fmodf(angle, 2.0f * USER_PID_PI);
    if (angle > USER_PID_PI) {
        angle -= 2.0f * USER_PID_PI;
    } else if (angle < -USER_PID_PI) {
        angle += 2.0f * USER_PID_PI;
    }
    return angle;
}

float USER_PID_ShortestAngleDelta(float target_angle,
                                  float feedback_angle)
{
    return USER_PID_NormalizeAngle(
        USER_PID_NormalizeAngle(target_angle) -
        USER_PID_NormalizeAngle(feedback_angle));
}

void USER_PID_SetAngleError(USER_PID_t *pid,
                            float target_angle,
                            float feedback_angle)
{
    if (pid == NULL) {
        return;
    }
    pid->error = USER_PID_ShortestAngleDelta(target_angle,
                                             feedback_angle);
}

float USER_PID_CalculateFromError(USER_PID_t *pid, float dt)
{
    float error;
    float derivative;
    float integral_delta;

    if ((pid == NULL) || (dt <= 0.0f)) {
        return 0.0f;
    }

    error = USER_PID_ApplyDeadZone(pid, pid->error);

    /* Integrate only near the target when a zone is configured. */
    if ((pid->integral_dead_zone <= 0.0f) ||
        (fabsf(error) <= pid->integral_dead_zone)) {
        integral_delta = pid->ki * error * dt;
        if ((pid->integral_step > 0.0f) &&
            (integral_delta > pid->integral_step)) {
            integral_delta = pid->integral_step;
        } else if ((pid->integral_step > 0.0f) &&
                   (integral_delta < -pid->integral_step)) {
            integral_delta = -pid->integral_step;
        }
        pid->integral += integral_delta;
        pid->integral = USER_PID_Limit(pid->integral,
                                       pid->integral_max);
    }

    derivative = (error - pid->last_error) / dt;
    pid->derivative += pid->derivative_filter_alpha *
                       (derivative - pid->derivative);
    pid->output = pid->kp * error
                + pid->integral
                + pid->kd * pid->derivative;
    pid->error = error;
    pid->last_error = error;

    return pid->output;
}

float USER_PID_Calculate(USER_PID_t *pid,
                         float target,
                         float feedback,
                         float dt)
{
    USER_PID_SetError(pid, target, feedback);
    return USER_PID_CalculateFromError(pid, dt);
}
