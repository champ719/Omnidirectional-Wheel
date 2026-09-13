#include "PID.h"

void PID_Set(volatile PID_t *pid, float kp, float ki, float kd, float kf,
             float out_limit, float integral_limit)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->kf = kf;
  pid->last_target = 0.0f;
  pid->integral = 0.0f;
  pid->last_err = 0.0f;
  pid->out_limit = out_limit;
  pid->integral_limit = integral_limit;
  pid->out_put = 0.0f;
}

void PID_Position_Calc(volatile PID_t *pid, float target, float feedback)
{
  float err = target - feedback;
  float out;

  if (err > 3.14159265f) {
    err -= 6.28318531f;
  } else if (err < -3.14159265f) {
    err += 6.28318531f;
  }

  pid->integral += err;
  if (pid->integral > pid->integral_limit) {
    pid->integral = pid->integral_limit;
  } else if (pid->integral < -pid->integral_limit) {
    pid->integral = -pid->integral_limit;
  }

  out = pid->kp * err + pid->ki * pid->integral +
        pid->kd * (err - pid->last_err) +
        pid->kf * (target - pid->last_target);
  pid->last_err = err;
  pid->last_target = target;

  if (out > pid->out_limit) {
    out = pid->out_limit;
  } else if (out < -pid->out_limit) {
    out = -pid->out_limit;
  }

  pid->out_put = out;
}

void PID_Speed_Calc(volatile PID_t *pid, float target, float feedback)
{
  float err = target - feedback;
  float out;

  pid->integral += err;
  if (pid->integral > pid->integral_limit) {
    pid->integral = pid->integral_limit;
  } else if (pid->integral < -pid->integral_limit) {
    pid->integral = -pid->integral_limit;
  }

  out = pid->kp * err + pid->ki * pid->integral +
        pid->kd * (err - pid->last_err) +
        pid->kf * (target - pid->last_target);
  pid->last_err = err;
  pid->last_target = target;

  if (out > pid->out_limit) {
    out = pid->out_limit;
  } else if (out < -pid->out_limit) {
    out = -pid->out_limit;
  }

  pid->out_put = out;
}

void PID_Clear(volatile PID_t *pid)
{
  pid->last_target = 0.0f;
  pid->integral = 0.0f;
  pid->last_err = 0.0f;
  pid->out_put = 0.0f;
}
