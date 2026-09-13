#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "motor.h"

#define K_ROTATE (0.17f+0.17f)
#define WHEEL_R  0.07f
#define LF  0U
#define RF  1U
#define LB  2U
#define RB  3U
#define Vx_max 1.8f
#define Vy_max 1.8f
#define WHEEL_SPEED_MAX 20.0f

#define TARGET_SPEED_STEP 0.30f


typedef struct
{
  Motor_t motor_3508[4];

  float chassis_w_smooth;

  struct
  {
    float Vx;
    float Vy;
    float w;
  } target_speed;

  struct
  {
    float Vx;
    float Vy;
    float w;
  } fb_speed;

  struct
  {
    float k1[4];
    float k2[4];
    float a[4];
    float torque_scale[4];
    float power_max;
    float model_power[4];
    float total_power;
    float pre_target_speed[4];
  } power_prediction;

  struct 
  {
    float current;
    float voltage;
    float power;
  }power_fb;
  

  PID_t follow_speed;
} Chassis_t;

extern volatile Chassis_t chassis;

void Chassis_Init(void);
void Chassis_Update(void);

#endif
