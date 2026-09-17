#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "motor.h"
#include "Slope.h"
#include "Power_Control.h"

#define K_ROTATE (0.17f+0.17f)
#define WHEEL_R  0.07f
#define LF  0U
#define RF  1U
#define LB  2U
#define RB  3U
#define Vx_max 1.8f
#define Vy_max 1.8f
#define WHEEL_SPEED_MAX 20.0f
#define CHASSIS_TRANSLATION_ACCEL_MPS2 2.0f   //控制功率做的缩放
#define CHASSIS_ROTATION_ACCEL_RADPS2  10.0f  //控制功率做的缩放


typedef struct
{
  Motor_t motor_3508[4];

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
    Slope forward;
    Slope right;
    Slope rotation;
  } command_slope;

  ChassisPowerControl_t power_prediction;

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
void Chassis_ResetControl(void);
void Chassis_Update(void);

#endif
