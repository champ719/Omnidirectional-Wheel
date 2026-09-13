#ifndef __GIMBAL_H
#define __GIMBAL_H
#include "main.h"
#include "motor.h"


#define PITCH_MAX_ANGLE 0.5345f
#define PITCH_MIN_ANGLE -0.3627f
#define YAW_ANGLE_RATE 0.030f
#define PITCH_ANGLE_RATE 0.020f

typedef struct
{
  Motor_t yaw_motor;
  Motor_t pitch_motor;

  struct{
    float yaw_w;
    float pitch_w;
  } gyro;

  struct{
    float fb_yaw_w;
    float target_yaw_w;
    float target_yaw_angle;
    float yaw_angle;
    float zero_angle;
    float machine_yaw_angle;
  } yaw;
  
  struct{
    float fb_pitch_w;
    float target_pitch_angle;
    float target_pitch_w;
    float pitch_angle;
    float zero_angle;
    float k_gravity_comp;
    float gravity_comp_offset;
    float gravity_comp;
  } pitch;

} Gimbal_t;

extern volatile Gimbal_t gimbal;

void Gimbal_Init(void);
void Gimbal_Update(void);

#endif 