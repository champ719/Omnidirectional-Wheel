#include "gimbal.h"
#include "motor.h"
#include "PID.h"
#include "Robot_Control.h"
#include "chassis.h"
#include "imu_attitude.h"
#include "math.h"



volatile Gimbal_t gimbal;
float test ;


static float wrap_pi(float a)
{
  while (a > 3.14159265f)  a -= 6.28318531f;
  while (a < -3.14159265f) a += 6.28318531f;
  return a;
}

void Gimbal_Init(void)
{
  Motor_Init(&gimbal.yaw_motor, 0x1FF, TYPE_6020,
    0.3f, 0.12f, 0.05f, 0.18f, 1.6f, 0.8f,
    22.0f, 0.0f, 35.0f, 0.0f,5.0f, 4.0f);
  
  Motor_Init(&gimbal.pitch_motor, 0x1FF, TYPE_6020,
    0.1f, 0.02f, 0.05f, 0.25f, 1.6f, 0.8f,
    15.0f, 0.0f, 5.0f, 0.0f, 3.0f, 0.5f);

  gimbal.yaw.zero_angle = 1.04077f;
  gimbal.pitch.zero_angle = 5.1847f;
  gimbal.pitch.target_pitch_angle = 0.0f;
  gimbal.pitch.target_pitch_w = 0.0f;
  gimbal.pitch.k_gravity_comp = 0.32f;
  gimbal.pitch.gravity_comp_offset = 0.55059f;
  gimbal.pitch.gravity_comp = 0.0f;
}

void Yaw_Speed_Calc(void)
{
  //gimbal.yaw.target_yaw_w = dbus_info.target.W_yaw_info * YAW_MAX_W;
  float yaw_angle_change = robot_control.command.gimbal_yaw_delta_rad;

  if(yaw_angle_change > 0.01f || yaw_angle_change < -0.01f)
  gimbal.yaw.target_yaw_angle = wrap_pi(gimbal.yaw.target_yaw_angle + yaw_angle_change);
  
  PID_Position_Calc(&gimbal.yaw_motor.pid_position, gimbal.yaw.target_yaw_angle, gimbal.yaw.yaw_angle);
  gimbal.yaw.target_yaw_w = gimbal.yaw_motor.pid_position.out_put;
  PID_Speed_Calc(&gimbal.yaw_motor.pid_speed, gimbal.yaw_motor.pid_position.out_put, gimbal.yaw.fb_yaw_w);

  gimbal.yaw_motor.give_current = gimbal.yaw_motor.pid_speed.out_put ;
}

void Pitch_Speed_Calc(void)
{
  float angle_change = robot_control.command.gimbal_pitch_delta_rad;

  gimbal.pitch.target_pitch_angle += angle_change ;

  if (gimbal.pitch.target_pitch_angle >  PITCH_MAX_ANGLE)
    gimbal.pitch.target_pitch_angle =  PITCH_MAX_ANGLE;
  if (gimbal.pitch.target_pitch_angle < PITCH_MIN_ANGLE)
    gimbal.pitch.target_pitch_angle = PITCH_MIN_ANGLE;

  
  PID_Position_Calc(&gimbal.pitch_motor.pid_position, gimbal.pitch.target_pitch_angle, gimbal.pitch.pitch_angle);
  gimbal.pitch.fb_pitch_w = gimbal.pitch_motor.fb_speed;
  gimbal.pitch.target_pitch_w = gimbal.pitch_motor.pid_position.out_put;
  PID_Speed_Calc(&gimbal.pitch_motor.pid_speed, gimbal.pitch.target_pitch_w, gimbal.pitch.fb_pitch_w);

  gimbal.pitch.gravity_comp = 0.2f * gimbal.pitch.gravity_comp + gimbal.pitch.k_gravity_comp * 0.8f * cosf(gimbal.pitch.pitch_angle - gimbal.pitch.gravity_comp_offset);//pitch重力扭矩最大点不在水平也不在最低或最高处，在offset这个角度附近达到最大
  //pitch电机给电流 = 重力补偿 + PID输出
  gimbal.pitch_motor.give_current = -gimbal.pitch.gravity_comp + gimbal.pitch_motor.pid_speed.out_put;
}

void Gimbal_Update(void)
{
  gimbal.gyro.yaw_w = imu_attitude.gyro_body[2];
  gimbal.gyro.pitch_w = imu_attitude.gyro_body[1];
  gimbal.pitch.pitch_angle = wrap_pi(gimbal.pitch_motor.total_angle - gimbal.pitch.zero_angle);
  gimbal.yaw.yaw_angle = imu_attitude.yaw_continuous;
  gimbal.yaw.fb_yaw_w = - gimbal.gyro.yaw_w;
  gimbal.yaw.machine_yaw_angle = wrap_pi(gimbal.yaw_motor.fb_angle - gimbal.yaw.zero_angle);

  Yaw_Speed_Calc();
  Pitch_Speed_Calc();
}

