#include "chassis.h"
#include "motor.h"
#include "Remote.h"
#include "Robot_Control.h"
#include <math.h>
#include "gimbal.h"

volatile Chassis_t chassis;

float fb_speed = 0.0f;
float fb_torque = 0.0f;
float give_current = 0.0f;
float pre_power = 0.0f;

void Power_Init(uint8_t Motor, float k1, float k2, float a, float torque_scale)
{
    chassis.power_prediction.k1[Motor] = k1;
    chassis.power_prediction.k2[Motor] = k2;
    chassis.power_prediction.a[Motor] = a;
    chassis.power_prediction.torque_scale[Motor] = torque_scale;
}

void Chassis_Init(void)
{ 
    Motor_Init(&chassis.motor_3508[LF], 0x200, TYPE_3508,
    0.4f, 0.2f, 0.7f, 0.2f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    Motor_Init(&chassis.motor_3508[RF], 0x200, TYPE_3508,
    0.8f, 0.1f, 0.6f, 0.0f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    Motor_Init(&chassis.motor_3508[LB], 0x200, TYPE_3508,
    0.8f, 0.1f, 0.6f, 0.0f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    Motor_Init(&chassis.motor_3508[RB], 0x200, TYPE_3508,
    0.8f, 0.1f, 0.6f, 0.0f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    
    PID_Set(&chassis.follow_speed, 5.0f, 0.005f, 80.0f, 0.0f, 5.0f, 3.0f);

    Power_Init(LF, 0.0092f, 0.0092f, 0.3282f, 1.1880f);
    Power_Init(RF, 0.0087f, 0.1043f, 0.5841f, 1.0644f);
    Power_Init(LB, 0.0079f, 0.0595f, 0.9790f, 1.1295f);
    Power_Init(RB, 0.0095f, 0.0876f, 0.5947f, 1.0428f);
    chassis.power_prediction.power_max = 100.0f;
    chassis.power_prediction.total_power = 0.0f;    
    for (int i = 0; i < 4; i++)
    {
      chassis.power_prediction.pre_target_speed[i] = 0.0f;
    }
}

void Power_Calc()
{
  float total_power = 0.0f;
  for (int i = 0; i < 4; i++)
  {
    chassis.power_prediction.model_power[i] = chassis.motor_3508[i].fb_torque * chassis.motor_3508[i].fb_speed 
       * chassis.power_prediction.torque_scale[i]
     + chassis.power_prediction.k1[i] * chassis.motor_3508[i].fb_speed * chassis.motor_3508[i].fb_speed 
     + chassis.power_prediction.k2[i] * chassis.motor_3508[i].give_current * chassis.motor_3508[i].give_current 
     + chassis.power_prediction.a[i];

     if(chassis.power_prediction.model_power[i] < 0.0f)
       continue;
     else
       total_power += chassis.power_prediction.model_power[i];
  }
   chassis.power_prediction.total_power = total_power; 
  if(chassis.power_prediction.total_power > chassis.power_prediction.power_max)
  {
    float scale = chassis.power_prediction.power_max / chassis.power_prediction.total_power;
    for (int i = 0; i < 4; i++)
    {
      if(chassis.power_prediction.model_power[i] < 0.0f)
        continue;
  
      chassis.power_prediction.model_power[i] *= scale;

      float a = chassis.power_prediction.k2[i];
      float b = chassis.motor_3508[i].fb_speed * K_TORQUE_3508 * chassis.power_prediction.torque_scale[i];
      float c = chassis.power_prediction.k1[i] * chassis.motor_3508[i].fb_speed * chassis.motor_3508[i].fb_speed 
              + chassis.power_prediction.a[i] - chassis.power_prediction.model_power[i];

      float delta = b*b - 4*a*c;
      if(delta < 0)
        chassis.motor_3508[i].give_current = -b / (2*a);
      else if(chassis.motor_3508[i].pid_speed.out_put > 0)
        {
        float temp = (-b + sqrt(delta)) / (2*a);
          if(temp > 20.0f)
            chassis.motor_3508[i].give_current = 20.0f;
          else
            chassis.motor_3508[i].give_current = temp;
        }
      else
      {
        float temp = (-b - sqrt(delta)) / (2*a);
          if(temp < -20.0f)
            chassis.motor_3508[i].give_current = -20.0f;
          else
            chassis.motor_3508[i].give_current = temp;
      }
    }
  }
}

void Chassis_Update(void)
{
    static uint8_t last_state = 0;

    if ((Remote_IsOnline() == 0U) ||
        (robot_control.command.source == ROBOT_CONTROL_SOURCE_NONE))
    {
      chassis.target_speed.Vx = 0.0f;
      chassis.target_speed.Vy = 0.0f;
      chassis.target_speed.w = 0.0f;
      last_state = 0;
      chassis.chassis_w_smooth = 0.0f;
    }
    else
    {
      float Vx_info = robot_control.command.forward * Vx_max;
      float Vy_info = robot_control.command.right * Vy_max;
      chassis.target_speed.Vx = Vx_info * cosf(gimbal.yaw.machine_yaw_angle) - Vy_info * sinf(gimbal.yaw.machine_yaw_angle);
      chassis.target_speed.Vy = Vx_info * sinf(gimbal.yaw.machine_yaw_angle) + Vy_info * cosf(gimbal.yaw.machine_yaw_angle);

      if ((uint8_t)robot_control.command.chassis_mode != last_state)
      {
        last_state = (uint8_t)robot_control.command.chassis_mode;
        PID_Clear(&chassis.motor_3508[LF].pid_speed);
        PID_Clear(&chassis.motor_3508[RF].pid_speed);
        PID_Clear(&chassis.motor_3508[LB].pid_speed);
        PID_Clear(&chassis.motor_3508[RB].pid_speed);
        chassis.chassis_w_smooth = chassis.fb_speed.w;
      }

      if(robot_control.command.chassis_mode == ROBOT_CHASSIS_FOLLOW)
      {
          if (gimbal.yaw.machine_yaw_angle > 0.03f || gimbal.yaw.machine_yaw_angle < -0.03f)
            {
              PID_Position_Calc(&chassis.follow_speed, 0, -gimbal.yaw.machine_yaw_angle);
              chassis.target_speed.w = chassis.follow_speed.out_put + gimbal.yaw.target_yaw_w;
            }
          else
            chassis.target_speed.w = gimbal.yaw.target_yaw_w;
      }
      else//突然反转容易超功率
      {
        float rotate_speed =
          -(float)robot_control.command.chassis_mode * 5.0f;
        chassis.chassis_w_smooth +=
          0.05f * (rotate_speed - chassis.chassis_w_smooth);
        chassis.target_speed.w = chassis.chassis_w_smooth;
      }
    }

    chassis.motor_3508[LF].target_speed = (+chassis.target_speed.Vx - chassis.target_speed.Vy - chassis.target_speed.w * K_ROTATE)/WHEEL_R;
    chassis.motor_3508[RF].target_speed = (-chassis.target_speed.Vx - chassis.target_speed.Vy - chassis.target_speed.w * K_ROTATE)/WHEEL_R;
    chassis.motor_3508[LB].target_speed = (+chassis.target_speed.Vx + chassis.target_speed.Vy - chassis.target_speed.w * K_ROTATE)/WHEEL_R;
    chassis.motor_3508[RB].target_speed = (-chassis.target_speed.Vx + chassis.target_speed.Vy - chassis.target_speed.w * K_ROTATE)/WHEEL_R;

    float max_speed = 0.0f;
    for (int i = 0; i < 4; i++)
    {
      float abs_spd = chassis.motor_3508[i].target_speed;
      if (abs_spd < 0.0f) abs_spd = -abs_spd;
      if (abs_spd > max_speed) max_speed = abs_spd;
    }
    if (max_speed > WHEEL_SPEED_MAX)
    {
      float scale = WHEEL_SPEED_MAX / max_speed;
      for (int i = 0; i < 4; i++)
        chassis.motor_3508[i].target_speed *= scale;
    }

    for (int i = 0; i < 4; i++)
    {
      float change = chassis.motor_3508[i].target_speed - chassis.power_prediction.pre_target_speed[i];
      if (change >  TARGET_SPEED_STEP) chassis.motor_3508[i].target_speed = chassis.power_prediction.pre_target_speed[i] + TARGET_SPEED_STEP;
      if (change < -TARGET_SPEED_STEP) chassis.motor_3508[i].target_speed = chassis.power_prediction.pre_target_speed[i] - TARGET_SPEED_STEP;
      chassis.power_prediction.pre_target_speed[i] = chassis.motor_3508[i].target_speed;
    }

    chassis.fb_speed.Vx = (chassis.motor_3508[LF].fb_speed - chassis.motor_3508[RF].fb_speed + chassis.motor_3508[LB].fb_speed - chassis.motor_3508[RB].fb_speed) * WHEEL_R / 4.0f;
    chassis.fb_speed.Vy = (-chassis.motor_3508[LF].fb_speed - chassis.motor_3508[RF].fb_speed + chassis.motor_3508[LB].fb_speed + chassis.motor_3508[RB].fb_speed) * WHEEL_R / 4.0f;
    chassis.fb_speed.w = (-chassis.motor_3508[LF].fb_speed - chassis.motor_3508[RF].fb_speed - chassis.motor_3508[LB].fb_speed - chassis.motor_3508[RB].fb_speed) * WHEEL_R / (4.0f * K_ROTATE);

    for (int i = 0; i < 4; i++)
    {
      PID_Speed_Calc(&chassis.motor_3508[i].pid_speed, chassis.motor_3508[i].target_speed, chassis.motor_3508[i].fb_speed);
      chassis.motor_3508[i].give_current = chassis.motor_3508[i].pid_speed.out_put;

    }

    Power_Calc();
    //fb_speed = chassis.motor_3508[RB].fb_speed;
    //fb_torque = chassis.motor_3508[RB].fb_torque;
    //give_current = chassis.motor_3508[RB].give_current;
    //pre_power = chassis.power_prediction.model_power[RB];

}

