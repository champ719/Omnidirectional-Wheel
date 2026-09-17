#include "chassis.h"
#include "motor.h"
#include "Remote.h"
#include "Robot_Control.h"
#include <math.h>
#include "gimbal.h"

volatile Chassis_t chassis;

void Chassis_Init(void)
{ 
    Motor_Init(&chassis.motor_3508[LF], 0x200, DJI_3508,
    0.4f, 0.2f, 0.7f, 0.2f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    Motor_Init(&chassis.motor_3508[RF], 0x200, DJI_3508,
    0.8f, 0.1f, 0.6f, 0.0f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    Motor_Init(&chassis.motor_3508[LB], 0x200, DJI_3508,
    0.8f, 0.1f, 0.6f, 0.0f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    Motor_Init(&chassis.motor_3508[RB], 0x200, DJI_3508,
    0.8f, 0.1f, 0.6f, 0.0f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    
    PID_Set(&chassis.follow_speed, 5.0f, 0.005f, 80.0f, 0.0f, 5.0f, 3.0f);

    Slope_Init(&chassis.command_slope.forward,
               CHASSIS_TRANSLATION_ACCEL_MPS2 * ROBOT_CONTROL_PERIOD_S,
               0.0f);
    Slope_Init(&chassis.command_slope.right,
               CHASSIS_TRANSLATION_ACCEL_MPS2 * ROBOT_CONTROL_PERIOD_S,
               0.0f);
    Slope_Init(&chassis.command_slope.rotation,
               CHASSIS_ROTATION_ACCEL_RADPS2 * ROBOT_CONTROL_PERIOD_S,
               0.0f);

    PowerControl_Init();
    Chassis_ResetControl();
}

void Chassis_ResetControl(void)
{
    Slope_Reset(&chassis.command_slope.forward, 0.0f);
    Slope_Reset(&chassis.command_slope.right, 0.0f);
    Slope_Reset(&chassis.command_slope.rotation, 0.0f);

    chassis.target_speed.Vx = 0.0f;
    chassis.target_speed.Vy = 0.0f;
    chassis.target_speed.w = 0.0f;
    PID_Clear(&chassis.follow_speed);

    for (uint8_t index = 0U; index < 4U; index++) {
      chassis.motor_3508[index].target_speed = 0.0f;
      chassis.motor_3508[index].give_current = 0.0f;
      PID_Clear(&chassis.motor_3508[index].pid_speed);
    }
    PowerControl_Reset();
}

void Task_Chassis_Callback()
{
    static uint8_t last_state = 0;
    float target_w;

    if ((Remote_IsOnline() == 0U) ||
        (robot_control.command.source == ROBOT_CONTROL_SOURCE_NONE))
    {
      Chassis_ResetControl();
      last_state = 0;
      return;
    }
    else
    {
      float Vx_info;
      float Vy_info;

      Slope_SetTarget(&chassis.command_slope.forward,
                      robot_control.command.forward * Vx_max);
      Slope_SetTarget(&chassis.command_slope.right,
                      robot_control.command.right * Vy_max);
      Vx_info = Slope_NextVal(&chassis.command_slope.forward);
      Vy_info = Slope_NextVal(&chassis.command_slope.right);
      chassis.target_speed.Vx = Vx_info * cosf(gimbal.yaw.machine_yaw_angle) - Vy_info * sinf(gimbal.yaw.machine_yaw_angle);
      chassis.target_speed.Vy = Vx_info * sinf(gimbal.yaw.machine_yaw_angle) + Vy_info * cosf(gimbal.yaw.machine_yaw_angle);

      if ((uint8_t)robot_control.command.chassis_mode != last_state)
      {
        last_state = (uint8_t)robot_control.command.chassis_mode;
        PID_Clear(&chassis.motor_3508[LF].pid_speed);
        PID_Clear(&chassis.motor_3508[RF].pid_speed);
        PID_Clear(&chassis.motor_3508[LB].pid_speed);
        PID_Clear(&chassis.motor_3508[RB].pid_speed);
        Slope_Reset(&chassis.command_slope.rotation, chassis.fb_speed.w);
      }

      if(robot_control.command.chassis_mode == ROBOT_CHASSIS_FOLLOW)
      {
          if (gimbal.yaw.machine_yaw_angle > 0.03f || gimbal.yaw.machine_yaw_angle < -0.03f)
            {
              PID_Position_Calc(&chassis.follow_speed, 0, -gimbal.yaw.machine_yaw_angle);
              target_w = chassis.follow_speed.out_put + gimbal.yaw.target_yaw_w;
            }
          else
            target_w = gimbal.yaw.target_yaw_w;
      }
      else//突然反转容易超功率
      {
        target_w = -(float)robot_control.command.chassis_mode * 5.0f;
      }

      Slope_SetTarget(&chassis.command_slope.rotation, target_w);
      chassis.target_speed.w =
        Slope_NextVal(&chassis.command_slope.rotation);
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

    chassis.fb_speed.Vx = (chassis.motor_3508[LF].fb_speed - chassis.motor_3508[RF].fb_speed + chassis.motor_3508[LB].fb_speed - chassis.motor_3508[RB].fb_speed) * WHEEL_R / 4.0f;
    chassis.fb_speed.Vy = (-chassis.motor_3508[LF].fb_speed - chassis.motor_3508[RF].fb_speed + chassis.motor_3508[LB].fb_speed + chassis.motor_3508[RB].fb_speed) * WHEEL_R / 4.0f;
    chassis.fb_speed.w = (-chassis.motor_3508[LF].fb_speed - chassis.motor_3508[RF].fb_speed - chassis.motor_3508[LB].fb_speed - chassis.motor_3508[RB].fb_speed) * WHEEL_R / (4.0f * K_ROTATE);

    for (int i = 0; i < 4; i++)
    {
      PID_Speed_Calc(&chassis.motor_3508[i].pid_speed, chassis.motor_3508[i].target_speed, chassis.motor_3508[i].fb_speed);
      chassis.motor_3508[i].give_current = chassis.motor_3508[i].pid_speed.out_put;

    }

    PowerControl_Apply();
}

void OS_ChassisCallback(void const * argument)
{
	osDelay(1500);
	Chassis_Init();
    for(;;)
    {
		Task_Chassis_Callback();
        osDelay(2);
    }
}

