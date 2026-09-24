#ifndef __GIMBAL_H
#define __GIMBAL_H
#include "main.h"
#include "motor.h"


typedef struct
{
  Motor_t yaw_motor;
  Motor_t pitch_motor;

  struct
  {
    float yaw_w;
    float pitch_w;
  } gyro;

  struct
  {
    float fb_yaw_w;
    float target_yaw_w;
    float target_yaw_angle;
    float yaw_angle;
    float zero_angle;
    float machine_yaw_angle;
  } yaw;
  struct
  {
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

extern Gimbal_t gimbal;

/* 初始化云台控制。 */
void Gimbal_Init(void);
/* 返回云台是否完成初始化。 */
uint8_t Gimbal_IsInitialized(void);
/* 更新一拍云台控制。 */
void Gimbal_Update(void);
/* 急停解除后保持当前偏航角。 */
void Gimbal_HoldCurrentYawAfterEmergencyStop(void);

/* FreeRTOS 云台任务入口：2 ms 一轮，算出的 give_current 由 MotorTask 发 CAN。 */
void OS_GimbalCallback(void const *argument);

#endif
