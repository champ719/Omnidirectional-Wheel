#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "FreeRTOS.h"
#include "stdint.h"
#include "PID.h"

#define GEAR_RATE_3508 (3591.0f / 187.0f)
#define DJI_6020 1U
#define DJI_3508 2U
#define K_TORQUE_3508 0.3f
#define K_TORQUE_6020 0.741f
#define MOTOR_FEEDBACK_TIMEOUT_MS 100U

typedef struct Motor_t
{
  /* 速度类字段统一用「输出轴 rad/s」：3508 在 FeedbackTrans 里已除以 GEAR_RATE，
     6020 无减速箱、本来就是输出轴。target_speed 必须与此同量纲，
     否则速度环恒饱和（差 GEAR_RATE*60/(2π) ≈ 183 倍）。 */
  float fb_speed;      // 反馈转速 rad/s
  float fb_angle;      // 机械角 rad，-π..π
  float fb_current;    // 反馈电流 A
  uint8_t fb_temp;     // 温度 ℃
  float total_angle;   // 累计角 rad，-π..π
  float last_angle;
  float fb_torque;     // 反馈力矩 N·m = fb_current × K_TORQUE

  float target_speed;  // 目标转速 rad/s，与 fb_speed 同量纲
  float target_angle;
  float target_current;
  uint8_t target_temp;

  uint32_t cmd_id;
  uint8_t motor_type;
  float give_current;  // 下发电流 A，3508 限 ±20、6020 限 ±1.6，见 Motor_Trans
  volatile uint32_t feedback_tick;
  volatile uint8_t feedback_received;

  PID pid_speed;
  PID pid_position;
} Motor_t;

void Motor_Init(volatile Motor_t *motor, uint32_t cmd_id, uint8_t motor_type,
  float kp_speed, float ki_speed, float kd_speed, float kf_speed,
  float out_limit_speed, float integral_limit_speed,
  float kp_position, float ki_position, float kd_position, float kf_position,
  float out_limit_position, float integral_limit_position);
uint16_t Motor_Trans(volatile Motor_t *motor);
uint8_t Motor_IsOnline(const volatile Motor_t *motor);
uint8_t Motor_FeedbackHealthy(void);
void Motor_UPDATE(void);
void Motor_STOP(void);

/* FreeRTOS 电机任务入口：1ms 一轮，把各模块算好的 give_current 发到 CAN 总线 */
void OS_MotorCallback(void const *argument);

#endif
