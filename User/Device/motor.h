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
  float fb_speed;      /* 反馈转速，单位 rad/s。 */
  float fb_angle;      /* 机械角，单位 rad，范围 -pi~pi。 */
  float fb_current;    /* 反馈电流，单位 A。 */
  uint8_t fb_temp;     /* 温度，单位摄氏度。 */
  float total_angle;   /* 累计角，单位 rad，范围 -pi~pi。 */
  float last_angle;
  float fb_torque;     /* 反馈力矩，单位 N·m。 */

  float target_speed;  /* 目标转速，单位 rad/s。 */
  float target_angle;
  float target_current;
  uint8_t target_temp;

  uint32_t cmd_id;
  uint8_t motor_type;
  float give_current;  /* 下发电流，单位 A；限幅见 Motor_Trans。 */
  volatile uint32_t feedback_tick;
  volatile uint8_t feedback_received;

  PID pid_speed;
  PID pid_position;
} Motor_t;

/* 初始化电机状态和双环 PID。 */
void Motor_Init(volatile Motor_t *motor, uint32_t cmd_id, uint8_t motor_type, float kp_speed, float ki_speed, float kd_speed, float kf_speed, float out_limit_speed, float integral_limit_speed, float kp_position, float ki_position, float kd_position, float kf_position, float out_limit_position, float integral_limit_position);
/* 将目标电流转换为 CAN 控制值。 */
uint16_t Motor_Trans(volatile Motor_t *motor);
/* 判断单个电机是否在线。 */
uint8_t Motor_IsOnline(const volatile Motor_t *motor);
/* 检查参与控制的电机反馈。 */
uint8_t Motor_FeedbackHealthy(void);
/* 更新全部电机控制帧。 */
void Motor_UPDATE(void);
/* 向全部电机发送零电流。 */
void Motor_STOP(void);
/* 通知电机任务控制模块已完成初始化。 */
void Motor_NotifyControlInitialized(void);

/* FreeRTOS 电机任务入口：2 ms 一轮，将最新 give_current 提交给 CAN 覆盖式缓存。 */
void OS_MotorCallback(void const *argument);

#endif
