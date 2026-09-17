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
  float fb_speed;
  float fb_angle;
  float fb_current;
  uint8_t fb_temp;
  float total_angle;
  float last_angle;
  float fb_torque;

  float target_speed;
  float target_angle;
  float target_current;
  uint8_t target_temp;

  uint32_t cmd_id;
  uint8_t motor_type;
  float give_current;
  volatile uint32_t feedback_tick;
  volatile uint8_t feedback_received;

  PID_t pid_speed;
  PID_t pid_position;
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

#endif
