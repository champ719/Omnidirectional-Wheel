#include "motor.h"
#include "USER_CAN.h"
#include "chassis.h"
#include "gimbal.h"

void Motor_Init(volatile Motor_t *motor, uint32_t cmd_id, uint8_t motor_type,
  float kp_speed, float ki_speed, float kd_speed, float kf_speed,
  float out_limit_speed, float integral_limit_speed,
  float kp_position, float ki_position, float kd_position, float kf_position,
  float out_limit_position, float integral_limit_position)
{
  motor->cmd_id = cmd_id;
  motor->motor_type = motor_type;
  motor->give_current = 0.0f;
  motor->feedback_tick = 0U;
  motor->feedback_received = 0U;

  PID_Set(&motor->pid_speed, kp_speed, ki_speed, kd_speed, kf_speed,
          out_limit_speed, integral_limit_speed);
  PID_Set(&motor->pid_position, kp_position, ki_position, kd_position,
          kf_position, out_limit_position, integral_limit_position);
}

//把电机目标电流 give_current 转换为 CAN 报文中使用的 16 位控制值
uint16_t Motor_Trans(volatile Motor_t *motor)
{
  float current;
  float current_limit;
  float command_limit;
  int16_t command;

  if (motor == NULL) {
    return 0U;
  }

  current = motor->give_current;
  if (current != current) {
    return 0U;
  }

  if (motor->motor_type == DJI_3508) {
    current_limit = 20.0f;
    command_limit = 16384.0f;
  } else if (motor->motor_type == DJI_6020) {
    current_limit = 1.6f;
    command_limit = 25000.0f;
  } else {
    return 0U;
  }

  if (current > current_limit) {
    current = current_limit;
  } else if (current < -current_limit) {
    current = -current_limit;
  }

  command = (int16_t)(current / current_limit * command_limit);
  return (uint16_t)command;
}

//电机掉线保护
uint8_t Motor_IsOnline(const volatile Motor_t *motor)
{
  if ((motor == NULL) || (motor->feedback_received == 0U)) {
    return 0U;
  }

  return ((HAL_GetTick() - motor->feedback_tick) <=
          MOTOR_FEEDBACK_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t Motor_FeedbackHealthy(void)
{
  uint8_t index;

  for (index = 0U; index < 4U; index++) {
    if (Motor_IsOnline(&chassis.motor_3508[index]) == 0U) {
      return 0U;
    }
  }

  /* Pitch motor feedback check is temporarily disabled because the motor is
     unavailable. Keep chassis and yaw motor offline protection active. */
  return (Motor_IsOnline(&gimbal.yaw_motor) != 0U) ? 1U : 0U;
}

void Motor_UPDATE(void)
{
  CAN_SendMessage(&hcan1, &chassis.motor_3508[LF],
                  Motor_Trans(&chassis.motor_3508[LF]),
                  Motor_Trans(&chassis.motor_3508[RF]),
                  Motor_Trans(&chassis.motor_3508[LB]),
                  Motor_Trans(&chassis.motor_3508[RB]));

  CAN_SendMessage(&hcan2, &gimbal.pitch_motor,
                  0, Motor_Trans(&gimbal.pitch_motor), 0, 0);
  CAN_SendMessage(&hcan1, &gimbal.yaw_motor,
                  Motor_Trans(&gimbal.yaw_motor), 0, 0, 0);
}

void Motor_STOP(void)
{
  CAN_SendMessage(&hcan1, &chassis.motor_3508[LF], 0, 0, 0, 0);
  CAN_SendMessage(&hcan1, &gimbal.yaw_motor, 0, 0, 0, 0);
  CAN_SendMessage(&hcan2, &gimbal.pitch_motor, 0, 0, 0, 0);
}
