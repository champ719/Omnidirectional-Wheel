#include "motor.h"
#include "Error.h"
#include "USER_CAN.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "gimbal.h"
#include "task.h"

extern osThreadId MotorTaskHandle;

/* 初始化电机反馈、目标值和双环 PID 参数。 */
void Motor_Init(volatile Motor_t *motor, uint32_t cmd_id, uint8_t motor_type, float kp_speed, float ki_speed, float kd_speed, float kf_speed, float out_limit_speed, float integral_limit_speed, float kp_position, float ki_position, float kd_position, float kf_position, float out_limit_position, float integral_limit_position)
{
  motor->cmd_id = cmd_id;
  motor->motor_type = motor_type;
  motor->fb_speed = 0.0f;
  motor->fb_angle = 0.0f;
  motor->fb_current = 0.0f;
  motor->fb_temp = 0U;
  motor->total_angle = 0.0f;
  motor->last_angle = 0.0f;
  motor->fb_torque = 0.0f;
  motor->target_speed = 0.0f;
  motor->target_angle = 0.0f;
  motor->target_current = 0.0f;
  motor->target_temp = 0U;
  motor->give_current = 0.0f;
  motor->feedback_tick = 0U;
  motor->feedback_received = 0U;

  /* 新 PID 库没有 kf 项，两个 kf 入参保留只为不改调用点，实际不参与运算 */
  (void)kf_speed;
  (void)kf_position;

  PID_Init(&motor->pid_speed, kp_speed, ki_speed, kd_speed, integral_limit_speed, out_limit_speed);
  PID_Init(&motor->pid_position, kp_position, ki_position, kd_position, integral_limit_position, out_limit_position);
}

/* 将目标电流转换为 CAN 报文使用的 16 位控制值。 */
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

/* 判断单个电机反馈是否在线。 */
uint8_t Motor_IsOnline(const volatile Motor_t *motor)
{
  if ((motor == NULL) || (motor->feedback_received == 0U)) {
    return 0U;
  }

  return ((HAL_GetTick() - motor->feedback_tick) <= MOTOR_FEEDBACK_TIMEOUT_MS) ? 1U : 0U;
}

/* 检查当前参与控制的底盘和云台电机反馈。 */
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

/* 将各电机最新目标电流写入 CAN 发送缓存。 */
void Motor_UPDATE(void)
{
  CAN_SendMessage(&hcan1, &chassis.motor_3508[LF], Motor_Trans(&chassis.motor_3508[LF]), Motor_Trans(&chassis.motor_3508[RF]), Motor_Trans(&chassis.motor_3508[LB]), Motor_Trans(&chassis.motor_3508[RB]));
  CAN_SendMessage(&hcan2, &gimbal.pitch_motor, 0, Motor_Trans(&gimbal.pitch_motor), 0, 0);
  CAN_SendMessage(&hcan1, &gimbal.yaw_motor, Motor_Trans(&gimbal.yaw_motor), 0, 0, 0);
}

/* 向底盘和云台电机发送零电流。 */
void Motor_STOP(void)
{
  /* 四个 3508 共用 0x200 帧，取首个电机拿命令 ID，四路电流给 0 */
  CAN_SendMessage(&hcan1, &chassis.motor_3508[LF], 0, 0, 0, 0);
  CAN_SendMessage(&hcan1, &gimbal.yaw_motor, 0, 0, 0, 0);
  CAN_SendMessage(&hcan2, &gimbal.pitch_motor, 0, 0, 0, 0);
}

/* 通知电机任务重新检查控制模块初始化状态。 */
void Motor_NotifyControlInitialized(void)
{
    if (MotorTaskHandle != NULL) {
        xTaskNotifyGive(MotorTaskHandle);
    }
}

/* 每 2 ms 覆盖控制帧缓存并尝试送入 CAN 硬件邮箱。 */
void OS_MotorCallback(void const *argument)
{
    TickType_t last_wake;
    const TickType_t period = pdMS_TO_TICKS(2U);

    (void)argument;

    /* 电机命令ID由底盘和云台初始化写入。用通知等待真实就绪条件，
       不再依赖固定上电延时，也不轮询唤醒。 */
    while ((Chassis_IsInitialized() == 0U) || (Gimbal_IsInitialized() == 0U)) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    last_wake = xTaskGetTickCount();

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        if ((now - last_wake) > (period * 2U)) {
            last_wake = now;
        }
        if (Error_GetResult() != ERROR_RESULT_NONE) {
            Motor_STOP();
        } else {
            Motor_UPDATE();
        }
        CAN_Service();
        vTaskDelayUntil(&last_wake, period);
    }
}
