#include "Power_Control.h"
#include "chassis.h"
#include "main.h"
#include "motor.h"

#include <math.h>

/* PI 输出是模型前馈缩放系数的修正量，量纲分别为 1/W 和 1/(W*s)。 */
#define POWER_FEEDBACK_KP                 0.008f
#define POWER_FEEDBACK_KI                 0.15f
#define POWER_CORRECTION_MIN             (-0.80f)
/* 实测闭环只允许进一步减小模型输出，不能反向放大电流。 */
#define POWER_CORRECTION_MAX              0.0f
#define POWER_LOOP_ACTIVE_RATIO           0.70f
#define POWER_FEEDBACK_DT_MIN_S            0.001f
#define POWER_FEEDBACK_DT_MAX_S            0.100f

static float PowerControl_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

static void PowerControl_SetMotorModel(uint8_t motor, float k1, float k2,
                                       float constant, float torque_scale)
{
  chassis.power_prediction.k1[motor] = k1;
  chassis.power_prediction.k2[motor] = k2;
  chassis.power_prediction.constant[motor] = constant;
  chassis.power_prediction.torque_scale[motor] = torque_scale;
}

static float PowerControl_PredictMotor(uint8_t motor, float current)
{
  float speed = chassis.motor_3508[motor].fb_speed;

  return chassis.power_prediction.k2[motor] * current * current
       + speed * K_TORQUE_3508
           * chassis.power_prediction.torque_scale[motor] * current
       + chassis.power_prediction.k1[motor] * speed * speed
       + chassis.power_prediction.constant[motor];
}

/* 根据电机模型求解 0..1 的前馈电流缩放系数。 */
static float PowerControl_CalculateFeedforward(float quadratic,
                                               float linear,
                                               float constant)
{
  const float coefficient_epsilon = 1.0e-6f;
  float requested_power = quadratic + linear + constant;
  float feedforward_limit = chassis.power_prediction.power_max
                          - CHASSIS_POWER_TARGET_MARGIN_W;
  float scale = 1.0f;

  chassis.power_prediction.requested_power = requested_power;
  if (!isfinite(requested_power)) {
    return 0.0f;
  }
  if (requested_power <= feedforward_limit) {
    return 1.0f;
  }

  scale = 0.0f;
  if (quadratic > coefficient_epsilon) {
    float equation_constant =
        constant - feedforward_limit;
    float discriminant = linear * linear
                       - 4.0f * quadratic * equation_constant;

    if (discriminant >= 0.0f) {
      float denominator = 2.0f * quadratic;
      float root_positive = (-linear + sqrtf(discriminant)) / denominator;
      float root_negative = (-linear - sqrtf(discriminant)) / denominator;

      if ((root_positive >= 0.0f) && (root_positive <= 1.0f)) {
        scale = root_positive;
      }
      if ((root_negative >= 0.0f) && (root_negative <= 1.0f) &&
          (root_negative > scale)) {
        scale = root_negative;
      }
    }
  } else if (linear > coefficient_epsilon) {
    scale = (feedforward_limit - constant) / linear;
  }

  return PowerControl_Clamp(scale, 0.0f, 1.0f);
}

/* 从 CAN ISR 写入的数据中取得一致快照，并检查功率计是否在线。 */
static uint8_t PowerControl_ReadFeedback(float *power,
                                         uint32_t *tick,
                                         uint32_t *sequence)
{
  uint32_t sequence_before;
  uint32_t sequence_after;
  float voltage;
  float current;

  if ((power == NULL) || (tick == NULL) || (sequence == NULL) ||
      (chassis.power_fb.received == 0U)) {
    return 0U;
  }

  do {
    sequence_before = chassis.power_fb.update_sequence;
    voltage = chassis.power_fb.voltage;
    current = chassis.power_fb.current;
    *power = chassis.power_fb.power;
    *tick = chassis.power_fb.update_tick;
    sequence_after = chassis.power_fb.update_sequence;
  } while (sequence_before != sequence_after);
  *sequence = sequence_after;

  if (!isfinite(voltage) || !isfinite(current) || !isfinite(*power) ||
      (voltage < 1.0f) || (voltage > 60.0f) ||
      ((HAL_GetTick() - *tick) > CHASSIS_POWER_FEEDBACK_TIMEOUT_MS)) {
    return 0U;
  }
  return 1U;
}

void PowerControl_Init(void)
{
  PowerControl_SetMotorModel(LF, 0.0092f, 0.1695f, 0.5396f, 0.8622f);
  PowerControl_SetMotorModel(RF, 0.0087f, 0.1043f, 0.5841f, 1.0644f);
  PowerControl_SetMotorModel(LB, 0.0079f, 0.0595f, 0.9790f, 1.1295f);
  PowerControl_SetMotorModel(RB, 0.0095f, 0.0876f, 0.5947f, 1.0428f);
  chassis.power_prediction.power_max = CHASSIS_POWER_LIMIT_W;
  PowerControl_Reset();
}

void PowerControl_Reset(void)
{
  uint8_t motor;

  for (motor = 0U; motor < 4U; motor++) {
    chassis.power_prediction.model_power[motor] = 0.0f;
  }
  chassis.power_prediction.requested_power = 0.0f;
  chassis.power_prediction.total_power = 0.0f;
  chassis.power_prediction.feedforward_scale = 0.0f;
  chassis.power_prediction.scale_factor = 0.0f;
  chassis.power_prediction.measured_power = 0.0f;
  chassis.power_prediction.power_error = 0.0f;
  chassis.power_prediction.feedback_integral = 0.0f;
  chassis.power_prediction.feedback_correction = 0.0f;
  chassis.power_prediction.feedback_sequence = 0U;
  chassis.power_prediction.feedback_tick = 0U;
  chassis.power_prediction.feedback_active = 0U;
}

void PowerControl_Apply(void)
{
  float quadratic = 0.0f;
  float linear = 0.0f;
  float constant = 0.0f;
  float measured_power = 0.0f;
  float model_scale;
  float final_scale;
  float target_power;
  uint32_t feedback_tick = 0U;
  uint32_t feedback_sequence = 0U;
  uint8_t feedback_valid;
  uint8_t motor;

  for (motor = 0U; motor < 4U; motor++) {
    float current = chassis.motor_3508[motor].give_current;
    float speed = chassis.motor_3508[motor].fb_speed;

    quadratic += chassis.power_prediction.k2[motor] * current * current;
    linear += speed * K_TORQUE_3508
            * chassis.power_prediction.torque_scale[motor] * current;
    constant += chassis.power_prediction.k1[motor] * speed * speed
              + chassis.power_prediction.constant[motor];
  }

  model_scale = PowerControl_CalculateFeedforward(quadratic, linear, constant);
  chassis.power_prediction.feedforward_scale = model_scale;
  final_scale = model_scale;
  target_power = chassis.power_prediction.power_max
               - CHASSIS_POWER_TARGET_MARGIN_W;
  feedback_valid = PowerControl_ReadFeedback(&measured_power,
                                              &feedback_tick,
                                              &feedback_sequence);

  if (feedback_valid != 0U) {
    chassis.power_prediction.measured_power = measured_power;

    /* 低负载不追逐功率上限；接近限功率区或已经超限时才闭环。 */
    if ((chassis.power_prediction.requested_power >=
         target_power * POWER_LOOP_ACTIVE_RATIO) ||
        (measured_power >= target_power * POWER_LOOP_ACTIVE_RATIO)) {
      float error = target_power - measured_power;

      if (chassis.power_prediction.feedback_active == 0U) {
        chassis.power_prediction.feedback_integral = 0.0f;
        chassis.power_prediction.feedback_tick = feedback_tick;
        chassis.power_prediction.feedback_sequence = feedback_sequence;
        chassis.power_prediction.feedback_active = 1U;
      } else if (feedback_sequence !=
                 chassis.power_prediction.feedback_sequence) {
        float dt = (float)(feedback_tick -
                           chassis.power_prediction.feedback_tick) * 0.001f;
        float candidate_integral;
        float candidate_scale;

        dt = PowerControl_Clamp(dt,
                                POWER_FEEDBACK_DT_MIN_S,
                                POWER_FEEDBACK_DT_MAX_S);
        candidate_integral = chassis.power_prediction.feedback_integral
                           + POWER_FEEDBACK_KI * error * dt;
        candidate_integral = PowerControl_Clamp(candidate_integral,
                                                 POWER_CORRECTION_MIN,
                                                 POWER_CORRECTION_MAX);
        candidate_scale = model_scale + POWER_FEEDBACK_KP * error
                        + candidate_integral;

        /* 条件积分抗饱和：输出顶住时不继续沿同方向积累。 */
        if (!(((candidate_scale >= 1.0f) && (error > 0.0f)) ||
              ((candidate_scale <= 0.0f) && (error < 0.0f)))) {
          chassis.power_prediction.feedback_integral = candidate_integral;
        }
        chassis.power_prediction.feedback_tick = feedback_tick;
        chassis.power_prediction.feedback_sequence = feedback_sequence;
      }

      chassis.power_prediction.power_error = error;
      chassis.power_prediction.feedback_correction =
          PowerControl_Clamp(POWER_FEEDBACK_KP * error
                           + chassis.power_prediction.feedback_integral,
                             POWER_CORRECTION_MIN,
                             POWER_CORRECTION_MAX);
      final_scale = model_scale
                  + chassis.power_prediction.feedback_correction;
      /* 安全不变量：反馈控制绝不能突破模型前馈给出的缩放上限。 */
      if (final_scale > model_scale) {
        final_scale = model_scale;
      }
    } else {
      chassis.power_prediction.feedback_active = 0U;
      chassis.power_prediction.feedback_integral = 0.0f;
      chassis.power_prediction.feedback_correction = 0.0f;
      chassis.power_prediction.power_error = target_power - measured_power;
      chassis.power_prediction.feedback_sequence = feedback_sequence;
      chassis.power_prediction.feedback_tick = feedback_tick;
    }
  } else {
    /* 功率计超时或数据非法：清 PI，退回原来的电机模型前馈保护。 */
    chassis.power_prediction.feedback_active = 0U;
    chassis.power_prediction.feedback_integral = 0.0f;
    chassis.power_prediction.feedback_correction = 0.0f;
    chassis.power_prediction.power_error = 0.0f;
  }

  if (!isfinite(chassis.power_prediction.requested_power)) {
    final_scale = 0.0f;
  }
  final_scale = PowerControl_Clamp(final_scale, 0.0f, 1.0f);
  chassis.power_prediction.scale_factor = final_scale;
  chassis.power_prediction.total_power = 0.0f;

  for (motor = 0U; motor < 4U; motor++) {
    chassis.motor_3508[motor].give_current *= final_scale;
    chassis.motor_3508[motor].pid_speed.output =
        chassis.motor_3508[motor].give_current;
    chassis.power_prediction.model_power[motor] =
        PowerControl_PredictMotor(
            motor, chassis.motor_3508[motor].give_current);
    chassis.power_prediction.total_power +=
        chassis.power_prediction.model_power[motor];
  }
}
