#include "Power_Control.h"
#include "chassis.h"
#include "main.h"
#include "motor.h"

#include <math.h>

/* 功率目标相对硬上限保留的安全余量，单位 W。 */
static const float power_target_margin_w = 5.0f;
/* 功率反馈 PI 比例增益，单位 1/W。 */
static const float power_feedback_kp = 0.008f;
/* 功率反馈 PI 积分增益，单位 1/(W*s)。 */
static const float power_feedback_ki = 0.15f;
/* 反馈最多可额外降低的电流缩放量。 */
static const float power_correction_min = -0.80f;
/* 反馈不允许突破模型给出的前馈缩放上限。 */
static const float power_correction_max = 0.0f;
/* 预测或实测达到目标功率该比例后启用反馈环。 */
static const float power_loop_active_ratio = 0.70f;
/* 功率反馈积分使用的最小时间间隔，单位 s。 */
static const float power_feedback_dt_min_s = 0.001f;
/* 功率反馈积分使用的最大时间间隔，单位 s。 */
static const float power_feedback_dt_max_s = 0.100f;
/* 功率计反馈超过该时间未更新即判为无效，单位 ms。 */
static const uint32_t power_feedback_timeout_ms = 100U;

/* 将数值限制在指定闭区间内。 */
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

/* 写入指定底盘电机的功率模型参数。 */
static void PowerControl_SetMotorModel(uint8_t motor, float k1, float k2, float constant, float torque_scale)
{
  chassis.power_prediction.k1[motor] = k1;
  chassis.power_prediction.k2[motor] = k2;
  chassis.power_prediction.constant[motor] = constant;
  chassis.power_prediction.torque_scale[motor] = torque_scale;
}

/* 根据电流和反馈转速计算单电机原始预测功率。 */
static float PowerControl_PredictMotor(uint8_t motor, float current)
{
  float speed = chassis.motor_3508[motor].fb_speed;

  return chassis.power_prediction.k2[motor] * current * current + speed * K_TORQUE_3508 * chassis.power_prediction.torque_scale[motor] * current + chassis.power_prediction.k1[motor] * speed * speed + chassis.power_prediction.constant[motor];
}

/* 对四轮二次电流损耗的合计值做连续分段修正。 */
static float PowerControl_CorrectQuadratic(float quadratic)
{
  float threshold = chassis.power_prediction.quadratic_correction_start_w;
  float gain = chassis.power_prediction.quadratic_high_current_gain;

  if (!isfinite(threshold) || (threshold < 0.0f)) {
    threshold = 0.0f;
  }
  if (!isfinite(gain)) {
    gain = 1.0f;
  }
  gain = PowerControl_Clamp(gain, 0.0f, 1.0f);
  if (quadratic <= threshold) {
    return quadratic;
  }
  return threshold + gain * (quadratic - threshold);
}

/* 计算统一缩放四轮电流后的修正总功率。 */
static float PowerControl_PredictTotalAtScale(float quadratic, float linear, float constant, float scale)
{
  float scaled_quadratic = quadratic * scale * scale;

  return PowerControl_CorrectQuadratic(scaled_quadratic) + linear * scale + constant;
}

/* 求解不超过目标功率的最大前馈电流缩放系数。 */
static float PowerControl_CalculateFeedforward(float quadratic, float linear, float constant)
{
  const uint8_t bracket_step_count = 16U;
  const uint8_t iteration_count = 20U;
  float requested_power_raw = quadratic + linear + constant;
  float requested_power = PowerControl_PredictTotalAtScale(quadratic, linear, constant, 1.0f);
  float feedforward_limit = chassis.power_prediction.power_max - power_target_margin_w;
  float low = 0.0f;
  float high = 1.0f;
  uint8_t bracket_found = 0U;
  uint8_t iteration;

  chassis.power_prediction.requested_power_raw = requested_power_raw;
  chassis.power_prediction.requested_power = requested_power;
  if (!isfinite(requested_power_raw) || !isfinite(requested_power) || !isfinite(feedforward_limit)) {
    return 0.0f;
  }
  if (requested_power <= feedforward_limit) {
    return 1.0f;
  }

  /* 即使电流缩放到零，转速损耗仍可能超过目标，此时只能输出零电流。 */
  if (PowerControl_PredictTotalAtScale(quadratic, linear, constant, 0.0f) > feedforward_limit) {
    return 0.0f;
  }

  /* 制动工况的 linear 可能为负，分段模型不保证全区间严格单调。
     从 1 向下找第一个可行点，确保求到最大的安全缩放系数。 */
  for (iteration = 1U; iteration <= bracket_step_count; iteration++) {
    float candidate = 1.0f - (float)iteration / (float)bracket_step_count;
    float candidate_power = PowerControl_PredictTotalAtScale(quadratic, linear, constant, candidate);

    if (candidate_power <= feedforward_limit) {
      low = candidate;
      bracket_found = 1U;
      break;
    }
    high = candidate;
  }
  if (bracket_found == 0U) {
    return 0.0f;
  }

  for (iteration = 0U; iteration < iteration_count; iteration++) {
    float middle = 0.5f * (low + high);
    float middle_power = PowerControl_PredictTotalAtScale(quadratic, linear, constant, middle);

    if (middle_power <= feedforward_limit) {
      low = middle;
    } else {
      high = middle;
    }
  }
  return low;
}

/* 读取一致的功率计快照并判断反馈是否有效。 */
static uint8_t PowerControl_ReadFeedback(float *power, uint32_t *tick, uint32_t *sequence)
{
  uint32_t sequence_before;
  uint32_t sequence_after;
  float voltage;
  float current;

  if ((power == NULL) || (tick == NULL) || (sequence == NULL) || (chassis.power_fb.received == 0U)) {
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

  if (!isfinite(voltage) || !isfinite(current) || !isfinite(*power) || (voltage < 1.0f) || (voltage > 60.0f) || ((HAL_GetTick() - *tick) > power_feedback_timeout_ms)) {
    return 0U;
  }
  return 1U;
}

/* 初始化四轮模型、功率上限和二次项修正参数。 */
void PowerControl_Init(void)
{
  PowerControl_SetMotorModel(LF, 0.00644f, 0.1695f, 0.5396f, 0.8622f);
  PowerControl_SetMotorModel(RF, 0.00609f, 0.1043f, 0.5841f, 1.0644f);
  PowerControl_SetMotorModel(LB, 0.00553f, 0.0595f, 0.9790f, 1.1295f);
  PowerControl_SetMotorModel(RB, 0.00665f, 0.0876f, 0.5947f, 1.0428f);
  chassis.power_prediction.power_max = CHASSIS_POWER_LIMIT_W;
  chassis.power_prediction.quadratic_correction_start_w = 0.0f;
  chassis.power_prediction.quadratic_high_current_gain = 1.0f;
  PowerControl_Reset();
}

/* 清空运行状态并保留模型、功率上限和调参参数。 */
void PowerControl_Reset(void)
{
  uint8_t motor;

  for (motor = 0U; motor < 4U; motor++) {
    chassis.power_prediction.model_power[motor] = 0.0f;
  }
  chassis.power_prediction.motor0_power = 0.0f;
  chassis.power_prediction.motor1_power = 0.0f;
  chassis.power_prediction.motor2_power = 0.0f;
  chassis.power_prediction.motor3_power = 0.0f;
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
  chassis.power_prediction.requested_power_raw = 0.0f;
  chassis.power_prediction.requested_current_square_sum = 0.0f;
  chassis.power_prediction.requested_quadratic_raw = 0.0f;
  chassis.power_prediction.requested_quadratic_corrected = 0.0f;
  chassis.power_prediction.requested_linear_power = 0.0f;
  chassis.power_prediction.requested_constant_power = 0.0f;
  chassis.power_prediction.applied_current_square_sum = 0.0f;
  chassis.power_prediction.applied_quadratic_raw = 0.0f;
  chassis.power_prediction.applied_quadratic_corrected = 0.0f;
  chassis.power_prediction.applied_linear_power = 0.0f;
  chassis.power_prediction.applied_constant_power = 0.0f;
  chassis.power_prediction.total_power_raw = 0.0f;
  chassis.power_prediction.quadratic_correction = 0.0f;
}

/* 预测总功率、执行反馈修正并统一缩放四轮电流。 */
void PowerControl_Apply(void)
{
  float quadratic = 0.0f;
  float linear = 0.0f;
  float constant = 0.0f;
  float current_square_sum = 0.0f;
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

    current_square_sum += current * current;
    quadratic += chassis.power_prediction.k2[motor] * current * current;
    linear += speed * K_TORQUE_3508 * chassis.power_prediction.torque_scale[motor] * current;
    constant += chassis.power_prediction.k1[motor] * speed * speed + chassis.power_prediction.constant[motor];
  }

  chassis.power_prediction.requested_current_square_sum = current_square_sum;
  chassis.power_prediction.requested_quadratic_raw = quadratic;
  chassis.power_prediction.requested_quadratic_corrected = PowerControl_CorrectQuadratic(quadratic);
  chassis.power_prediction.requested_linear_power = linear;
  chassis.power_prediction.requested_constant_power = constant;

  model_scale = PowerControl_CalculateFeedforward(quadratic, linear, constant);
  chassis.power_prediction.feedforward_scale = model_scale;
  final_scale = model_scale;
  target_power = chassis.power_prediction.power_max - power_target_margin_w;
  feedback_valid = PowerControl_ReadFeedback(&measured_power, &feedback_tick, &feedback_sequence);

  if (feedback_valid != 0U) {
    chassis.power_prediction.measured_power = measured_power;

    /* 低负载不追逐功率上限；接近限功率区或已经超限时才闭环。 */
    if ((chassis.power_prediction.requested_power >= target_power * power_loop_active_ratio) || (measured_power >= target_power * power_loop_active_ratio)) {
      float error = target_power - measured_power;

      if (chassis.power_prediction.feedback_active == 0U) {
        chassis.power_prediction.feedback_integral = 0.0f;
        chassis.power_prediction.feedback_tick = feedback_tick;
        chassis.power_prediction.feedback_sequence = feedback_sequence;
        chassis.power_prediction.feedback_active = 1U;
      } else if (feedback_sequence != chassis.power_prediction.feedback_sequence) {
        float dt = (float)(feedback_tick - chassis.power_prediction.feedback_tick) * 0.001f;
        float candidate_integral;
        float candidate_scale;

        dt = PowerControl_Clamp(dt, power_feedback_dt_min_s, power_feedback_dt_max_s);
        candidate_integral = chassis.power_prediction.feedback_integral + power_feedback_ki * error * dt;
        candidate_integral = PowerControl_Clamp(candidate_integral, power_correction_min, power_correction_max);
        candidate_scale = model_scale + power_feedback_kp * error + candidate_integral;

        /* 条件积分抗饱和：输出顶住时不继续沿同方向积累。 */
        if (!(((candidate_scale >= 1.0f) && (error > 0.0f)) || ((candidate_scale <= 0.0f) && (error < 0.0f)))) {
          chassis.power_prediction.feedback_integral = candidate_integral;
        }
        chassis.power_prediction.feedback_tick = feedback_tick;
        chassis.power_prediction.feedback_sequence = feedback_sequence;
      }

      chassis.power_prediction.power_error = error;
      chassis.power_prediction.feedback_correction = PowerControl_Clamp(power_feedback_kp * error + chassis.power_prediction.feedback_integral, power_correction_min, power_correction_max);
      final_scale = model_scale + chassis.power_prediction.feedback_correction;
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
  chassis.power_prediction.total_power_raw = 0.0f;

  for (motor = 0U; motor < 4U; motor++) {
    chassis.motor_3508[motor].give_current *= final_scale;
    chassis.motor_3508[motor].pid_speed.output = chassis.motor_3508[motor].give_current;
    chassis.power_prediction.model_power[motor] = PowerControl_PredictMotor(motor, chassis.motor_3508[motor].give_current);
    chassis.power_prediction.total_power_raw += chassis.power_prediction.model_power[motor];
  }

  chassis.power_prediction.motor0_power = chassis.power_prediction.model_power[LF];
  chassis.power_prediction.motor1_power = chassis.power_prediction.model_power[RF];
  chassis.power_prediction.motor2_power = chassis.power_prediction.model_power[LB];
  chassis.power_prediction.motor3_power = chassis.power_prediction.model_power[RB];

  chassis.power_prediction.applied_current_square_sum = current_square_sum * final_scale * final_scale;
  chassis.power_prediction.applied_quadratic_raw = quadratic * final_scale * final_scale;
  chassis.power_prediction.applied_quadratic_corrected = PowerControl_CorrectQuadratic(chassis.power_prediction.applied_quadratic_raw);
  chassis.power_prediction.applied_linear_power = linear * final_scale;
  chassis.power_prediction.applied_constant_power = constant;
  chassis.power_prediction.quadratic_correction = chassis.power_prediction.applied_quadratic_corrected - chassis.power_prediction.applied_quadratic_raw;
  chassis.power_prediction.total_power = chassis.power_prediction.total_power_raw + chassis.power_prediction.quadratic_correction;
}
