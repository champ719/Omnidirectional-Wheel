#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H

#include <stdint.h>

#define CHASSIS_POWER_LIMIT_W 100.0f

typedef struct
{
  float k1[4];
  float k2[4];
  float constant[4];
  float torque_scale[4];
  float power_max;
  float requested_power;
  float model_power[4];
  /* 调试采集器对数组元素支持不一致，保留四个独立镜像变量。 */
  float motor0_power;
  float motor1_power;
  float motor2_power;
  float motor3_power;
  float total_power;
  float feedforward_scale;
  float scale_factor;
  float measured_power;
  float power_error;
  float feedback_integral;
  float feedback_correction;
  uint32_t feedback_sequence;
  uint32_t feedback_tick;
  uint8_t feedback_active;

  /* 以下字段放在结构体末尾，避免改变前面已有调试变量的地址。 */
  float requested_power_raw;
  float requested_current_square_sum;
  float requested_quadratic_raw;
  float requested_quadratic_corrected;
  float requested_linear_power;
  float requested_constant_power;
  float applied_current_square_sum;
  float applied_quadratic_raw;
  float applied_quadratic_corrected;
  float applied_linear_power;
  float applied_constant_power;
  float total_power_raw;
  float quadratic_correction;

  /* 二次项达到该功率后开始修正，单位 W；调试时根据残差拐点设置。 */
  float quadratic_correction_start_w;
  /* 大电流区二次项增益，范围 0~1；1 表示不修正。 */
  float quadratic_high_current_gain;
} ChassisPowerControl_t;

/* 初始化四轮功率模型和总功率控制参数。 */
void PowerControl_Init(void);
/* 清空功率控制运行状态并保留模型与调参参数。 */
void PowerControl_Reset(void);
/* 预测本周期功率并统一限制四轮输出电流。 */
void PowerControl_Apply(void);

#endif
