#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H

#include <stdint.h>

#define CHASSIS_POWER_LIMIT_W 100.0f
#define CHASSIS_POWER_TARGET_MARGIN_W 5.0f
#define CHASSIS_POWER_FEEDBACK_TIMEOUT_MS 100U

typedef struct
{
  float k1[4];
  float k2[4];
  float constant[4];
  float torque_scale[4];
  float power_max;
  float requested_power;
  float model_power[4];
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
} ChassisPowerControl_t;

void PowerControl_Init(void);
void PowerControl_Reset(void);
void PowerControl_Apply(void);

#endif
