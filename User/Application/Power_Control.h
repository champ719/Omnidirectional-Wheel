#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H

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
  float total_power;
  float scale_factor;
} ChassisPowerControl_t;

void PowerControl_Init(void);
void PowerControl_Reset(void);
void PowerControl_Apply(void);

#endif
