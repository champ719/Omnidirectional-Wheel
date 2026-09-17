#include "Power_Control.h"
#include "chassis.h"
#include "motor.h"
#include <math.h>

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

void PowerControl_Init(void)
{
  PowerControl_SetMotorModel(LF, 0.0092f, 0.0092f, 0.3282f, 1.1880f);
  PowerControl_SetMotorModel(RF, 0.0087f, 0.1043f, 0.5841f, 1.0644f);
  PowerControl_SetMotorModel(LB, 0.0079f, 0.0595f, 0.9790f, 1.1295f);
  PowerControl_SetMotorModel(RB, 0.0095f, 0.0876f, 0.5947f, 1.0428f);
  chassis.power_prediction.power_max = CHASSIS_POWER_LIMIT_W;
  PowerControl_Reset();
}

void PowerControl_Reset(void)
{
  for (uint8_t motor = 0U; motor < 4U; motor++) {
    chassis.power_prediction.model_power[motor] = 0.0f;
  }
  chassis.power_prediction.requested_power = 0.0f;
  chassis.power_prediction.total_power = 0.0f;
  chassis.power_prediction.scale_factor = 0.0f;
}

void PowerControl_Apply(void)
{
  const float coefficient_epsilon = 1.0e-6f;
  float quadratic = 0.0f;
  float linear = 0.0f;
  float constant = 0.0f;
  float scale = 1.0f;

  for (uint8_t motor = 0U; motor < 4U; motor++) {
    float current = chassis.motor_3508[motor].give_current;
    float speed = chassis.motor_3508[motor].fb_speed;

    quadratic += chassis.power_prediction.k2[motor] * current * current;
    linear += speed * K_TORQUE_3508
            * chassis.power_prediction.torque_scale[motor] * current;
    constant += chassis.power_prediction.k1[motor] * speed * speed
              + chassis.power_prediction.constant[motor];
  }

  chassis.power_prediction.requested_power = quadratic + linear + constant;

  if (chassis.power_prediction.requested_power !=
      chassis.power_prediction.requested_power) {
    scale = 0.0f;
  } else if (chassis.power_prediction.requested_power >
             chassis.power_prediction.power_max) {
    float equation_constant =
        constant - chassis.power_prediction.power_max;

    scale = 0.0f;
    if (quadratic > coefficient_epsilon) {
      float discriminant = linear * linear
                         - 4.0f * quadratic * equation_constant;

      if (discriminant >= 0.0f) {
        float denominator = 2.0f * quadratic;
        float root_positive =
            (-linear + sqrtf(discriminant)) / denominator;
        float root_negative =
            (-linear - sqrtf(discriminant)) / denominator;

        if ((root_positive >= 0.0f) && (root_positive <= 1.0f)) {
          scale = root_positive;
        }
        if ((root_negative >= 0.0f) && (root_negative <= 1.0f) &&
            (root_negative > scale)) {
          scale = root_negative;
        }
      }
    } else if (linear > coefficient_epsilon) {
      scale = (chassis.power_prediction.power_max - constant) / linear;
      if (scale < 0.0f) {
        scale = 0.0f;
      } else if (scale > 1.0f) {
        scale = 1.0f;
      }
    }
  }

  chassis.power_prediction.scale_factor = scale;
  chassis.power_prediction.total_power = 0.0f;
  for (uint8_t motor = 0U; motor < 4U; motor++) {
    chassis.motor_3508[motor].give_current *= scale;
    chassis.motor_3508[motor].pid_speed.output =
        chassis.motor_3508[motor].give_current;
    chassis.power_prediction.model_power[motor] =
        PowerControl_PredictMotor(
            motor, chassis.motor_3508[motor].give_current);
    chassis.power_prediction.total_power +=
        chassis.power_prediction.model_power[motor];
  }
}
