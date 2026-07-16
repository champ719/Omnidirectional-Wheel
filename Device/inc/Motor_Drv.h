#ifndef MOTOR_DRV_H
#define MOTOR_DRV_H

#include "main.h"
#include "can.h"

typedef struct
{
    uint16_t Position;
    float Velocity;
    int16_t Speed;
    int16_t Torque;
    uint8_t temperate;
} DJI_Rx_Data_t;

typedef struct
{
    DJI_Rx_Data_t Rx_Data;
    float Target_Torque;
    uint16_t motor_id;
} Wheel_Motor_t;

void Motor_Drv_Init(void);
void Motor_Drv_StopAll(void);
void DJI3508_Get_Data(uint8_t *data, Wheel_Motor_t *motor);
void GM6020_Get_Data(uint8_t *data, Wheel_Motor_t *motor);
HAL_StatusTypeDef Motor_Drv_Send_All(void);
void Motor_Drv_SetWheelTorque(uint8_t index, float torque);
void Motor_Drv_SetFrictionWheelTorque(uint8_t index, float torque);
void Motor_Drv_SetGimbalTorque(uint8_t index, float torque);

extern Wheel_Motor_t wheel_motors[4];
extern Wheel_Motor_t friction_wheels[2];
extern Wheel_Motor_t gimbal_motors[2];

#endif
