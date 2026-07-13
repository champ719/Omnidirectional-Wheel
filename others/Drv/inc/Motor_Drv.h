#ifndef MOTOR_DRV_H
#define MOTOR_DRV_H

#include "main.h"
#include "can.h"

typedef struct DJI_Rx_Data
{
    uint16_t Position;
    float Velocity;
    int16_t Speed;
    int16_t Torque;
    uint8_t temperate;
    int16_t last_ecd;
} DJI_Rx_Data_t;

typedef struct Wheel_Motor
{
    DJI_Rx_Data_t Rx_Data;
    int16_t TX_data;
    float Target_Torque;
    float TMAX;
    float PMAX;
    float VMAX;
    uint16_t motor_id;
    uint16_t rx_timeout;
    uint8_t rx_received;
} Wheel_Motor_t;

void DM_Wheel_Motor_Init(Wheel_Motor_t *Motor, float TMAX, float PMAX, float VMAX, uint16_t motor_id);
void Motor_Drv_Init(void);
void DJI3508_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor);
void GM6020_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor);
void DJI3508_Torque_Ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float torque);
void GM6020_Angle_Ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float angle);
void Motor_Drv_Send_All(void);
void Motor_Drv_SetWheelTorque(uint8_t index, float torque);
void Motor_Drv_SetFrictionWheelTorque(uint8_t index, float torque);
void Motor_Drv_SetGimbalAngle(uint8_t index, float angle);
Wheel_Motor_t *Motor_Drv_GetWheelMotor(uint8_t index);
Wheel_Motor_t *Motor_Drv_GetFrictionWheelMotor(uint8_t index);
Wheel_Motor_t *Motor_Drv_GetGimbalMotor(uint8_t index);

extern Wheel_Motor_t wheel_motors[4];
extern Wheel_Motor_t friction_wheels[2];
extern Wheel_Motor_t gimbal_motors[2];

#endif