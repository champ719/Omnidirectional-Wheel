#include "Motor_Drv.h"
#include "USER_CAN.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Wheel_Motor_t wheel_motors[4];
Wheel_Motor_t friction_wheels[2];
Wheel_Motor_t gimbal_motors[2];

void DM_Wheel_Motor_Init(Wheel_Motor_t *Motor, float TMAX, float PMAX, float VMAX, uint16_t motor_id)
{
    Motor->TMAX = TMAX;
    Motor->PMAX = PMAX;
    Motor->VMAX = VMAX;
    Motor->motor_id = motor_id;
    Motor->rx_timeout = 0;
    Motor->rx_received = 0;
    Motor->TX_data = 0;
    Motor->Target_Torque = 0.0f;
    Motor->Rx_Data.Position = 0;
    Motor->Rx_Data.Velocity = 0.0f;
    Motor->Rx_Data.Speed = 0;
    Motor->Rx_Data.Torque = 0;
    Motor->Rx_Data.temperate = 0;
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void Motor_Drv_Init(void)
{
    static uint8_t initialized = 0;
    if (initialized) {
        return;
    }

    /* Wheel motors: front-left/front-right/rear-left/rear-right */
    DM_Wheel_Motor_Init(&wheel_motors[0], 20.0f, 20000.0f, 30000.0f, 0x204); /* FL */
    DM_Wheel_Motor_Init(&wheel_motors[1], 20.0f, 20000.0f, 30000.0f, 0x203); /* FR */
    DM_Wheel_Motor_Init(&wheel_motors[2], 20.0f, 20000.0f, 30000.0f, 0x202); /* RL */
    DM_Wheel_Motor_Init(&wheel_motors[3], 20.0f, 20000.0f, 30000.0f, 0x201); /* RR */

    /* Friction wheels on CAN2 */
    DM_Wheel_Motor_Init(&friction_wheels[0], 20.0f, 20000.0f, 30000.0f, 0x201);
    DM_Wheel_Motor_Init(&friction_wheels[1], 20.0f, 20000.0f, 30000.0f, 0x202);

    /* Gimbal motors: yaw and pitch */
    DM_Wheel_Motor_Init(&gimbal_motors[0], 20.0f, 20000.0f, 30000.0f, 0x205); /* yaw */
    DM_Wheel_Motor_Init(&gimbal_motors[1], 20.0f, 20000.0f, 30000.0f, 0x206); /* pitch */

    initialized = 1;
}

void DJI3508_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor)
{
    Motor->Rx_Data.Position = (uint16_t)(Data[0] << 8 | Data[1]);
    Motor->Rx_Data.Velocity = ((((int16_t)(Data[2] << 8 | Data[3])) / 60.0f) * 2.0f * M_PI) / 14.88f;
    Motor->Rx_Data.Speed = (int16_t)(Data[2] << 8 | Data[3]);
    Motor->Rx_Data.Torque = (int16_t)(Data[4] << 8 | Data[5]);
    Motor->Rx_Data.temperate = Data[6];
    Motor->rx_timeout = 0;
    Motor->rx_received = 1;
}

void GM6020_Get_Data(uint8_t *Data, Wheel_Motor_t *Motor)
{
    DJI3508_Get_Data(Data, Motor);
}

/* Send one 0x200 frame carrying up to 4 motors (IDs 0x201-0x204).
   slot = motor_id - 0x201, each slot occupies 2 bytes. */
static void DJI_SendGroup(CAN_HandleTypeDef *hcan,
                           Wheel_Motor_t *motors, uint8_t count,
                           uint16_t base_id, uint16_t frame_id)
{
    uint8_t data[8] = {0};
    uint8_t i;
    int16_t tq;
    uint8_t slot;

    for (i = 0; i < count; i++) {
        tq = (int16_t)(motors[i].Target_Torque * 100.0f);
        if (tq >  16384) tq =  16384;
        if (tq < -16384) tq = -16384;
        slot = (uint8_t)(motors[i].motor_id - base_id);
        if (slot < 4) {
            data[slot * 2]     = (uint8_t)(tq >> 8);
            data[slot * 2 + 1] = (uint8_t)tq;
        }
    }
    CAN_Send_DM_Motor_Data(hcan, frame_id, data);
}

void Motor_Drv_Send_All(void)
{
    Motor_Drv_Init();

    /* CAN1 wheel motors 0x201-0x204 → one frame to 0x200 */
    DJI_SendGroup(&hcan1, wheel_motors, 4, 0x201, 0x200);

    /* CAN2 friction wheels 0x201-0x202 → one frame to 0x200 */
    DJI_SendGroup(&hcan2, friction_wheels, 2, 0x201, 0x200);

    /* CAN1 gimbal yaw 0x205 → one frame to 0x1FF */
    DJI_SendGroup(&hcan1, &gimbal_motors[0], 1, 0x205, 0x1FF);

    /* CAN2 gimbal pitch 0x206 → one frame to 0x1FF */
    DJI_SendGroup(&hcan2, &gimbal_motors[1], 1, 0x205, 0x1FF);
}

void Motor_Drv_SetWheelTorque(uint8_t index, float torque)
{
    if (index < 4) {
        wheel_motors[index].Target_Torque = torque;
    }
}

void Motor_Drv_SetFrictionWheelTorque(uint8_t index, float torque)
{
    if (index < 2) {
        friction_wheels[index].Target_Torque = torque;
    }
}

void Motor_Drv_SetGimbalAngle(uint8_t index, float angle)
{
    if (index < 2) {
        gimbal_motors[index].Target_Torque = angle;
    }
}

Wheel_Motor_t *Motor_Drv_GetWheelMotor(uint8_t index)
{
    if (index < 4) {
        return &wheel_motors[index];
    }
    return NULL;
}

Wheel_Motor_t *Motor_Drv_GetFrictionWheelMotor(uint8_t index)
{
    if (index < 2) {
        return &friction_wheels[index];
    }
    return NULL;
}

Wheel_Motor_t *Motor_Drv_GetGimbalMotor(uint8_t index)
{
    if (index < 2) {
        return &gimbal_motors[index];
    }
    return NULL;
}

