#include "Motor_Drv.h"
#include "USER_CAN.h"

#define MOTOR_DRV_PI 3.14159265358979323846f
#define MOTOR_DRV_COMMAND_MAX 163.84f

Wheel_Motor_t wheel_motors[4];
Wheel_Motor_t friction_wheels[2];
Wheel_Motor_t gimbal_motors[2];

static void Motor_Drv_InitMotor(Wheel_Motor_t *motor, uint16_t motor_id)
{
    motor->motor_id = motor_id;
    motor->Target_Torque = 0.0f;
    motor->Rx_Data.Position = 0U;
    motor->Rx_Data.Velocity = 0.0f;
    motor->Rx_Data.Speed = 0;
    motor->Rx_Data.Torque = 0;
    motor->Rx_Data.temperate = 0U;
}

void Motor_Drv_Init(void)
{
    static uint8_t initialized = 0U;

    if (initialized != 0U) {
        return;
    }

    Motor_Drv_InitMotor(&wheel_motors[0], 0x204U);
    Motor_Drv_InitMotor(&wheel_motors[1], 0x203U);
    Motor_Drv_InitMotor(&wheel_motors[2], 0x202U);
    Motor_Drv_InitMotor(&wheel_motors[3], 0x201U);
    Motor_Drv_InitMotor(&friction_wheels[0], 0x201U);
    Motor_Drv_InitMotor(&friction_wheels[1], 0x202U);
    Motor_Drv_InitMotor(&gimbal_motors[0], 0x205U);
    Motor_Drv_InitMotor(&gimbal_motors[1], 0x206U);

    initialized = 1U;
}

void Motor_Drv_StopAll(void)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        wheel_motors[i].Target_Torque = 0.0f;
    }
    for (i = 0U; i < 2U; i++) {
        friction_wheels[i].Target_Torque = 0.0f;
        gimbal_motors[i].Target_Torque = 0.0f;
    }
}

void DJI3508_Get_Data(uint8_t *data, Wheel_Motor_t *motor)
{
    motor->Rx_Data.Position =
        (uint16_t)((uint16_t)data[0] << 8U | data[1]);
    motor->Rx_Data.Velocity =
        (((float)(int16_t)((uint16_t)data[2] << 8U | data[3]) / 60.0f) *
         2.0f * MOTOR_DRV_PI) / 14.88f;
    motor->Rx_Data.Speed =
        (int16_t)((uint16_t)data[2] << 8U | data[3]);
    motor->Rx_Data.Torque =
        (int16_t)((uint16_t)data[4] << 8U | data[5]);
    motor->Rx_Data.temperate = data[6];
}

void GM6020_Get_Data(uint8_t *data, Wheel_Motor_t *motor)
{
    motor->Rx_Data.Position =
        (uint16_t)((uint16_t)data[0] << 8U | data[1]);
    motor->Rx_Data.Velocity =
        (((float)(int16_t)((uint16_t)data[2] << 8U | data[3]) / 60.0f) *
         2.0f * MOTOR_DRV_PI);
    motor->Rx_Data.Speed =
        (int16_t)((uint16_t)data[2] << 8U | data[3]);
    motor->Rx_Data.Torque =
        (int16_t)((uint16_t)data[4] << 8U | data[5]);
    motor->Rx_Data.temperate = data[6];
}

static HAL_StatusTypeDef Motor_Drv_SendGroup(CAN_HandleTypeDef *hcan,
                                             Wheel_Motor_t *motors,
                                             uint8_t count,
                                             uint16_t base_id,
                                             uint16_t frame_id)
{
    uint8_t data[8] = {0U};
    uint8_t i;

    for (i = 0U; i < count; i++) {
        float torque_command = motors[i].Target_Torque;
        int16_t torque;
        uint8_t slot =
            (uint8_t)(motors[i].motor_id - base_id);

        if (torque_command > MOTOR_DRV_COMMAND_MAX) {
            torque_command = MOTOR_DRV_COMMAND_MAX;
        } else if (torque_command < -MOTOR_DRV_COMMAND_MAX) {
            torque_command = -MOTOR_DRV_COMMAND_MAX;
        }
        torque = (int16_t)(torque_command * 100.0f);
        if (slot < 4U) {
            data[slot * 2U] = (uint8_t)(torque >> 8U);
            data[slot * 2U + 1U] = (uint8_t)torque;
        }
    }

    return CAN_Send_DM_Motor_Data(hcan, frame_id, data);
}

HAL_StatusTypeDef Motor_Drv_Send_All(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (Motor_Drv_SendGroup(&hcan1,
                            wheel_motors,
                            4U,
                            0x201U,
                            0x200U) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (Motor_Drv_SendGroup(&hcan2,
                            friction_wheels,
                            2U,
                            0x201U,
                            0x200U) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (Motor_Drv_SendGroup(&hcan1,
                            &gimbal_motors[0],
                            1U,
                            0x205U,
                            0x1FFU) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (Motor_Drv_SendGroup(&hcan2,
                            &gimbal_motors[1],
                            1U,
                            0x205U,
                            0x1FFU) != HAL_OK) {
        status = HAL_ERROR;
    }

    return status;
}

void Motor_Drv_SetWheelTorque(uint8_t index, float torque)
{
    if (index < 4U) {
        wheel_motors[index].Target_Torque = torque;
    }
}

void Motor_Drv_SetFrictionWheelTorque(uint8_t index, float torque)
{
    if (index < 2U) {
        friction_wheels[index].Target_Torque = torque;
    }
}

void Motor_Drv_SetGimbalTorque(uint8_t index, float torque)
{
    if (index < 2U) {
        gimbal_motors[index].Target_Torque = torque;
    }
}
