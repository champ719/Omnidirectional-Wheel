#ifndef USER_CAN_H
#define USER_CAN_H

#include "main.h"
#include "can.h"
#include "motor.h"

typedef struct
{
    volatile uint32_t tx_error_count;
    volatile uint32_t tx_congestion_count;
    volatile uint32_t tx_overwrite_count;
    volatile uint32_t rx_error_count;
    volatile uint32_t bus_off_count;
    volatile uint32_t restart_count;
    volatile uint32_t last_error;
    volatile HAL_StatusTypeDef last_tx_status;
    volatile uint8_t consecutive_tx_errors;
    volatile uint8_t recovery_pending;
} CAN_Diagnostics_t;

extern CAN_Diagnostics_t can1_diagnostics;
extern CAN_Diagnostics_t can2_diagnostics;

/* 初始化 CAN 控制器及发送缓存。 */
void CAN_Init(void);
/* 维护 CAN 恢复和待发送控制帧。 */
void CAN_Service(void);
/* 判断两个 CAN 控制器是否健康。 */
uint8_t CAN_IsHealthy(void);
/* 缓存一组待发送的电机控制电流。 */
HAL_StatusTypeDef CAN_SendMessage(CAN_HandleTypeDef *hcan, volatile Motor_t *motor, uint16_t iq1, uint16_t iq2, uint16_t iq3, uint16_t iq4);

#endif
