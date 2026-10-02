#ifndef USER_CAN_H
#define USER_CAN_H

#include "main.h"
#include "can.h"
#include "motor.h"

typedef struct
{
    HAL_StatusTypeDef can1_init_error;
    HAL_StatusTypeDef can2_init_error;
    volatile uint16_t can1_receive_error;
    volatile uint16_t can2_receive_error;
    volatile uint16_t can_tx_error;
} CanState_t;

extern volatile CanState_t can_state;

void CAN1_Init(void);
void CAN2_Init(void);
void CAN_SendMessage(CAN_HandleTypeDef *hcan, volatile Motor_t *motor,
                     int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
