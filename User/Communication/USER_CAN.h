#ifndef USER_CAN_H
#define USER_CAN_H

#include "main.h"
#include "can.h"
#include "motor.h"

typedef struct
{
    volatile uint32_t tx_error_count;
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

void CAN_Init(void);
void CAN_Service(void);
uint8_t CAN_IsHealthy(void);
HAL_StatusTypeDef CAN_SendMessage(CAN_HandleTypeDef *hcan,
                                  volatile Motor_t *motor,
                                  uint16_t iq1,
                                  uint16_t iq2,
                                  uint16_t iq3,
                                  uint16_t iq4);

#endif
