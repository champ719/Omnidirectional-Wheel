#ifndef USER_CAN_H
#define USER_CAN_H

#include "main.h"
#include "can.h"

void CAN_Init(void);
void CAN_Send_DM_Motor_Data(CAN_HandleTypeDef *hcan, int16_t StdId, uint8_t *Data);

extern uint16_t can_send_error, can_receive_error;

#endif
