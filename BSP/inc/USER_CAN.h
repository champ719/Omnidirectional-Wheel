#ifndef USER_CAN_H
#define USER_CAN_H

#include "main.h"
#include "can.h"

void CAN_Init(void);
HAL_StatusTypeDef CAN_Send_DM_Motor_Data(CAN_HandleTypeDef *hcan,
                                         uint16_t std_id,
                                         uint8_t *data);

#endif
