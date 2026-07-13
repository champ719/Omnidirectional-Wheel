#include "USER_CAN.h"
#include "can.h"
#include "Motor_Drv.h"

uint16_t can_send_error = 0;
uint16_t can_receive_error = 0;
uint16_t can1_send_error = 0;
uint16_t can2_send_error = 0;
uint16_t can1_rx_count = 0;
uint16_t can2_rx_count = 0;

/* Diagnostics: snapshot of the error status register at the moment a send fails */
uint32_t can1_last_esr = 0;   /* full ESR value */
uint32_t can2_last_esr = 0;
uint8_t  can1_last_lec = 0;   /* bit[6:4] of ESR: last error code */
uint8_t  can2_last_lec = 0;
uint8_t  can1_lec_seen = 0;   /* bitmask: which LEC codes have occurred */
uint8_t  can2_lec_seen = 0;

static CAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];

extern Wheel_Motor_t wheel_motors[4];
extern Wheel_Motor_t friction_wheels[2];
extern Wheel_Motor_t gimbal_motors[2];

static void CAN_RouteMessage(CAN_HandleTypeDef *hcan, uint32_t stdId)
{
    if (hcan == &hcan1) {
        switch (stdId) {
            case 0x204:
                DJI3508_Get_Data(rx_data, &wheel_motors[0]);
                break;
            case 0x203:
                DJI3508_Get_Data(rx_data, &wheel_motors[1]);
                break;
            case 0x202:
                DJI3508_Get_Data(rx_data, &wheel_motors[2]);
                break;
            case 0x201:
                DJI3508_Get_Data(rx_data, &wheel_motors[3]);
                break;
            case 0x205:
                GM6020_Get_Data(rx_data, &gimbal_motors[0]);
                break;
            default:
                break;
        }
    } else if (hcan == &hcan2) {
        switch (stdId) {
            case 0x201:
                DJI3508_Get_Data(rx_data, &friction_wheels[0]);
                break;
            case 0x202:
                DJI3508_Get_Data(rx_data, &friction_wheels[1]);
                break;
            case 0x206:
                GM6020_Get_Data(rx_data, &gimbal_motors[1]);
                break;
            default:
                break;
        }
    }
}

void CAN_Init(void)
{
    CAN_FilterTypeDef filter;

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        can_receive_error++;
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        can_receive_error++;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        can_receive_error++;
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        can_receive_error++;
    }

    filter.FilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK) {
        can_receive_error++;
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        can_receive_error++;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0U) {
        can_receive_error++;
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        can_receive_error++;
    }

    Motor_Drv_Init();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        if (hcan == &hcan1) {
            can1_rx_count++;
        } else if (hcan == &hcan2) {
            can2_rx_count++;
        }
        if ((rx_header.IDE == CAN_ID_STD) && (rx_header.RTR == CAN_RTR_DATA)) {
            CAN_RouteMessage(hcan, rx_header.StdId);
        }
    } else {
        can_receive_error++;
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    can_receive_error++;
}

void CAN_Send_DM_Motor_Data(CAN_HandleTypeDef *hcan, int16_t StdId, uint8_t *Data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox = 0;
    HAL_StatusTypeDef status;

    tx_header.StdId = StdId;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    status = HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &tx_mailbox);
    if (status != HAL_OK) {
        uint32_t esr = hcan->Instance->ESR;
        uint8_t  lec = (uint8_t)((esr >> 4) & 0x7U);
        can_send_error++;
        if (hcan == &hcan1) {
            can1_send_error++;
            can1_last_esr = esr;
            can1_last_lec = lec;
            can1_lec_seen |= (uint8_t)(1U << lec);
        } else if (hcan == &hcan2) {
            can2_send_error++;
            can2_last_esr = esr;
            can2_last_lec = lec;
            can2_lec_seen |= (uint8_t)(1U << lec);
        }
    }
}

void CAN_Transmit(void)
{
    Motor_Drv_Send_All();
}
