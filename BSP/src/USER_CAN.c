#include "USER_CAN.h"
#include "Motor_Drv.h"

static void CAN_RouteMessage(CAN_HandleTypeDef *hcan,
                             uint32_t std_id,
                             uint8_t *rx_data)
{
    if (hcan == &hcan1) {
        switch (std_id) {
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
        switch (std_id) {
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
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(
            &hcan1,
            CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    filter.FilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(
            &hcan2,
            CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan,
                             CAN_RX_FIFO0,
                             &rx_header,
                             rx_data) != HAL_OK) {
        return;
    }

    if ((rx_header.IDE == CAN_ID_STD) &&
        (rx_header.RTR == CAN_RTR_DATA)) {
        CAN_RouteMessage(hcan, rx_header.StdId, rx_data);
    }
}

HAL_StatusTypeDef CAN_Send_DM_Motor_Data(CAN_HandleTypeDef *hcan,
                                         uint16_t std_id,
                                         uint8_t *data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox = 0U;

    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(hcan,
                                &tx_header,
                                data,
                                &tx_mailbox);
}
