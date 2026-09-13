#include "USER_CAN.h"
#include "chassis.h"
#include "gimbal.h"
#include <string.h>

#define CAN_TX_ERROR_RESTART_THRESHOLD 5U
#define CAN_ERROR_NOTIFICATIONS \
    (CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_ERROR_WARNING | \
     CAN_IT_ERROR_PASSIVE | CAN_IT_LAST_ERROR_CODE)

CAN_Diagnostics_t can1_diagnostics;
CAN_Diagnostics_t can2_diagnostics;

static CAN_Diagnostics_t *CAN_GetDiagnostics(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) {
        return &can1_diagnostics;
    }
    if (hcan == &hcan2) {
        return &can2_diagnostics;
    }
    return NULL;
}

static void FeedbackTrans(volatile Motor_t *motor, const uint8_t *rx_data)
{
    float change;
    float half_range;
    float full_range;

    motor->last_angle = motor->fb_angle;
    motor->fb_current =
        (float)((int16_t)(((uint16_t)rx_data[4] << 8U) | rx_data[5])) /
        16384.0f * 20.0f;
    motor->fb_temp = rx_data[6];
    motor->feedback_tick = HAL_GetTick();
    motor->feedback_received = 1U;

    if (motor->motor_type == TYPE_3508) {
        motor->fb_angle =
            (float)(((uint16_t)rx_data[0] << 8U) | rx_data[1]) /
            GEAR_RATE_3508 / 8192.0f * 2.0f * 3.1415f;
        motor->fb_speed =
            (float)((int16_t)(((uint16_t)rx_data[2] << 8U) | rx_data[3])) /
            GEAR_RATE_3508 * 2.0f * 3.1415f / 60.0f;
        motor->fb_torque = motor->fb_current * K_TORQUE_3508;
    } else if (motor->motor_type == TYPE_6020) {
        motor->fb_angle =
            (float)(((uint16_t)rx_data[0] << 8U) | rx_data[1]) /
            8192.0f * 2.0f * 3.1415f;
        motor->fb_speed =
            (float)((int16_t)(((uint16_t)rx_data[2] << 8U) | rx_data[3])) *
            2.0f * 3.1415f / 60.0f;
        motor->fb_torque = motor->fb_current * K_TORQUE_6020;
    }

    if ((motor->total_angle == 0.0f) && (motor->last_angle == 0.0f)) {
        motor->total_angle = motor->fb_angle;
        return;
    }

    change = motor->fb_angle - motor->last_angle;
    half_range = (motor->motor_type == TYPE_3508) ?
        (3.1415f / 19.0f) : 3.1415f;
    full_range = half_range * 2.0f;
    if (change > half_range) {
        change -= full_range;
    } else if (change < -half_range) {
        change += full_range;
    }
    motor->total_angle += change;

    while (motor->total_angle > 3.14159265f) {
        motor->total_angle -= 6.28318531f;
    }
    while (motor->total_angle < -3.14159265f) {
        motor->total_angle += 6.28318531f;
    }
}

static void CAN_RouteMessage(CAN_HandleTypeDef *hcan,
                             uint32_t std_id,
                             const uint8_t *rx_data)
{
    if (hcan == &hcan1) {
        switch (std_id) {
        case 0x201:
            FeedbackTrans(&chassis.motor_3508[LF], rx_data);
            break;
        case 0x202:
            FeedbackTrans(&chassis.motor_3508[RF], rx_data);
            break;
        case 0x203:
            FeedbackTrans(&chassis.motor_3508[LB], rx_data);
            break;
        case 0x204:
            FeedbackTrans(&chassis.motor_3508[RB], rx_data);
            break;
        case 0x205:
            FeedbackTrans(&gimbal.yaw_motor, rx_data);
            break;
        case 0x213:
            chassis.power_fb.voltage =
                (float)((int16_t)(((uint16_t)rx_data[1] << 8U) |
                                  rx_data[0])) / 100.0f;
            chassis.power_fb.current =
                (float)((int16_t)(((uint16_t)rx_data[3] << 8U) |
                                  rx_data[2])) / 100.0f;
            chassis.power_fb.power =
                0.8f * chassis.power_fb.voltage * chassis.power_fb.current +
                0.2f * chassis.power_fb.power;
            break;
        default:
            break;
        }
    } else if ((hcan == &hcan2) && (std_id == 0x206U)) {
        FeedbackTrans(&gimbal.pitch_motor, rx_data);
    }
}

void CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};

    memset(&can1_diagnostics, 0, sizeof(can1_diagnostics));
    memset(&can2_diagnostics, 0, sizeof(can2_diagnostics));

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if ((HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) ||
        (HAL_CAN_Start(&hcan1) != HAL_OK) ||
        (HAL_CAN_ActivateNotification(
            &hcan1,
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_ERROR_NOTIFICATIONS) != HAL_OK)) {
        Error_Handler();
    }

    filter.FilterBank = 14;
    if ((HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK) ||
        (HAL_CAN_Start(&hcan2) != HAL_OK) ||
        (HAL_CAN_ActivateNotification(
            &hcan2,
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_ERROR_NOTIFICATIONS) != HAL_OK)) {
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
        CAN_Diagnostics_t *diagnostics = CAN_GetDiagnostics(hcan);

        if (diagnostics != NULL) {
            diagnostics->rx_error_count++;
            diagnostics->last_error = HAL_CAN_GetError(hcan);
        }
        return;
    }

    if ((rx_header.IDE == CAN_ID_STD) &&
        (rx_header.RTR == CAN_RTR_DATA)) {
        CAN_RouteMessage(hcan, rx_header.StdId, rx_data);
    }
}

HAL_StatusTypeDef CAN_SendMessage(CAN_HandleTypeDef *hcan,
                                  volatile Motor_t *motor,
                                  uint16_t iq1,
                                  uint16_t iq2,
                                  uint16_t iq3,
                                  uint16_t iq4)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    CAN_Diagnostics_t *diagnostics = CAN_GetDiagnostics(hcan);
    HAL_StatusTypeDef status;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    if ((hcan == NULL) || (motor == NULL) || (diagnostics == NULL)) {
        return HAL_ERROR;
    }

    tx_header.StdId = motor->cmd_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_data[0] = (uint8_t)(iq1 >> 8U);
    tx_data[1] = (uint8_t)iq1;
    tx_data[2] = (uint8_t)(iq2 >> 8U);
    tx_data[3] = (uint8_t)iq2;
    tx_data[4] = (uint8_t)(iq3 >> 8U);
    tx_data[5] = (uint8_t)iq3;
    tx_data[6] = (uint8_t)(iq4 >> 8U);
    tx_data[7] = (uint8_t)iq4;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
        status = HAL_BUSY;
    } else {
        status = HAL_CAN_AddTxMessage(
            hcan, &tx_header, tx_data, &tx_mailbox);
    }

    diagnostics->last_tx_status = status;
    if (status == HAL_OK) {
        diagnostics->consecutive_tx_errors = 0U;
        return HAL_OK;
    }

    diagnostics->tx_error_count++;
    diagnostics->last_error = HAL_CAN_GetError(hcan);
    if (diagnostics->consecutive_tx_errors < UINT8_MAX) {
        diagnostics->consecutive_tx_errors++;
    }
    if ((diagnostics->consecutive_tx_errors >=
         CAN_TX_ERROR_RESTART_THRESHOLD) ||
        ((diagnostics->last_error & HAL_CAN_ERROR_BOF) != 0U)) {
        diagnostics->recovery_pending = 1U;
    }
    return status;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Diagnostics_t *diagnostics = CAN_GetDiagnostics(hcan);
    uint32_t error;

    if (diagnostics == NULL) {
        return;
    }

    error = HAL_CAN_GetError(hcan);
    diagnostics->last_error = error;
    diagnostics->rx_error_count++;
    if ((error & HAL_CAN_ERROR_BOF) != 0U) {
        diagnostics->bus_off_count++;
        diagnostics->recovery_pending = 1U;
    }
}

static void CAN_ServiceController(CAN_HandleTypeDef *hcan,
                                  CAN_Diagnostics_t *diagnostics)
{
    if (diagnostics->recovery_pending == 0U) {
        return;
    }

    if ((HAL_CAN_Stop(hcan) == HAL_OK) &&
        (HAL_CAN_ResetError(hcan) == HAL_OK) &&
        (HAL_CAN_Start(hcan) == HAL_OK) &&
        (HAL_CAN_ActivateNotification(
            hcan,
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_ERROR_NOTIFICATIONS) == HAL_OK)) {
        diagnostics->recovery_pending = 0U;
        diagnostics->consecutive_tx_errors = 0U;
        diagnostics->last_error = HAL_CAN_ERROR_NONE;
        diagnostics->restart_count++;
    }
}

void CAN_Service(void)
{
    CAN_ServiceController(&hcan1, &can1_diagnostics);
    CAN_ServiceController(&hcan2, &can2_diagnostics);
}

uint8_t CAN_IsHealthy(void)
{
    return ((can1_diagnostics.recovery_pending == 0U) &&
            (can2_diagnostics.recovery_pending == 0U) &&
            (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING) &&
            (HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_LISTENING)) ? 1U : 0U;
}
