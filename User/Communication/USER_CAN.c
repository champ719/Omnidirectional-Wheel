#include "USER_CAN.h"
#include "chassis.h"
#include "gimbal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define CAN_ERROR_NOTIFICATIONS \
    (CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_ERROR_WARNING | \
     CAN_IT_ERROR_PASSIVE | CAN_IT_LAST_ERROR_CODE)
#define CAN_CONTROL_TX_SLOT_COUNT 3U

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint32_t std_id;
    uint8_t data[8];
    uint32_t sequence;
    uint8_t pending;
} CAN_ControlTxSlot_t;

CAN_Diagnostics_t can1_diagnostics;
CAN_Diagnostics_t can2_diagnostics;

/* 控制帧只有三个固定组合。每个槽只保存对应 ID 的最新值，新的控制量会
   覆盖尚未进入硬件邮箱的旧值，不形成软件 FIFO。 */
static CAN_ControlTxSlot_t can_control_tx_slots[CAN_CONTROL_TX_SLOT_COUNT];
static uint8_t can_service_active;
static uint8_t can1_tx_scan_start;
static uint8_t can2_tx_scan_start;

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

static CAN_ControlTxSlot_t *CAN_GetControlTxSlot(CAN_HandleTypeDef *hcan,
                                                  uint32_t std_id)
{
    uint32_t index;

    for (index = 0U; index < CAN_CONTROL_TX_SLOT_COUNT; index++) {
        if ((can_control_tx_slots[index].hcan == hcan) &&
            (can_control_tx_slots[index].std_id == std_id)) {
            return &can_control_tx_slots[index];
        }
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

    if (motor->motor_type == DJI_3508) {
        motor->fb_angle =
            (float)(((uint16_t)rx_data[0] << 8U) | rx_data[1]) /
            GEAR_RATE_3508 / 8192.0f * 2.0f * 3.1415f;
        motor->fb_speed =
            (float)((int16_t)(((uint16_t)rx_data[2] << 8U) | rx_data[3])) /
            GEAR_RATE_3508 * 2.0f * 3.1415f / 60.0f;
        motor->fb_torque = motor->fb_current * K_TORQUE_3508;
    } else if (motor->motor_type == DJI_6020) {
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
    half_range = (motor->motor_type == DJI_3508) ?
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
            chassis.power_fb.update_tick = HAL_GetTick();
            chassis.power_fb.received = 1U;
            /* 序号最后更新，任务看到新序号时本帧数据已完整。 */
            chassis.power_fb.update_sequence++;
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
    memset(can_control_tx_slots, 0, sizeof(can_control_tx_slots));
    can_service_active = 0U;
    can1_tx_scan_start = 0U;
    can2_tx_scan_start = 0U;

    can_control_tx_slots[0].hcan = &hcan1;
    can_control_tx_slots[0].std_id = 0x200U;
    can_control_tx_slots[1].hcan = &hcan1;
    can_control_tx_slots[1].std_id = 0x1FFU;
    can_control_tx_slots[2].hcan = &hcan2;
    can_control_tx_slots[2].std_id = 0x1FFU;

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
            /* HAL明确报告接收失败，由任务上下文完成控制器重启。 */
            diagnostics->recovery_pending = 1U;
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
    CAN_Diagnostics_t *diagnostics = CAN_GetDiagnostics(hcan);
    CAN_ControlTxSlot_t *slot;

    if ((hcan == NULL) || (motor == NULL) || (diagnostics == NULL)) {
        return HAL_ERROR;
    }

    slot = CAN_GetControlTxSlot(hcan, motor->cmd_id);
    if (slot == NULL) {
        diagnostics->tx_error_count++;
        diagnostics->last_tx_status = HAL_ERROR;
        return HAL_ERROR;
    }

    taskENTER_CRITICAL();
    if (slot->pending != 0U) {
        diagnostics->tx_overwrite_count++;
    }
    slot->data[0] = (uint8_t)(iq1 >> 8U);
    slot->data[1] = (uint8_t)iq1;
    slot->data[2] = (uint8_t)(iq2 >> 8U);
    slot->data[3] = (uint8_t)iq2;
    slot->data[4] = (uint8_t)(iq3 >> 8U);
    slot->data[5] = (uint8_t)iq3;
    slot->data[6] = (uint8_t)(iq4 >> 8U);
    slot->data[7] = (uint8_t)iq4;
    slot->sequence++;
    slot->pending = 1U;
    taskEXIT_CRITICAL();

    diagnostics->last_tx_status = HAL_OK;
    return HAL_OK;
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
    uint8_t *scan_start;
    uint8_t first_slot;
    uint32_t offset;

    scan_start = (hcan == &hcan1) ?
        &can1_tx_scan_start : &can2_tx_scan_start;
    first_slot = *scan_start;

    if (diagnostics->recovery_pending != 0U) {
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
        } else {
            return;
        }
    }

    for (offset = 0U; offset < CAN_CONTROL_TX_SLOT_COUNT; offset++) {
        uint32_t index = ((uint32_t)first_slot + offset) %
                         CAN_CONTROL_TX_SLOT_COUNT;
        CAN_ControlTxSlot_t *slot = &can_control_tx_slots[index];
        CAN_TxHeaderTypeDef tx_header = {0};
        HAL_StatusTypeDef status;
        uint32_t sequence;
        uint32_t tx_mailbox;
        uint8_t tx_data[8];

        if ((slot->hcan != hcan) || (slot->pending == 0U)) {
            continue;
        }

        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
            diagnostics->last_tx_status = HAL_BUSY;
            diagnostics->tx_congestion_count++;
            return;
        }

        taskENTER_CRITICAL();
        sequence = slot->sequence;
        memcpy(tx_data, slot->data, sizeof(tx_data));
        taskEXIT_CRITICAL();

        tx_header.StdId = slot->std_id;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 8U;
        tx_header.TransmitGlobalTime = DISABLE;

        status = HAL_CAN_AddTxMessage(
            hcan, &tx_header, tx_data, &tx_mailbox);
        diagnostics->last_tx_status = status;

        if (status == HAL_OK) {
            taskENTER_CRITICAL();
            if (slot->sequence == sequence) {
                slot->pending = 0U;
            }
            taskEXIT_CRITICAL();
            *scan_start = (uint8_t)((index + 1U) %
                                    CAN_CONTROL_TX_SLOT_COUNT);
            diagnostics->consecutive_tx_errors = 0U;
            continue;
        }

        if (status == HAL_BUSY) {
            diagnostics->tx_congestion_count++;
            return;
        }

        /* HAL_ERROR/HAL_TIMEOUT属于明确的HAL发送错误，才请求重启。 */
        diagnostics->tx_error_count++;
        diagnostics->last_error = HAL_CAN_GetError(hcan);
        if (diagnostics->consecutive_tx_errors < UINT8_MAX) {
            diagnostics->consecutive_tx_errors++;
        }
        diagnostics->recovery_pending = 1U;
        return;
    }
}

void CAN_Service(void)
{
    taskENTER_CRITICAL();
    if (can_service_active != 0U) {
        taskEXIT_CRITICAL();
        return;
    }
    can_service_active = 1U;
    taskEXIT_CRITICAL();

    CAN_ServiceController(&hcan1, &can1_diagnostics);
    CAN_ServiceController(&hcan2, &can2_diagnostics);

    taskENTER_CRITICAL();
    can_service_active = 0U;
    taskEXIT_CRITICAL();
}

uint8_t CAN_IsHealthy(void)
{
    return ((can1_diagnostics.recovery_pending == 0U) &&
            (can2_diagnostics.recovery_pending == 0U) &&
            (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING) &&
            (HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_LISTENING)) ? 1U : 0U;
}
