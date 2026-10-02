#include "USER_CAN.h"
#include "chassis.h"
#include "gimbal.h"
#include "FreeRTOS.h"
#include "task.h"

volatile CanState_t can_state = {0};

void CAN1_Init(void)
{
  CAN_FilterTypeDef filter = {0};

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

  can_state.can1_init_error = HAL_CAN_ConfigFilter(&hcan1, &filter);
  if (can_state.can1_init_error != HAL_OK)
  {
    Error_Handler();
  }

  can_state.can1_init_error = HAL_CAN_ActivateNotification(&hcan1,
                                                           CAN_IT_RX_FIFO0_MSG_PENDING);
  if (can_state.can1_init_error != HAL_OK)
  {
    Error_Handler();
  }

  can_state.can1_init_error = HAL_CAN_Start(&hcan1);
  if (can_state.can1_init_error != HAL_OK)
  {
    Error_Handler();
  }
}

void CAN2_Init(void)
{
  CAN_FilterTypeDef filter = {0};

  /* bxCAN 的两路控制器共用过滤器组，CAN2 使用分界后的 bank。 */
  filter.FilterBank = 14;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  can_state.can2_init_error = HAL_CAN_ConfigFilter(&hcan2, &filter);
  if (can_state.can2_init_error != HAL_OK)
  {
    Error_Handler();
  }

  can_state.can2_init_error = HAL_CAN_ActivateNotification(&hcan2,
                                                           CAN_IT_RX_FIFO0_MSG_PENDING);
  if (can_state.can2_init_error != HAL_OK)
  {
    Error_Handler();
  }

  can_state.can2_init_error = HAL_CAN_Start(&hcan2);
  if (can_state.can2_init_error != HAL_OK)
  {
    Error_Handler();
  }
}

void FeedbackTrans(volatile Motor_t *motor, const uint8_t *rxdata)
{
  motor->last_angle = motor->fb_angle;
  motor->fb_current = (float)((int16_t)(((uint16_t)rxdata[4] << 8) | (uint16_t)rxdata[5])) / 16384.0f * 20.0f;
  motor->fb_temp = rxdata[6];
  motor->feedback_tick = HAL_GetTick();
  motor->feedback_received = 1U;

  if(motor->motor_type == DJI_3508) {
    motor->fb_angle = (float)(((uint16_t)rxdata[0] << 8) | (uint16_t)rxdata[1]) / GEAR_RATE_3508 / 8192.0f * 2.0f * 3.1415f;
    motor->fb_speed = (float)((int16_t)(((uint16_t)rxdata[2] << 8) | (uint16_t)rxdata[3])) / GEAR_RATE_3508 * 2.0f * 3.1415f / 60.0f;
    motor->fb_torque = motor->fb_current * K_TORQUE_3508;
  }
  if(motor->motor_type == DJI_6020) {
    motor->fb_angle = (float)(((uint16_t)rxdata[0] << 8) | (uint16_t)rxdata[1]) / 8192.0f * 2.0f * 3.1415f;
    motor->fb_speed = (float)((int16_t)(((uint16_t)rxdata[2] << 8) | (uint16_t)rxdata[3])) * 2.0f * 3.1415f / 60.0f;
    motor->fb_torque = motor->fb_current * K_TORQUE_6020;
  }

  /* 3508 和 6020 的单圈机械角范围不同。 */
  if (motor->total_angle == 0.0f && motor->last_angle == 0.0f)
  {
    motor->total_angle = motor->fb_angle;
  }
  else
  {
    float change = motor->fb_angle - motor->last_angle;
    float half_range = (motor->motor_type == DJI_3508) ? (3.14159265f / GEAR_RATE_3508) : 3.14159265f;
    float full_range = half_range * 2.0f;
    if(change >  half_range)
      change -= full_range;
    else if (change < -half_range)
      change += full_range;
    motor->total_angle += change;
  }
  while (motor->total_angle >  3.14159265f) motor->total_angle -= 6.28318531f;
  while (motor->total_angle < -3.14159265f) motor->total_angle += 6.28318531f;
}

void CAN1_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, const uint8_t *rxdata)
{
  switch (rx_header->StdId)
  {

    case 0x201:
      FeedbackTrans((volatile Motor_t *)&chassis.motor_3508[LF], rxdata);
      break;
    case 0x202:
      FeedbackTrans((volatile Motor_t *)&chassis.motor_3508[RF], rxdata);
      break;
    case 0x203:
      FeedbackTrans((volatile Motor_t *)&chassis.motor_3508[LB], rxdata);
      break;
    case 0x204:
      FeedbackTrans((volatile Motor_t *)&chassis.motor_3508[RB], rxdata);
      break;
    case 0x205:
      FeedbackTrans((volatile Motor_t *)&gimbal.yaw_motor, rxdata);
      break;
		case 0x213:
      chassis.power_fb.voltage = (float)((int16_t)(((uint16_t)rxdata[1] << 8) | (uint16_t)rxdata[0])) / 100.0f;
      chassis.power_fb.current = (float)((int16_t)(((uint16_t)rxdata[3] << 8) | (uint16_t)rxdata[2])) / 100.0f;
      chassis.power_fb.power = 0.8f * chassis.power_fb.voltage * chassis.power_fb.current + 0.2f * chassis.power_fb.power;
      chassis.power_fb.update_tick = HAL_GetTick();
      chassis.power_fb.received = 1U;
      chassis.power_fb.update_sequence++;
      break;
    default:
      break;
  }
}

void CAN2_Rx0Callback(CAN_RxHeaderTypeDef *rx_header, const uint8_t *rxdata)
{
  switch (rx_header->StdId)
  {
    case 0x206:
      FeedbackTrans((volatile Motor_t *)&gimbal.pitch_motor, rxdata);
      break;
    default:
      break;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_StatusTypeDef if_can_get_message_ok;
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if(hcan == &hcan1)
  {
    if_can_get_message_ok = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (if_can_get_message_ok == HAL_OK)
    {
      CAN1_Rx0Callback(&rx_header, rx_data);
    }
    else
    {
      can_state.can1_receive_error++;
    }
  }

  if (hcan == &hcan2)
  {
   if_can_get_message_ok = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (if_can_get_message_ok == HAL_OK)
    {
      CAN2_Rx0Callback(&rx_header, rx_data);
    }
    else
    {
      can_state.can2_receive_error++;
    }
  }

}
void CAN_SendMessage(CAN_HandleTypeDef *hcan, volatile Motor_t *motor , int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8];

  tx_header.StdId = (uint32_t)motor->cmd_id;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[0] = (uint8_t)((uint16_t)iq1 >> 8);
  tx_data[1] = (uint8_t)(iq1 & 0xFF);
  tx_data[2] = (uint8_t)((uint16_t)iq2 >> 8);
  tx_data[3] = (uint8_t)(iq2 & 0xFF);
  tx_data[4] = (uint8_t)((uint16_t)iq3 >> 8);
  tx_data[5] = (uint8_t)(iq3 & 0xFF);
  tx_data[6] = (uint8_t)((uint16_t)iq4 >> 8);
  tx_data[7] = (uint8_t)(iq4 & 0xFF);

  uint32_t mailbox;
  taskENTER_CRITICAL();
  if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &mailbox) != HAL_OK)
  {
    can_state.can_tx_error++;
  }
  taskEXIT_CRITICAL();
}
