#include "Remote.h"
#include "Error.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include <string.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

#define RC_ONLINE_TIMEOUT_MS  100U
#define RC_CHANNEL_FULL_SCALE 660.0f
#define RC_CHANNEL_DEADBAND   20

/* 拨轮推到此值切换控制方式（满量程 ±660） */
#define RC_WHEEL_SWITCH_THRESHOLD 600
/* 遥控输入层任务周期 */
#define RC_TASK_PERIOD_MS         15U

volatile RC_Ctrl_t rc_ctrl;
volatile uint8_t Rocker_Ctrl = 1U;

/* 接收缓冲取 2 帧长度。IDLE 事件在总线空闲时触发，缓冲大于一帧才能稳定
   按“空闲”而不是“收满”结束，避免与下一帧粘连错位。 */
static uint8_t rc_rx_buf[RC_RX_BUF_LENGTH];

/* 解析一整帧（18 字节）。拨杆值非法则判为坏帧，返回 0 不写入。 */
static uint8_t Remote_DecodeFrame(const uint8_t *f)
{
    RC_Ctrl_t decoded;
    uint8_t s1 = (uint8_t)((f[5] >> 4) & 0x03U);
    uint8_t s2 = (uint8_t)((f[5] >> 6) & 0x03U);

    /* DR16 上电初期/干扰会产生 s=0 的坏帧，直接丢弃 */
    if ((s1 == 0U) || (s2 == 0U)) {
        return 0U;
    }

    memset(&decoded, 0, sizeof(decoded));
    decoded.rc.ch0 = (int16_t)((f[0] | (f[1] << 8)) & 0x07FF) - RC_CH_VALUE_MID;
    decoded.rc.ch1 = (int16_t)(((f[1] >> 3) | (f[2] << 5)) & 0x07FF) - RC_CH_VALUE_MID;
    decoded.rc.ch2 = (int16_t)(((f[2] >> 6) | (f[3] << 2) | (f[4] << 10)) & 0x07FF) - RC_CH_VALUE_MID;
    decoded.rc.ch3 = (int16_t)(((f[4] >> 1) | (f[5] << 7)) & 0x07FF) - RC_CH_VALUE_MID;
    decoded.rc.ch4 = (int16_t)((f[16] | (f[17] << 8)) & 0x07FF) - RC_CH_VALUE_MID;
    decoded.rc.s1 = s1;
    decoded.rc.s2 = s2;

    decoded.mouse.x = (int16_t)(f[6]  | (f[7]  << 8));
    decoded.mouse.y = (int16_t)(f[8]  | (f[9]  << 8));
    decoded.mouse.z = (int16_t)(f[10] | (f[11] << 8));
    decoded.mouse.press_l = f[12];
    decoded.mouse.press_r = f[13];
    decoded.key.v = (uint16_t)(f[14] | (f[15] << 8));
    decoded.update_tick = HAL_GetTick();
    decoded.update_sequence = rc_ctrl.update_sequence + 1U;

    rc_ctrl = decoded;
    return 1U;
}

void Remote_Init(void)
{
    rc_ctrl = (RC_Ctrl_t){0};

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rx_buf, RC_RX_BUF_LENGTH);

    /* 关掉 DMA 半传输中断，只在整帧空闲/完成时处理 */
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

uint8_t Remote_IsOnline(void)
{
    RC_Ctrl_t snapshot;

    Remote_GetSnapshot(&snapshot);
    if (snapshot.update_sequence == 0U) {
        return 0U;
    }

    return ((HAL_GetTick() - snapshot.update_tick) <= RC_ONLINE_TIMEOUT_MS) ? 1U : 0U;
}

float Remote_NormalizeChannel(int16_t value)
{
    if ((value > -RC_CHANNEL_DEADBAND) &&
        (value < RC_CHANNEL_DEADBAND)) {
        return 0.0f;
    }

    if (value > (int16_t)RC_CHANNEL_FULL_SCALE) {
        value = (int16_t)RC_CHANNEL_FULL_SCALE;
    } else if (value < -(int16_t)RC_CHANNEL_FULL_SCALE) {
        value = -(int16_t)RC_CHANNEL_FULL_SCALE;
    }

    return (float)value / RC_CHANNEL_FULL_SCALE;
}

void Remote_GetSnapshot(RC_Ctrl_t *snapshot)
{
    uint32_t interrupt_state;

    if (snapshot == NULL) {
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    *snapshot = rc_ctrl;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
}

/* 接收出错时 HAL 会结束接收，这里清错误标志并重新武装，否则接收会永久停住 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) {
        return;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* 先彻底停掉再重启，避免状态机停在 BUSY 导致重启返回 HAL_BUSY */
    HAL_UART_AbortReceive(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rc_rx_buf, RC_RX_BUF_LENGTH);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

/* IDLE 或 DMA 传输完成时由 HAL 调用；Size 为本次实际收到的字节数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART3) {
        return;
    }

    /* 从收到的数据里，取最后一个完整的 18 字节帧解析。
       正常每次空闲收到 1 帧 = 18 字节；若粘连收到多帧，取最新那帧。 */
    if (Size >= RC_FRAME_LENGTH) {
        uint16_t offset = Size - (Size % RC_FRAME_LENGTH) - RC_FRAME_LENGTH;
        (void)Remote_DecodeFrame(&rc_rx_buf[offset]);
    }

    /* 重新武装接收，等待下一帧 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rx_buf, RC_RX_BUF_LENGTH);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

/************************freertos任务****************************/

/**
 * @brief 遥控输入层：切换控制方式、处理急停、刷新故障监控。
 * @note 只做输入处理，不产生运动指令。各控制模块自行调用
 *       Remote_GetSnapshot() 取本拍输入。
 */
void Task_RC_Callback(void)
{
    RC_Ctrl_t remote;

    Remote_GetSnapshot(&remote);

    /* 拨轮是自回中的，中位必须保持上一次的选择，所以只在阈值外才改写 */
    if (remote.rc.ch4 > RC_WHEEL_SWITCH_THRESHOLD) {
        Rocker_Ctrl = 1U;
    } else if (remote.rc.ch4 < -RC_WHEEL_SWITCH_THRESHOLD) {
        Rocker_Ctrl = 0U;
    }

    /* 右拨杆向下：整车急停。ErrorTask 是最高优先级，唤醒后立即抢占并持续
       发零电流，所以这里只触发起停，不直接发 Motor_STOP 以免和控制环并发发帧。 */
    if (remote.rc.s1 == RC_SW_DOWN) {
        Error_TriggerEmergencyStop();
    }

    /* 遥控、IMU、CAN、电机反馈的健康检查挂在同一个 15ms 节拍上 */
    Error_MonitorUpdate();
}

/**
 * @brief 遥控输入层任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 * @note 优先级 AboveNormal，要抢在 Chassis/Gimbal 控制环前拿到最新帧。
 */
void OS_RcCallback(void const *argument)
{
    TickType_t last_wake;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;) {
        Task_RC_Callback();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(RC_TASK_PERIOD_MS));
    }
}
