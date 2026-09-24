#include "Remote.h"
#include "Error.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include <string.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* 超过该时长未收到有效遥控帧即判定离线，单位 ms。 */
#define RC_ONLINE_TIMEOUT_MS  100U
/* DBUS 摇杆有效满量程。 */
#define RC_CHANNEL_FULL_SCALE 660.0f
/* 摇杆归一化前使用的中心死区。 */
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
static volatile uint8_t remote_valid_frame_count;

/* 解析一整帧（18 字节）。拨杆值非法则判为坏帧，返回 0 不写入。 */
static uint8_t Remote_DecodeFrame(const uint8_t *frame)
{
    RC_Ctrl_t decoded;
    uint8_t s1 = (uint8_t)((frame[5] >> 4) & 0x03U);
    uint8_t s2 = (uint8_t)((frame[5] >> 6) & 0x03U);

    memset(&decoded, 0, sizeof(decoded));
    decoded.rc.ch0 = (int16_t)((frame[0] | (frame[1] << 8)) & 0x07FF) - RC_CH_VALUE_MID;
    decoded.rc.ch1 = (int16_t)(((frame[1] >> 3) | (frame[2] << 5)) & 0x07FF) - RC_CH_VALUE_MID;
    decoded.rc.ch2 = (int16_t)(((frame[2] >> 6) | (frame[3] << 2) | (frame[4] << 10)) & 0x07FF) - RC_CH_VALUE_MID;
    decoded.rc.ch3 = (int16_t)(((frame[4] >> 1) | (frame[5] << 7)) & 0x07FF) - RC_CH_VALUE_MID;
    decoded.rc.ch4 = (int16_t)((frame[16] | (frame[17] << 8)) & 0x07FF) - RC_CH_VALUE_MID;
    decoded.rc.s1 = s1;
    decoded.rc.s2 = s2;

    decoded.mouse.x = (int16_t)(frame[6] | (frame[7] << 8));
    decoded.mouse.y = (int16_t)(frame[8] | (frame[9] << 8));
    decoded.mouse.z = (int16_t)(frame[10] | (frame[11] << 8));
    decoded.mouse.press_l = frame[12];
    decoded.mouse.press_r = frame[13];
    decoded.key.v = (uint16_t)(frame[14] | (frame[15] << 8));

    if ((s1 == 0U) || (s2 == 0U) || (decoded.rc.ch0 < -700) || (decoded.rc.ch0 > 700) || (decoded.rc.ch1 < -700) || (decoded.rc.ch1 > 700) || (decoded.rc.ch2 < -700) || (decoded.rc.ch2 > 700) || (decoded.rc.ch3 < -700) || (decoded.rc.ch3 > 700) || (decoded.rc.ch4 < -700) || (decoded.rc.ch4 > 700) || (decoded.mouse.press_l > 1U) || (decoded.mouse.press_r > 1U)) {
        remote_valid_frame_count = 0U;
        return 0U;
    }

    decoded.update_tick = HAL_GetTick();
    decoded.update_sequence = rc_ctrl.update_sequence + 1U;

    rc_ctrl = decoded;
    if (remote_valid_frame_count < 5U) {
        remote_valid_frame_count++;
    }
    return 1U;
}

/* 初始化遥控数据并启动 DBUS 空闲接收。 */
void Remote_Init(void)
{
    rc_ctrl = (RC_Ctrl_t){0};
    remote_valid_frame_count = 0U;

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rx_buf, RC_RX_BUF_LENGTH);

    /* 关掉 DMA 半传输中断，只在整帧空闲/完成时处理 */
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

/* 判断是否已连续接收至少五帧有效数据。 */
uint8_t Remote_HasFiveValidFrames(void)
{
    return (remote_valid_frame_count >= 5U) ? 1U : 0U;
}

/* 判断四个摇杆通道是否均处于中心区域。 */
uint8_t Remote_ControlsAreCentered(const RC_Ctrl_t *remote)
{
    if (remote == NULL) {
        return 0U;
    }

    return ((remote->rc.ch0 >= -30) && (remote->rc.ch0 <= 30) && (remote->rc.ch1 >= -30) && (remote->rc.ch1 <= 30) && (remote->rc.ch2 >= -30) && (remote->rc.ch2 <= 30) && (remote->rc.ch3 >= -30) && (remote->rc.ch3 <= 30)) ? 1U : 0U;
}

/* 清空连续有效遥控帧计数。 */
void Remote_ResetValidFrameCount(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    remote_valid_frame_count = 0U;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
}

/* 根据最近有效帧时间判断遥控器是否在线。 */
uint8_t Remote_IsOnline(void)
{
    RC_Ctrl_t snapshot;

    Remote_GetSnapshot(&snapshot);
    if (snapshot.update_sequence == 0U) {
        return 0U;
    }

    return ((HAL_GetTick() - snapshot.update_tick) <= RC_ONLINE_TIMEOUT_MS) ? 1U : 0U;
}

/* 对遥控通道应用死区、限幅并归一化。 */
float Remote_NormalizeChannel(int16_t value)
{
    if ((value > -RC_CHANNEL_DEADBAND) && (value < RC_CHANNEL_DEADBAND)) {
        return 0.0f;
    }

    if (value > (int16_t)RC_CHANNEL_FULL_SCALE) {
        value = (int16_t)RC_CHANNEL_FULL_SCALE;
    } else if (value < -(int16_t)RC_CHANNEL_FULL_SCALE) {
        value = -(int16_t)RC_CHANNEL_FULL_SCALE;
    }

    return (float)value / RC_CHANNEL_FULL_SCALE;
}

/* 在临界区内复制一份一致的遥控数据快照。 */
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

    remote_valid_frame_count = 0U;
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

    if ((Size > 0U) && (Size <= RC_RX_BUF_LENGTH) && ((Size % RC_FRAME_LENGTH) == 0U)) {
        uint16_t offset;

        for (offset = 0U; offset < Size; offset += RC_FRAME_LENGTH) {
            (void)Remote_DecodeFrame(&rc_rx_buf[offset]);
        }
    } else {
        remote_valid_frame_count = 0U;
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
