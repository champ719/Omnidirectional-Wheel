#include "Remote.h"
#include "usart.h"
#include <string.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

#define RC_ONLINE_TIMEOUT_MS  100U

RC_Ctrl_t rc_ctrl;
volatile uint16_t rc_frame_count = 0;
volatile uint16_t rc_cb_count = 0;
volatile uint16_t rc_last_size = 0;
volatile uint16_t rc_bad_frame = 0;

/* 接收缓冲取 2 帧长度。IDLE 事件在总线空闲时触发，缓冲大于一帧才能稳定
   按“空闲”而不是“收满”结束，避免与下一帧粘连错位。 */
static uint8_t rc_rx_buf[RC_RX_BUF_LENGTH];

/* 解析一整帧（18 字节）。拨杆值非法则判为坏帧，返回 0 不写入。 */
static uint8_t Remote_DecodeFrame(const uint8_t *f)
{
    uint8_t s1 = (uint8_t)((f[5] >> 4) & 0x03U);
    uint8_t s2 = (uint8_t)((f[5] >> 6) & 0x03U);

    /* DR16 上电初期/干扰会产生 s=0 的坏帧，直接丢弃 */
    if ((s1 == 0U) || (s2 == 0U)) {
        return 0U;
    }

    rc_ctrl.rc.ch0 = (int16_t)((f[0] | (f[1] << 8)) & 0x07FF) - RC_CH_VALUE_MID;
    rc_ctrl.rc.ch1 = (int16_t)(((f[1] >> 3) | (f[2] << 5)) & 0x07FF) - RC_CH_VALUE_MID;
    rc_ctrl.rc.ch2 = (int16_t)(((f[2] >> 6) | (f[3] << 2) | (f[4] << 10)) & 0x07FF) - RC_CH_VALUE_MID;
    rc_ctrl.rc.ch3 = (int16_t)(((f[4] >> 1) | (f[5] << 7)) & 0x07FF) - RC_CH_VALUE_MID;

    rc_ctrl.rc.s1 = s1;
    rc_ctrl.rc.s2 = s2;

    rc_ctrl.mouse.x = (int16_t)(f[6]  | (f[7]  << 8));
    rc_ctrl.mouse.y = (int16_t)(f[8]  | (f[9]  << 8));
    rc_ctrl.mouse.z = (int16_t)(f[10] | (f[11] << 8));
    rc_ctrl.mouse.press_l = f[12];
    rc_ctrl.mouse.press_r = f[13];

    rc_ctrl.key.v = (uint16_t)(f[14] | (f[15] << 8));

    rc_ctrl.rc.ch4 = (int16_t)((f[16] | (f[17] << 8)) & 0x07FF) - RC_CH_VALUE_MID;

    rc_ctrl.update_tick = HAL_GetTick();
    return 1U;
}

void Remote_Init(void)
{
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rx_buf, RC_RX_BUF_LENGTH);

    /* 关掉 DMA 半传输中断，只在整帧空闲/完成时处理 */
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

uint8_t Remote_IsOnline(void)
{
    return ((HAL_GetTick() - rc_ctrl.update_tick) <= RC_ONLINE_TIMEOUT_MS) ? 1U : 0U;
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

    rc_cb_count++;
    rc_last_size = Size;

    /* 从收到的数据里，取最后一个完整的 18 字节帧解析。
       正常每次空闲收到 1 帧 = 18 字节；若粘连收到多帧，取最新那帧。 */
    if (Size >= RC_FRAME_LENGTH) {
        uint16_t offset = Size - (Size % RC_FRAME_LENGTH) - RC_FRAME_LENGTH;
        if (Remote_DecodeFrame(&rc_rx_buf[offset]) != 0U) {
            rc_frame_count++;
        } else {
            rc_bad_frame++;
        }
    } else {
        rc_bad_frame++;
    }

    /* 重新武装接收，等待下一帧 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rx_buf, RC_RX_BUF_LENGTH);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}
