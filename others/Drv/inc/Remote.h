#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

#define RC_FRAME_LENGTH   18U   /* DR16 每帧固定 18 字节 */
#define RC_RX_BUF_LENGTH  36U   /* 接收缓冲取 2 帧，保证 IDLE 稳定触发 */

/* 拨杆位置：DJI 遥控器 s1/s2 三挡 */
#define RC_SW_UP          1U
#define RC_SW_MID         3U
#define RC_SW_DOWN        2U

/* 摇杆通道范围：364 ~ 1684，中值 1024 */
#define RC_CH_VALUE_MIN   364
#define RC_CH_VALUE_MID   1024
#define RC_CH_VALUE_MAX   1684

typedef struct
{
    struct
    {
        int16_t ch0;   /* 右摇杆 左右 */
        int16_t ch1;   /* 右摇杆 上下 */
        int16_t ch2;   /* 左摇杆 左右 */
        int16_t ch3;   /* 左摇杆 上下 */
        int16_t ch4;   /* 左侧拨轮 */
        uint8_t s1;    /* 左拨杆 */
        uint8_t s2;    /* 右拨杆 */
    } rc;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct
    {
        uint16_t v;    /* 键盘按键位掩码 */
    } key;

    uint32_t update_tick;   /* 最近一次成功解析的时间戳 */
} RC_Ctrl_t;

extern RC_Ctrl_t rc_ctrl;
extern volatile uint16_t rc_frame_count;   /* 成功解析的帧计数，调试用 */
extern volatile uint16_t rc_cb_count;      /* 回调进入次数，调试用 */
extern volatile uint16_t rc_last_size;     /* 最近一次回调收到的字节数，调试用 */
extern volatile uint16_t rc_bad_frame;     /* 校验失败丢弃的帧数 */

void Remote_Init(void);
uint8_t Remote_IsOnline(void);

#endif
