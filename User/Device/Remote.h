#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

#define RC_FRAME_LENGTH   18U
#define RC_RX_BUF_LENGTH  36U

#define RC_SW_UP          1U
#define RC_SW_MID         3U
#define RC_SW_DOWN        2U

#define RC_CH_VALUE_MID   1024

#define RC_KEY_W          (1U << 0)
#define RC_KEY_S          (1U << 1)
#define RC_KEY_A          (1U << 2)
#define RC_KEY_D          (1U << 3)
#define RC_KEY_SHIFT      (1U << 4)
#define RC_KEY_CTRL       (1U << 5)
#define RC_KEY_Q          (1U << 6)
#define RC_KEY_E          (1U << 7)

typedef struct
{
    struct
    {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        int16_t ch4;
        uint8_t s1;
        uint8_t s2;
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
        uint16_t v;
    } key;

    uint32_t update_tick;
    uint32_t update_sequence;
} RC_Ctrl_t;

/* Latest decoded remote-control data; exposed globally for debugging. */
extern volatile RC_Ctrl_t rc_ctrl;

/* 摇杆/键鼠控制方式：由 Task_RC_Callback 按拨轮 ±600 置位，各控制模块直读。
   1 = 摇杆控制，0 = 键鼠控制。 */
extern volatile uint8_t Rocker_Ctrl;

void Remote_Init(void);
uint8_t Remote_IsOnline(void);
float Remote_NormalizeChannel(int16_t value);
void Remote_GetSnapshot(RC_Ctrl_t *snapshot);

/* 遥控输入层任务入口。轮询按键、按拨轮切换控制方式、右拨杆急停，并刷新故障监控。 */
void Task_RC_Callback(void);
void OS_RcCallback(void const *argument);

#endif
