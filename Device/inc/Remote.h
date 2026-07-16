#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

#define RC_FRAME_LENGTH   18U
#define RC_RX_BUF_LENGTH  36U

#define RC_SW_UP          1U
#define RC_SW_MID         3U
#define RC_SW_DOWN        2U

#define RC_CH_VALUE_MID   1024

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
} RC_Ctrl_t;

extern RC_Ctrl_t rc_ctrl;

void Remote_Init(void);
uint8_t Remote_IsOnline(void);
float Remote_NormalizeChannel(int16_t value);

#endif
