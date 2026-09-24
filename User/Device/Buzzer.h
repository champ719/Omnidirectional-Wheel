#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"

/* 初始化蜂鸣器 PWM。 */
void Buzzer_Init(void);
/* 每 2 ms 推进一次蜂鸣器状态机。 */
void Buzzer_Update_2ms(void);
/* 播放急停双响提示。 */
void Buzzer_PlayEmergencyDoubleBeep(void);

#endif
