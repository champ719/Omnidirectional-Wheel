#include "Buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"

#define BUZZER_FIRST_BEEP_TICKS   50U
#define BUZZER_GAP_TICKS          40U
#define BUZZER_SECOND_BEEP_TICKS  50U

typedef enum
{
    BUZZER_STATE_IDLE = 0,
    BUZZER_STATE_FIRST_BEEP,
    BUZZER_STATE_GAP,
    BUZZER_STATE_SECOND_BEEP
} Buzzer_State_t;

static volatile Buzzer_State_t buzzer_state = BUZZER_STATE_IDLE;
static volatile uint16_t buzzer_ticks_remaining = 0U;

static void Buzzer_SetEnabled(uint8_t enabled)
{
    uint32_t compare = 0U;

    if (enabled != 0U) {
        compare = (__HAL_TIM_GET_AUTORELOAD(&htim4) + 1U) / 2U;
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, compare);
}

void Buzzer_Init(void)
{
    Buzzer_SetEnabled(0U);
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
}

void Buzzer_PlayEmergencyDoubleBeep(void)
{
    buzzer_state = BUZZER_STATE_FIRST_BEEP;
    buzzer_ticks_remaining = BUZZER_FIRST_BEEP_TICKS;
    Buzzer_SetEnabled(1U);
}

void Buzzer_Update_2ms(void)
{
    if (buzzer_state == BUZZER_STATE_IDLE) {
        return;
    }

    if (buzzer_ticks_remaining > 0U) {
        buzzer_ticks_remaining--;
    }
    if (buzzer_ticks_remaining > 0U) {
        return;
    }

    switch (buzzer_state) {
    case BUZZER_STATE_FIRST_BEEP:
        buzzer_state = BUZZER_STATE_GAP;
        buzzer_ticks_remaining = BUZZER_GAP_TICKS;
        Buzzer_SetEnabled(0U);
        break;

    case BUZZER_STATE_GAP:
        buzzer_state = BUZZER_STATE_SECOND_BEEP;
        buzzer_ticks_remaining = BUZZER_SECOND_BEEP_TICKS;
        Buzzer_SetEnabled(1U);
        break;

    case BUZZER_STATE_SECOND_BEEP:
    default:
        buzzer_state = BUZZER_STATE_IDLE;
        buzzer_ticks_remaining = 0U;
        Buzzer_SetEnabled(0U);
        break;
    }
}

void OS_BeepCallback(void const *argument)
{
    TickType_t last_wake;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        Buzzer_Update_2ms();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2U));
    }
}
