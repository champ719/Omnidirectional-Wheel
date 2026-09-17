#include "Slope.h"

#include <math.h>
#include <stddef.h>

void Slope_Init(volatile Slope *slope, float step, float deadzone)
{
    if (slope == NULL) {
        return;
    }

    slope->step = fabsf(step);
    slope->deadzone = fabsf(deadzone);
    Slope_Reset(slope, 0.0f);
}

void Slope_Reset(volatile Slope *slope, float value)
{
    if (slope == NULL) {
        return;
    }

    slope->target = value;
    slope->value = value;
}

void Slope_SetTarget(volatile Slope *slope, float target)
{
    if (slope != NULL) {
        slope->target = target;
    }
}

void Slope_SetStep(volatile Slope *slope, float step)
{
    if (slope != NULL) {
        slope->step = fabsf(step);
    }
}

float Slope_NextVal(volatile Slope *slope)
{
    float error;

    if (slope == NULL) {
        return 0.0f;
    }

    error = slope->target - slope->value;
    if ((fabsf(error) <= slope->deadzone) ||
        (fabsf(error) <= slope->step)) {
        slope->value = slope->target;
    } else if (error > 0.0f) {
        slope->value += slope->step;
    } else if (error < 0.0f) {
        slope->value -= slope->step;
    }

    return slope->value;
}

float Slope_GetVal(const volatile Slope *slope)
{
    return (slope != NULL) ? slope->value : 0.0f;
}
