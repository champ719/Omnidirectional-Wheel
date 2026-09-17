#ifndef SLOPE_H
#define SLOPE_H

typedef struct
{
    float target;
    float step;
    float value;
    float deadzone;
} Slope;

void Slope_Init(volatile Slope *slope, float step, float deadzone);
void Slope_Reset(volatile Slope *slope, float value);
void Slope_SetTarget(volatile Slope *slope, float target);
void Slope_SetStep(volatile Slope *slope, float step);
float Slope_NextVal(volatile Slope *slope);
float Slope_GetVal(const volatile Slope *slope);

#endif
