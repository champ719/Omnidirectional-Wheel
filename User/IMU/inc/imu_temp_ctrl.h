#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H

#include <stdint.h>

typedef struct
{
    float q[4];
    float gyro[3];
    float accel[3];
    float temp;
    float roll;
    float pitch;
    float yaw;
    float YawTotalAngle;
} INS_t;

extern INS_t INS;

void INS_Init(void);
void INS_Task(void);
void OS_IMUCallback(void const *argument);

uint8_t IMU_Attitude_IsReady(void);
void IMU_Attitude_GetGyroBody(float gyro_body[3]);
float IMU_Attitude_GetYawContinuousRad(void);
uint32_t IMU_Attitude_GetUpdateSequence(void);

#endif
