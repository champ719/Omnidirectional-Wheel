#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include "struct_typedef.h"
#include <stdint.h>

/*
 * BMI088 sensor-axis to robot/body-axis mapping. Adjust these six macros if
 * the control board is installed in a different orientation. The body Z axis
 * must point upward for the default yaw convention.
 */
#define IMU_BODY_X_SENSOR_INDEX              0U
#define IMU_BODY_Y_SENSOR_INDEX              1U
#define IMU_BODY_Z_SENSOR_INDEX              2U
#define IMU_BODY_X_SENSOR_SIGN               (+1.0f)
#define IMU_BODY_Y_SENSOR_SIGN               (+1.0f)
#define IMU_BODY_Z_SENSOR_SIGN               (+1.0f)

/* One second of stationary data at the 500 Hz control rate. */
#define IMU_GYRO_CALIBRATION_SAMPLES         50U
#define IMU_GYRO_STATIONARY_LIMIT_RADPS      0.20f
#define IMU_ACCEL_STATIONARY_MIN_MPS2        7.50f
#define IMU_ACCEL_STATIONARY_MAX_MPS2        12.00f

/* 6-axis Mahony attitude correction gains. */
#define IMU_ATTITUDE_KP                      1.50f
#define IMU_ATTITUDE_KI                      0.05f
#define IMU_INTEGRAL_FEEDBACK_LIMIT_RADPS    0.20f
#define IMU_ACCEL_CORRECTION_MIN_MPS2        7.00f
#define IMU_ACCEL_CORRECTION_MAX_MPS2        12.50f

typedef struct
{
    volatile float quaternion[4];
    volatile float roll;
    volatile float pitch;
    volatile float yaw;
    volatile float yaw_continuous;
    volatile float gyro_body[3];
    volatile float gyro_bias[3];
    volatile uint32_t calibration_count;
    volatile uint8_t ready;

    float gyro_bias_sum[3];
    float accel_sum[3];
    float integral_feedback[3];
    float last_yaw;
} IMU_Attitude_t;

typedef struct
{
    fp32 gyro[3];
    fp32 accel[3];
    fp32 temperature;
} IMU_RawData_t;

extern IMU_Attitude_t imu_attitude;
extern IMU_RawData_t imu_raw_data;

void IMU_Attitude_Init(void);
void IMU_Attitude_Update(const fp32 gyro_sensor[3],
                         const fp32 accel_sensor[3],
                         float dt);
uint8_t IMU_Attitude_IsReady(void);

#endif
