#include "imu_temp_ctrl.h"

#include "BMI088driver.h"
#include "FreeRTOS.h"
#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#include "task.h"

#include <math.h>

#define IMU_TASK_PERIOD_MS                 2U
#define IMU_EKF_PERIOD_S                   0.002f
#define IMU_GYRO_CALIBRATION_SAMPLES       500U
#define IMU_STATIONARY_GYRO_LIMIT_RAD_S    0.15f
#define IMU_STATIONARY_ACCEL_MIN_M_S2      8.0f
#define IMU_STATIONARY_ACCEL_MAX_M_S2      12.0f
#define DEG_TO_RAD                         0.01745329252f

INS_t INS;

static volatile uint8_t imu_ready;
static float gyro_bias[3];
static float gyro_body[3];
static uint32_t calibration_samples;

static uint8_t IMU_IsStationary(const float gyro_sensor[3],
                                const float accel_sensor[3])
{
    float gyro_norm;
    float accel_norm;

    gyro_norm = sqrtf(gyro_sensor[0] * gyro_sensor[0] +
                      gyro_sensor[1] * gyro_sensor[1] +
                      gyro_sensor[2] * gyro_sensor[2]);
    accel_norm = sqrtf(accel_sensor[0] * accel_sensor[0] +
                       accel_sensor[1] * accel_sensor[1] +
                       accel_sensor[2] * accel_sensor[2]);

    return (gyro_norm < IMU_STATIONARY_GYRO_LIMIT_RAD_S) &&
           (accel_norm > IMU_STATIONARY_ACCEL_MIN_M_S2) &&
           (accel_norm < IMU_STATIONARY_ACCEL_MAX_M_S2);
}

static uint8_t IMU_UpdateGyroBias(const float gyro_sensor[3],
                                  const float accel_sensor[3])
{
    uint32_t axis;

    if (IMU_IsStationary(gyro_sensor, accel_sensor) == 0U) {
        calibration_samples = 0U;
        for (axis = 0U; axis < 3U; axis++) {
            gyro_bias[axis] = 0.0f;
        }
        return 0U;
    }

    for (axis = 0U; axis < 3U; axis++) {
        gyro_bias[axis] += gyro_sensor[axis];
    }
    calibration_samples++;

    if (calibration_samples < IMU_GYRO_CALIBRATION_SAMPLES) {
        return 0U;
    }

    for (axis = 0U; axis < 3U; axis++) {
        gyro_bias[axis] /= (float)IMU_GYRO_CALIBRATION_SAMPLES;
    }
    return 1U;
}

static void IMU_MapToBody(const float gyro_sensor[3],
                          const float accel_sensor[3])
{
    float gyro_corrected[3];

    gyro_corrected[0] = gyro_sensor[0] - gyro_bias[0];
    gyro_corrected[1] = gyro_sensor[1] - gyro_bias[1];
    gyro_corrected[2] = gyro_sensor[2] - gyro_bias[2];

    gyro_body[0] = gyro_corrected[2];
    gyro_body[1] = -gyro_corrected[0];
    gyro_body[2] = -gyro_corrected[1];

    INS.gyro[0] = gyro_body[0];
    INS.gyro[1] = gyro_body[1];
    INS.gyro[2] = gyro_body[2];
    INS.accel[0] = accel_sensor[2];
    INS.accel[1] = -accel_sensor[0];
    INS.accel[2] = -accel_sensor[1];
}

void INS_Init(void)
{
    DWT_Init(SystemCoreClock / 1000000U);
    imu_ready = 0U;
    calibration_samples = 0U;

    while (BMI088_init() != 0U) {
        vTaskDelay(pdMS_TO_TICKS(10U));
    }

    IMU_QuaternionEKF_Init(10.0f,
                           0.001f,
                           10000000.0f,
                           1.0f,
                           IMU_EKF_PERIOD_S,
                           0.001f);
}

void INS_Task(void)
{
    float gyro_sensor[3];
    float accel_sensor[3];

    BMI088_read(gyro_sensor, accel_sensor, &INS.temp);

    if (imu_ready == 0U) {
        if (IMU_UpdateGyroBias(gyro_sensor, accel_sensor) == 0U) {
            return;
        }
    }

    IMU_MapToBody(gyro_sensor, accel_sensor);
    IMU_QuaternionEKF_Update(INS.gyro[0],
                             INS.gyro[1],
                             INS.gyro[2],
                             INS.accel[0],
                             INS.accel[1],
                             INS.accel[2]);

    INS.q[0] = QEKF_INS.q[0];
    INS.q[1] = QEKF_INS.q[1];
    INS.q[2] = QEKF_INS.q[2];
    INS.q[3] = QEKF_INS.q[3];
    INS.roll = QEKF_INS.Roll * DEG_TO_RAD;
    INS.pitch = QEKF_INS.Pitch * DEG_TO_RAD;
    INS.yaw = QEKF_INS.Yaw * DEG_TO_RAD;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle * DEG_TO_RAD;
    imu_ready = 1U;
}

uint8_t IMU_Attitude_IsReady(void)
{
    return imu_ready;
}

void IMU_Attitude_GetGyroBody(float output[3])
{
    if (output == NULL) {
        return;
    }

    output[0] = gyro_body[0];
    output[1] = gyro_body[1];
    output[2] = gyro_body[2];
}

float IMU_Attitude_GetYawContinuousRad(void)
{
    return INS.YawTotalAngle;
}

void OS_IMUCallback(void const *argument)
{
    TickType_t last_wake;

    (void)argument;
    INS_Init();
    last_wake = xTaskGetTickCount();

    for (;;) {
        INS_Task();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}
