#include "imu_attitude.h"
#include <math.h>
#include <string.h>

#define IMU_PI      3.14159265358979323846f
#define IMU_TWO_PI  6.28318530717958647692f

IMU_Attitude_t imu_attitude;

static float IMU_Attitude_Limit(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float IMU_Attitude_WrapPi(float angle)
{
    while (angle > IMU_PI) {
        angle -= IMU_TWO_PI;
    }
    while (angle < -IMU_PI) {
        angle += IMU_TWO_PI;
    }
    return angle;
}

static void IMU_Attitude_MapSensorAxes(const fp32 sensor[3], float body[3])
{
    body[0] = sensor[IMU_BODY_X_SENSOR_INDEX] * IMU_BODY_X_SENSOR_SIGN;
    body[1] = sensor[IMU_BODY_Y_SENSOR_INDEX] * IMU_BODY_Y_SENSOR_SIGN;
    body[2] = sensor[IMU_BODY_Z_SENSOR_INDEX] * IMU_BODY_Z_SENSOR_SIGN;
}

static void IMU_Attitude_ResetCalibration(void)
{
    uint8_t i;

    imu_attitude.calibration_count = 0U;
    for (i = 0U; i < 3U; i++) {
        imu_attitude.gyro_bias_sum[i] = 0.0f;
        imu_attitude.accel_sum[i] = 0.0f;
    }
}

static void IMU_Attitude_UpdateEuler(uint8_t initialize_continuous_yaw)
{
    const float q0 = imu_attitude.quaternion[0];
    const float q1 = imu_attitude.quaternion[1];
    const float q2 = imu_attitude.quaternion[2];
    const float q3 = imu_attitude.quaternion[3];
    float pitch_sine;
    float yaw;
    float yaw_delta;

    imu_attitude.roll =
        atan2f(2.0f * (q0 * q1 + q2 * q3),
               1.0f - 2.0f * (q1 * q1 + q2 * q2));

    pitch_sine = 2.0f * (q0 * q2 - q3 * q1);
    if (pitch_sine > 1.0f) {
        pitch_sine = 1.0f;
    } else if (pitch_sine < -1.0f) {
        pitch_sine = -1.0f;
    }
    imu_attitude.pitch = asinf(pitch_sine);

    yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    imu_attitude.yaw = yaw;

    if (initialize_continuous_yaw != 0U) {
        imu_attitude.yaw_continuous = yaw;
    } else {
        yaw_delta = IMU_Attitude_WrapPi(yaw - imu_attitude.last_yaw);
        imu_attitude.yaw_continuous += yaw_delta;
    }
    imu_attitude.last_yaw = yaw;
}

static void IMU_Attitude_FinishCalibration(void)
{
    float accel_average[3];
    float roll;
    float pitch;
    float half_roll;
    float half_pitch;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
    float horizontal_norm;
    uint8_t i;

    for (i = 0U; i < 3U; i++) {
        imu_attitude.gyro_bias[i] =
            imu_attitude.gyro_bias_sum[i] /
            (float)IMU_GYRO_CALIBRATION_SAMPLES;
        accel_average[i] =
            imu_attitude.accel_sum[i] /
            (float)IMU_GYRO_CALIBRATION_SAMPLES;
        imu_attitude.integral_feedback[i] = 0.0f;
    }

    horizontal_norm = sqrtf(accel_average[1] * accel_average[1] +
                            accel_average[2] * accel_average[2]);
    roll = atan2f(accel_average[1], accel_average[2]);
    pitch = atan2f(-accel_average[0], horizontal_norm);

    half_roll = 0.5f * roll;
    half_pitch = 0.5f * pitch;
    cos_roll = cosf(half_roll);
    sin_roll = sinf(half_roll);
    cos_pitch = cosf(half_pitch);
    sin_pitch = sinf(half_pitch);

    /* Initial yaw is arbitrary without a magnetometer; define it as zero. */
    imu_attitude.quaternion[0] = cos_roll * cos_pitch;
    imu_attitude.quaternion[1] = sin_roll * cos_pitch;
    imu_attitude.quaternion[2] = cos_roll * sin_pitch;
    imu_attitude.quaternion[3] = -sin_roll * sin_pitch;
    IMU_Attitude_UpdateEuler(1U);
    imu_attitude.ready = 1U;
}

void IMU_Attitude_Init(void)
{
    memset(&imu_attitude, 0, sizeof(imu_attitude));
    imu_attitude.quaternion[0] = 1.0f;
}

uint8_t IMU_Attitude_IsReady(void)
{
    return imu_attitude.ready;
}

void IMU_Attitude_Update(const fp32 gyro_sensor[3],
                         const fp32 accel_sensor[3],
                         float dt)
{
    float gyro_body[3];
    float accel_body[3];
    float gyro_norm_sq;
    float accel_norm_sq;
    float accel_norm;
    float reciprocal_norm;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float q0;
    float q1;
    float q2;
    float q3;
    float half_vx;
    float half_vy;
    float half_vz;
    float half_ex;
    float half_ey;
    float half_ez;
    float q_dot0;
    float q_dot1;
    float q_dot2;
    float q_dot3;
    uint8_t i;

    if ((gyro_sensor == NULL) || (accel_sensor == NULL) || (dt <= 0.0f)) {
        return;
    }

    IMU_Attitude_MapSensorAxes(gyro_sensor, gyro_body);
    IMU_Attitude_MapSensorAxes(accel_sensor, accel_body);
    gyro_norm_sq = gyro_body[0] * gyro_body[0] +
                   gyro_body[1] * gyro_body[1] +
                   gyro_body[2] * gyro_body[2];
    accel_norm_sq = accel_body[0] * accel_body[0] +
                    accel_body[1] * accel_body[1] +
                    accel_body[2] * accel_body[2];

    if (imu_attitude.ready == 0U) {
        const float gyro_limit_sq =
            IMU_GYRO_STATIONARY_LIMIT_RADPS *
            IMU_GYRO_STATIONARY_LIMIT_RADPS;
        const float accel_min_sq =
            IMU_ACCEL_STATIONARY_MIN_MPS2 *
            IMU_ACCEL_STATIONARY_MIN_MPS2;
        const float accel_max_sq =
            IMU_ACCEL_STATIONARY_MAX_MPS2 *
            IMU_ACCEL_STATIONARY_MAX_MPS2;

        if ((gyro_norm_sq > gyro_limit_sq) ||
            (accel_norm_sq < accel_min_sq) ||
            (accel_norm_sq > accel_max_sq)) {
            IMU_Attitude_ResetCalibration();
            return;
        }

        for (i = 0U; i < 3U; i++) {
            imu_attitude.gyro_bias_sum[i] += gyro_body[i];
            imu_attitude.accel_sum[i] += accel_body[i];
        }
        imu_attitude.calibration_count++;
        if (imu_attitude.calibration_count >=
            IMU_GYRO_CALIBRATION_SAMPLES) {
            IMU_Attitude_FinishCalibration();
        }
        return;
    }

    gx = gyro_body[0] - imu_attitude.gyro_bias[0];
    gy = gyro_body[1] - imu_attitude.gyro_bias[1];
    gz = gyro_body[2] - imu_attitude.gyro_bias[2];
    imu_attitude.gyro_body[0] = gx;
    imu_attitude.gyro_body[1] = gy;
    imu_attitude.gyro_body[2] = gz;

    q0 = imu_attitude.quaternion[0];
    q1 = imu_attitude.quaternion[1];
    q2 = imu_attitude.quaternion[2];
    q3 = imu_attitude.quaternion[3];

    accel_norm = sqrtf(accel_norm_sq);
    if ((accel_norm >= IMU_ACCEL_CORRECTION_MIN_MPS2) &&
        (accel_norm <= IMU_ACCEL_CORRECTION_MAX_MPS2)) {
        reciprocal_norm = 1.0f / accel_norm;
        ax = accel_body[0] * reciprocal_norm;
        ay = accel_body[1] * reciprocal_norm;
        az = accel_body[2] * reciprocal_norm;

        half_vx = q1 * q3 - q0 * q2;
        half_vy = q0 * q1 + q2 * q3;
        half_vz = q0 * q0 - 0.5f + q3 * q3;
        half_ex = ay * half_vz - az * half_vy;
        half_ey = az * half_vx - ax * half_vz;
        half_ez = ax * half_vy - ay * half_vx;

        imu_attitude.integral_feedback[0] +=
            2.0f * IMU_ATTITUDE_KI * half_ex * dt;
        imu_attitude.integral_feedback[1] +=
            2.0f * IMU_ATTITUDE_KI * half_ey * dt;
        imu_attitude.integral_feedback[2] +=
            2.0f * IMU_ATTITUDE_KI * half_ez * dt;
        imu_attitude.integral_feedback[0] =
            IMU_Attitude_Limit(imu_attitude.integral_feedback[0],
                               IMU_INTEGRAL_FEEDBACK_LIMIT_RADPS);
        imu_attitude.integral_feedback[1] =
            IMU_Attitude_Limit(imu_attitude.integral_feedback[1],
                               IMU_INTEGRAL_FEEDBACK_LIMIT_RADPS);
        imu_attitude.integral_feedback[2] =
            IMU_Attitude_Limit(imu_attitude.integral_feedback[2],
                               IMU_INTEGRAL_FEEDBACK_LIMIT_RADPS);

        gx += 2.0f * IMU_ATTITUDE_KP * half_ex +
              imu_attitude.integral_feedback[0];
        gy += 2.0f * IMU_ATTITUDE_KP * half_ey +
              imu_attitude.integral_feedback[1];
        gz += 2.0f * IMU_ATTITUDE_KP * half_ez +
              imu_attitude.integral_feedback[2];
    }

    q_dot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    q_dot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    q_dot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
    q0 += q_dot0 * dt;
    q1 += q_dot1 * dt;
    q2 += q_dot2 * dt;
    q3 += q_dot3 * dt;

    reciprocal_norm = 1.0f /
        sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    imu_attitude.quaternion[0] = q0 * reciprocal_norm;
    imu_attitude.quaternion[1] = q1 * reciprocal_norm;
    imu_attitude.quaternion[2] = q2 * reciprocal_norm;
    imu_attitude.quaternion[3] = q3 * reciprocal_norm;
    IMU_Attitude_UpdateEuler(0U);
}
