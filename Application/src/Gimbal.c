#include "Gimbal.h"
Gimbal_Control_t gimbal_control;

static float Gimbal_Limit(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

float Gimbal_Ctrl_CircularError(float target, float feedback)
{
    float error = target - feedback;
    const float half_range = GIMBAL_ENCODER_RANGE * 0.5f;

    if (error > half_range) {
        error -= GIMBAL_ENCODER_RANGE;
    } else if (error < -half_range) {
        error += GIMBAL_ENCODER_RANGE;
    }
    return error;
}

static void Gimbal_InitializeTargets(void)
{
    if (gimbal_control.initialized == 0U) {
        gimbal_control.yaw_target = GIMBAL_YAW_TARGET_COUNT;
        gimbal_control.pitch_target = GIMBAL_PITCH_START_COUNT;
        USER_PID_Reset(&gimbal_control.yaw_position_pid);
        USER_PID_Reset(&gimbal_control.yaw_speed_pid);
        USER_PID_Reset(&gimbal_control.inertial_yaw_position_pid);
        USER_PID_Reset(&gimbal_control.pitch_position_pid);
        USER_PID_Reset(&gimbal_control.pitch_speed_pid);
        gimbal_control.inertial_hold_active = 0U;
        gimbal_control.initialized = 1U;
    }
}

static void Gimbal_UpdatePitchTarget(float remote_pitch, float dt)
{
    gimbal_control.pitch_target +=
        remote_pitch * GIMBAL_PITCH_RATE_COUNT_S * dt;
    gimbal_control.pitch_target =
        Gimbal_Limit(gimbal_control.pitch_target,
                     GIMBAL_PITCH_MIN_COUNT,
                     GIMBAL_PITCH_MAX_COUNT);
}

static float Gimbal_CalculateSpeedTarget(USER_PID_t *position_pid,
                                         float position_error,
                                         float speed_limit,
                                         float dt)
{
    return Gimbal_Limit(
        USER_PID_Calculate(position_pid, position_error, 0.0f, dt),
        -speed_limit,
        speed_limit);
}

static float Gimbal_CalculateSpeedOutput(USER_PID_t *speed_pid,
                                         float speed_target,
                                         float speed_feedback,
                                         float dt)
{
    float output = USER_PID_Calculate(speed_pid,
                                      speed_target,
                                      speed_feedback,
                                      dt);

    return Gimbal_Limit(output,
                        -GIMBAL_COMMAND_MAX,
                        GIMBAL_COMMAND_MAX);
}

static void Gimbal_UpdateYawControl(float yaw_feedback,
                                    float yaw_speed_feedback,
                                    float dt)
{
    gimbal_control.yaw_feedback = yaw_feedback;
    gimbal_control.yaw_speed_feedback = yaw_speed_feedback;
    gimbal_control.yaw_error =
        Gimbal_Ctrl_CircularError(gimbal_control.yaw_target,
                                  gimbal_control.yaw_feedback);

    gimbal_control.yaw_speed_target =
        Gimbal_CalculateSpeedTarget(
            &gimbal_control.yaw_position_pid,
            gimbal_control.yaw_error,
            GIMBAL_YAW_SPEED_MAX_RADPS,
            dt);

    gimbal_control.yaw_output = Gimbal_CalculateSpeedOutput(
        &gimbal_control.yaw_speed_pid,
        gimbal_control.yaw_speed_target,
        gimbal_control.yaw_speed_feedback,
        dt);
}

static void Gimbal_UpdatePitchControl(float pitch_feedback,
                                      float pitch_speed_feedback,
                                      float dt)
{
    gimbal_control.pitch_feedback = pitch_feedback;
    gimbal_control.pitch_speed_feedback = pitch_speed_feedback;
    gimbal_control.pitch_error =
        gimbal_control.pitch_target - gimbal_control.pitch_feedback;

    gimbal_control.pitch_speed_target =
        Gimbal_CalculateSpeedTarget(
            &gimbal_control.pitch_position_pid,
            gimbal_control.pitch_error,
            GIMBAL_PITCH_SPEED_MAX_RADPS,
            dt);

    gimbal_control.pitch_output = Gimbal_CalculateSpeedOutput(
        &gimbal_control.pitch_speed_pid,
        gimbal_control.pitch_speed_target,
        gimbal_control.pitch_speed_feedback,
        dt);
}

void Gimbal_Ctrl_Init(void)
{
    USER_PID_Init(&gimbal_control.yaw_position_pid,
                  0.06f,
                  0.0f,
                  0.0f,
                  0.0f,
                  0.0f,
                  0.0f,
                  0.0f,
                  0.0f);
    USER_PID_Init(&gimbal_control.yaw_speed_pid,
                  45.0f,
                  0.0f,
                  0.0f,
                  14.0f,
                  100.0f,
                  0.0f,
                  0.0f,
                  0.0f);
    USER_PID_Init(&gimbal_control.inertial_yaw_position_pid,
                  100.0f,
                  0.0f,
                  0.0f,
                  0.0f,
                  0.0f,
                  0.0f,
                  0.0f,
                  0.0035f);
    USER_PID_Init(&gimbal_control.pitch_position_pid,
                  0.01f,
                  0.0f,
                  0.0f,
                  0.00005f,
                  0.0f,
                  0.0f,
                  0.0f,
                  0.0f);
    USER_PID_Init(&gimbal_control.pitch_speed_pid,
                  0.05f,
                  0.0f,
                  0.0f,
                  440.0f,
                  0.0f,
                  0.0f,
                  0.0f,
                  0.0f);
    Gimbal_Ctrl_Stop();
}

void Gimbal_Ctrl_Stop(void)
{
    USER_PID_Reset(&gimbal_control.yaw_position_pid);
    USER_PID_Reset(&gimbal_control.yaw_speed_pid);
    USER_PID_Reset(&gimbal_control.inertial_yaw_position_pid);
    USER_PID_Reset(&gimbal_control.pitch_position_pid);
    USER_PID_Reset(&gimbal_control.pitch_speed_pid);
    gimbal_control.yaw_speed_target = 0.0f;
    gimbal_control.yaw_speed_feedback = 0.0f;
    gimbal_control.yaw_output = 0.0f;
    gimbal_control.pitch_speed_target = 0.0f;
    gimbal_control.pitch_speed_feedback = 0.0f;
    gimbal_control.pitch_output = 0.0f;
    gimbal_control.inertial_hold_active = 0U;
    gimbal_control.initialized = 0U;
}

void Gimbal_Ctrl_UpdateNormal(float remote_pitch,
                              float yaw_feedback,
                              float yaw_speed_feedback,
                              float pitch_feedback,
                              float pitch_speed_feedback,
                              float dt)
{
    Gimbal_InitializeTargets();
    Gimbal_UpdatePitchTarget(remote_pitch, dt);
    if (gimbal_control.inertial_hold_active != 0U) {
        USER_PID_Reset(&gimbal_control.yaw_position_pid);
        USER_PID_Reset(&gimbal_control.yaw_speed_pid);
    }
    gimbal_control.inertial_hold_active = 0U;
    gimbal_control.yaw_target = GIMBAL_YAW_TARGET_COUNT;
    Gimbal_UpdateYawControl(yaw_feedback, yaw_speed_feedback, dt);

    Gimbal_UpdatePitchControl(pitch_feedback,
                              pitch_speed_feedback,
                              dt);
}

void Gimbal_Ctrl_UpdateRotate(float remote_yaw,
                              float remote_pitch,
                              float yaw_feedback,
                              float yaw_speed_feedback,
                              float pitch_feedback,
                              float pitch_speed_feedback,
                              float imu_yaw_continuous,
                              float dt)
{
    Gimbal_InitializeTargets();
    Gimbal_UpdatePitchTarget(remote_pitch, dt);
    gimbal_control.yaw_feedback = yaw_feedback;
    gimbal_control.yaw_speed_feedback = yaw_speed_feedback;

    if (gimbal_control.inertial_hold_active == 0U) {
        gimbal_control.inertial_yaw_target = imu_yaw_continuous;
        USER_PID_Reset(&gimbal_control.inertial_yaw_position_pid);
        USER_PID_Reset(&gimbal_control.yaw_speed_pid);
        gimbal_control.inertial_hold_active = 1U;
    }

    gimbal_control.inertial_yaw_target -=
        remote_yaw * GIMBAL_YAW_SPEED_MAX_RADPS * dt;
    gimbal_control.inertial_yaw_feedback = imu_yaw_continuous;
    gimbal_control.inertial_yaw_error =
        gimbal_control.inertial_yaw_target -
        gimbal_control.inertial_yaw_feedback;
    gimbal_control.yaw_error =
        gimbal_control.inertial_yaw_error *
        GIMBAL_ENCODER_COUNT_PER_RAD;
    gimbal_control.yaw_speed_target =
        Gimbal_CalculateSpeedTarget(
            &gimbal_control.inertial_yaw_position_pid,
            gimbal_control.inertial_yaw_error,
            GIMBAL_YAW_SPEED_MAX_RADPS,
            dt);
    gimbal_control.yaw_output = Gimbal_CalculateSpeedOutput(
        &gimbal_control.yaw_speed_pid,
        gimbal_control.yaw_speed_target,
        gimbal_control.yaw_speed_feedback,
        dt);

    Gimbal_UpdatePitchControl(pitch_feedback,
                              pitch_speed_feedback,
                              dt);
}
