#include "Chassis.h"
#include "Gimbal.h"
#include <math.h>

#define CHASSIS_INV_SQRT_2 0.70710678118f

static const float chassis_gyro_exit_deceleration_radps2 = 8.0f;
static const float chassis_gyro_exit_brake_margin_rad = 0.10f;
static const float chassis_gyro_exit_approach_offset_rad = 0.12f;
static const float chassis_gyro_exit_direction_detect_count = 20.0f;
static const uint16_t chassis_gyro_exit_direction_detect_timeout_cycles = 100U;
static const float chassis_gyro_exit_align_min_wz_radps = 0.4f;
static const float chassis_gyro_exit_tolerance_count = 30.0f;
static const float chassis_gyro_exit_wheel_stop_radps = 1.0f;
static const uint16_t chassis_gyro_exit_stable_cycles = 50U;

static const float chassis_wheel_direction[CHASSIS_WHEEL_COUNT] =
{
    CHASSIS_DIRECTION_FL,
    CHASSIS_DIRECTION_FR,
    CHASSIS_DIRECTION_RL,
    CHASSIS_DIRECTION_RR
};

Chassis_Control_t chassis_control;

static float Chassis_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Chassis_Limit(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static uint8_t Chassis_WheelsStopped(
    const float wheel_feedback[CHASSIS_WHEEL_COUNT])
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) {
        if (Chassis_Abs(wheel_feedback[i]) >
            chassis_gyro_exit_wheel_stop_radps) {
            return 0U;
        }
    }
    return 1U;
}

static float Chassis_EstimateWz(
    const float wheel_feedback[CHASSIS_WHEEL_COUNT])
{
    const float scale = CHASSIS_INV_SQRT_2 / CHASSIS_WHEEL_RADIUS_M;
    const float rotation_radius =
        CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M;
    float fl;
    float fr;
    float rl;
    float rr;
    float rotation_wheel_speed;

    fl = wheel_feedback[CHASSIS_WHEEL_FL] *
         chassis_wheel_direction[CHASSIS_WHEEL_FL];
    fr = wheel_feedback[CHASSIS_WHEEL_FR] *
         chassis_wheel_direction[CHASSIS_WHEEL_FR];
    rl = wheel_feedback[CHASSIS_WHEEL_RL] *
         chassis_wheel_direction[CHASSIS_WHEEL_RL];
    rr = wheel_feedback[CHASSIS_WHEEL_RR] *
         chassis_wheel_direction[CHASSIS_WHEEL_RR];
    rotation_wheel_speed = 0.25f * (fl - fr + rl - rr);

    return rotation_wheel_speed / (scale * rotation_radius);
}

static float Chassis_ForwardEncoderDistance(float target,
                                            float feedback,
                                            float encoder_direction)
{
    float distance = (target - feedback) * encoder_direction;

    while (distance < 0.0f) {
        distance += GIMBAL_ENCODER_RANGE;
    }
    while (distance >= GIMBAL_ENCODER_RANGE) {
        distance -= GIMBAL_ENCODER_RANGE;
    }
    return distance;
}

static void Chassis_PlanRotateExitTarget(
    float current_encoder,
    const float wheel_feedback[CHASSIS_WHEEL_COUNT])
{
    float current_wz;
    float stopping_distance_count;
    float brake_margin_count;

    current_wz = Chassis_Abs(Chassis_EstimateWz(wheel_feedback));
    if (current_wz < Chassis_Abs(chassis_control.wz_target)) {
        current_wz = Chassis_Abs(chassis_control.wz_target);
    }

    chassis_control.exit_align_remaining_count =
        Chassis_ForwardEncoderDistance(
            GIMBAL_YAW_TARGET_COUNT,
            current_encoder,
            chassis_control.exit_align_encoder_direction);
    stopping_distance_count =
        current_wz * current_wz /
        (2.0f * chassis_gyro_exit_deceleration_radps2) *
        GIMBAL_ENCODER_COUNT_PER_RAD;
    brake_margin_count =
        chassis_gyro_exit_brake_margin_rad *
        GIMBAL_ENCODER_COUNT_PER_RAD;

    if (chassis_control.exit_align_remaining_count <
        stopping_distance_count + brake_margin_count) {
        chassis_control.exit_align_remaining_count +=
            GIMBAL_ENCODER_RANGE;
    }

    chassis_control.exit_align_stable_count = 0U;
    chassis_control.exit_align_creep_mode = 0U;
}

static void Chassis_InitializeRotateExit(float current_encoder)
{
    chassis_control.exit_align_remaining_count = GIMBAL_ENCODER_RANGE;
    chassis_control.exit_align_last_encoder_count = current_encoder;
    chassis_control.exit_align_direction_accumulator = 0.0f;
    chassis_control.exit_align_encoder_direction = 0.0f;
    chassis_control.exit_align_stable_count = 0U;
    chassis_control.exit_align_direction_detect_cycles = 0U;
    chassis_control.exit_align_direction_detected = 0U;
    chassis_control.exit_align_creep_mode = 0U;
    chassis_control.exit_align_initialized = 1U;
}

static void Chassis_UpdateRotateExitProgress(
    float current_encoder,
    const float wheel_feedback[CHASSIS_WHEEL_COUNT])
{
    float encoder_delta;
    float encoder_progress;
    const float half_range = GIMBAL_ENCODER_RANGE * 0.5f;

    encoder_delta = current_encoder -
                    chassis_control.exit_align_last_encoder_count;
    if (encoder_delta < -half_range) {
        encoder_delta += GIMBAL_ENCODER_RANGE;
    } else if (encoder_delta > half_range) {
        encoder_delta -= GIMBAL_ENCODER_RANGE;
    }
    chassis_control.exit_align_last_encoder_count = current_encoder;

    if (chassis_control.exit_align_direction_detected == 0U) {
        if (chassis_control.exit_align_direction_detect_cycles <
            chassis_gyro_exit_direction_detect_timeout_cycles) {
            chassis_control.exit_align_direction_detect_cycles++;
        }
        chassis_control.exit_align_direction_accumulator += encoder_delta;
        if (Chassis_Abs(chassis_control.exit_align_direction_accumulator) >=
            chassis_gyro_exit_direction_detect_count) {
            chassis_control.exit_align_encoder_direction =
                (chassis_control.exit_align_direction_accumulator > 0.0f)
                    ? 1.0f
                    : -1.0f;
            chassis_control.exit_align_direction_detected = 1U;
            Chassis_PlanRotateExitTarget(current_encoder, wheel_feedback);
        }
        return;
    }

    encoder_progress =
        encoder_delta * chassis_control.exit_align_encoder_direction;
    if (encoder_progress > 0.0f) {
        chassis_control.exit_align_remaining_count -= encoder_progress;
        if (chassis_control.exit_align_remaining_count < 0.0f) {
            chassis_control.exit_align_remaining_count = 0.0f;
        }
    }
}

static float Chassis_CalculateRotateExitWz(float remaining_count)
{
    float remaining_rad;
    float wz;

    remaining_rad =
        remaining_count / GIMBAL_ENCODER_COUNT_PER_RAD -
        chassis_gyro_exit_approach_offset_rad;
    if (remaining_rad < 0.0f) {
        remaining_rad = 0.0f;
    }

    wz = sqrtf(2.0f * chassis_gyro_exit_deceleration_radps2 *
               remaining_rad);
    wz = Chassis_Limit(wz, 0.0f, CHASSIS_SMALL_GYRO_WZ_RADPS);
    if ((wz > 0.0f) && (wz < chassis_gyro_exit_align_min_wz_radps)) {
        wz = chassis_gyro_exit_align_min_wz_radps;
    }
    return wz;
}

void Chassis_Ctrl_Init(void)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) {
        chassis_control.wheel_feedback[i] = 0.0f;
        USER_PID_Init(&chassis_control.speed_pid[i],
                      20.0f,
                      0.0f,
                      0.0f,
                      10.0f,
                      0.0f,
                      0.0f,
                      0.0f);
    }
    Chassis_Ctrl_Stop();
}

void Chassis_Ctrl_Stop(void)
{
    uint8_t i;

    chassis_control.vx_target = 0.0f;
    chassis_control.vy_target = 0.0f;
    chassis_control.wz_target = 0.0f;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) {
        chassis_control.wheel_target[i] = 0.0f;
        chassis_control.wheel_output[i] = 0.0f;
        USER_PID_Reset(&chassis_control.speed_pid[i]);
    }
    Chassis_Ctrl_ResetRotateExit();
}

void Chassis_Ctrl_ResetWheelPIDs(void)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) {
        USER_PID_Reset(&chassis_control.speed_pid[i]);
    }
}

void Chassis_Ctrl_ResetRotateExit(void)
{
    chassis_control.exit_align_error_count = 0.0f;
    chassis_control.exit_align_remaining_count = 0.0f;
    chassis_control.exit_align_last_encoder_count = 0.0f;
    chassis_control.exit_align_direction_accumulator = 0.0f;
    chassis_control.exit_align_encoder_direction = 0.0f;
    chassis_control.exit_align_stable_count = 0U;
    chassis_control.exit_align_direction_detect_cycles = 0U;
    chassis_control.exit_align_direction_detected = 0U;
    chassis_control.exit_align_creep_mode = 0U;
    chassis_control.exit_align_initialized = 0U;
}

void Chassis_Ctrl_PrepareRotate(void)
{
    Chassis_Ctrl_ResetRotateExit();
}

uint8_t Chassis_Ctrl_DirectionsCalibrated(void)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) {
        if (Chassis_Abs(chassis_wheel_direction[i]) < 0.5f) {
            return 0U;
        }
    }
    return 1U;
}

void Chassis_Ctrl_SetRemoteTargets(float remote_forward,
                                   float remote_right,
                                   float remote_clockwise)
{
    chassis_control.vx_target = remote_forward * CHASSIS_MAX_VX_MPS;
    chassis_control.vy_target = remote_right * CHASSIS_MAX_VY_MPS;
    chassis_control.wz_target = remote_clockwise * CHASSIS_MAX_WZ_RADPS;
}

void Chassis_Ctrl_SetRotateTargets(float remote_forward,
                                   float remote_right,
                                   float yaw_feedback)
{
    float gimbal_forward;
    float gimbal_right;
    float relative_encoder_count;
    float gimbal_clockwise_angle;
    float angle_cos;
    float angle_sin;

    gimbal_forward = remote_forward * CHASSIS_MAX_VX_MPS;
    gimbal_right = remote_right * CHASSIS_MAX_VY_MPS;
    relative_encoder_count =
        Gimbal_Ctrl_CircularError(yaw_feedback,
                                  GIMBAL_YAW_TARGET_COUNT);
    gimbal_clockwise_angle =
        -relative_encoder_count *
        GIMBAL_IMU_YAW_TO_ENCODER_DIRECTION /
        GIMBAL_ENCODER_COUNT_PER_RAD;
    angle_cos = cosf(gimbal_clockwise_angle);
    angle_sin = sinf(gimbal_clockwise_angle);

    chassis_control.vx_target =
        gimbal_forward * angle_cos - gimbal_right * angle_sin;
    chassis_control.vy_target =
        gimbal_forward * angle_sin + gimbal_right * angle_cos;
    chassis_control.wz_target = CHASSIS_SMALL_GYRO_WZ_RADPS;
}

uint8_t Chassis_Ctrl_UpdateRotateExit(
    float yaw_feedback,
    const float wheel_feedback[CHASSIS_WHEEL_COUNT])
{
    float approach_offset_count;
    float current_wz;
    float creep_stop_count;

    chassis_control.vx_target = 0.0f;
    chassis_control.vy_target = 0.0f;

    if (chassis_control.exit_align_initialized == 0U) {
        Chassis_InitializeRotateExit(yaw_feedback);
    } else {
        Chassis_UpdateRotateExitProgress(yaw_feedback, wheel_feedback);
    }

    chassis_control.exit_align_error_count =
        Gimbal_Ctrl_CircularError(GIMBAL_YAW_TARGET_COUNT,
                                  yaw_feedback);

    if (chassis_control.exit_align_direction_detected == 0U) {
        if (chassis_control.exit_align_direction_detect_cycles >=
            chassis_gyro_exit_direction_detect_timeout_cycles) {
            chassis_control.wz_target = 0.0f;
            if (Chassis_WheelsStopped(wheel_feedback) != 0U) {
                Chassis_Ctrl_ResetRotateExit();
                return 1U;
            }
        } else {
            chassis_control.wz_target = CHASSIS_SMALL_GYRO_WZ_RADPS;
        }
        return 0U;
    }

    approach_offset_count =
        chassis_gyro_exit_approach_offset_rad *
        GIMBAL_ENCODER_COUNT_PER_RAD;

    if (chassis_control.exit_align_creep_mode == 0U) {
        chassis_control.exit_align_stable_count = 0U;
        if (chassis_control.exit_align_remaining_count <=
            approach_offset_count) {
            chassis_control.wz_target = 0.0f;
            if (Chassis_WheelsStopped(wheel_feedback) != 0U) {
                chassis_control.exit_align_creep_mode = 1U;
            }
        } else {
            chassis_control.wz_target =
                Chassis_CalculateRotateExitWz(
                    chassis_control.exit_align_remaining_count);
        }
        return 0U;
    }

    current_wz = Chassis_Abs(Chassis_EstimateWz(wheel_feedback));
    creep_stop_count =
        chassis_gyro_exit_tolerance_count +
        current_wz * current_wz /
        (2.0f * chassis_gyro_exit_deceleration_radps2) *
        GIMBAL_ENCODER_COUNT_PER_RAD;

    if (chassis_control.exit_align_remaining_count <= creep_stop_count) {
        chassis_control.wz_target = 0.0f;
        if (Chassis_WheelsStopped(wheel_feedback) != 0U) {
            if (chassis_control.exit_align_remaining_count <=
                chassis_gyro_exit_tolerance_count) {
                if (chassis_control.exit_align_stable_count <
                    chassis_gyro_exit_stable_cycles) {
                    chassis_control.exit_align_stable_count++;
                }
            } else {
                chassis_control.exit_align_stable_count = 0U;
            }
        }
    } else {
        chassis_control.exit_align_stable_count = 0U;
        chassis_control.wz_target = chassis_gyro_exit_align_min_wz_radps;
    }

    if (chassis_control.exit_align_stable_count >=
        chassis_gyro_exit_stable_cycles) {
        Chassis_Ctrl_ResetRotateExit();
        return 1U;
    }
    return 0U;
}

void Chassis_Ctrl_CalculateWheelTargets(uint8_t preserve_rotation)
{
    const float scale = CHASSIS_INV_SQRT_2 / CHASSIS_WHEEL_RADIUS_M;
    const float rotation =
        (CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M) *
        chassis_control.wz_target;
    float translation[CHASSIS_WHEEL_COUNT];
    float rotation_target[CHASSIS_WHEEL_COUNT];
    float translation_scale = 1.0f;
    float scale_limit;
    const float wheel_speed_limit = CHASSIS_MAX_VX_MPS * scale;
    uint8_t i;

    if (preserve_rotation == 0U) {
        chassis_control.wheel_target[CHASSIS_WHEEL_FL] =
            (chassis_control.vx_target + chassis_control.vy_target +
             rotation) * scale;
        chassis_control.wheel_target[CHASSIS_WHEEL_FR] =
            (chassis_control.vx_target - chassis_control.vy_target -
             rotation) * scale;
        chassis_control.wheel_target[CHASSIS_WHEEL_RL] =
            (chassis_control.vx_target - chassis_control.vy_target +
             rotation) * scale;
        chassis_control.wheel_target[CHASSIS_WHEEL_RR] =
            (chassis_control.vx_target + chassis_control.vy_target -
             rotation) * scale;
        return;
    }

    rotation_target[CHASSIS_WHEEL_FL] = rotation * scale;
    rotation_target[CHASSIS_WHEEL_FR] = -rotation * scale;
    rotation_target[CHASSIS_WHEEL_RL] = rotation * scale;
    rotation_target[CHASSIS_WHEEL_RR] = -rotation * scale;
    translation[CHASSIS_WHEEL_FL] =
        (chassis_control.vx_target + chassis_control.vy_target) * scale;
    translation[CHASSIS_WHEEL_FR] =
        (chassis_control.vx_target - chassis_control.vy_target) * scale;
    translation[CHASSIS_WHEEL_RL] =
        (chassis_control.vx_target - chassis_control.vy_target) * scale;
    translation[CHASSIS_WHEEL_RR] =
        (chassis_control.vx_target + chassis_control.vy_target) * scale;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) {
        if (translation[i] > 0.0f) {
            scale_limit =
                (wheel_speed_limit - rotation_target[i]) / translation[i];
            if (scale_limit < translation_scale) {
                translation_scale = scale_limit;
            }
        } else if (translation[i] < 0.0f) {
            scale_limit =
                (-wheel_speed_limit - rotation_target[i]) / translation[i];
            if (scale_limit < translation_scale) {
                translation_scale = scale_limit;
            }
        }
    }
    translation_scale = Chassis_Limit(translation_scale, 0.0f, 1.0f);

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) {
        chassis_control.wheel_target[i] =
            rotation_target[i] + translation_scale * translation[i];
    }
}

void Chassis_Ctrl_UpdateOutputs(
    const float wheel_feedback[CHASSIS_WHEEL_COUNT],
    float dt)
{
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) {
        chassis_control.wheel_target[i] *= chassis_wheel_direction[i];
        chassis_control.wheel_feedback[i] = wheel_feedback[i];
        chassis_control.wheel_output[i] =
            USER_PID_Calculate(&chassis_control.speed_pid[i],
                               chassis_control.wheel_target[i],
                               chassis_control.wheel_feedback[i],
                               dt);
    }
}
