#include "motor.h"
#include "Motor_Drv.h"
#include "Remote.h"
#include "imu_attitude.h"
#include "tim.h"
#include "Buzzer.h"
#include <math.h>

#define MOTOR_INV_SQRT_2  0.70710678118f

static const float motor_gyro_exit_deceleration_radps2 = 8.0f;
static const float motor_gyro_exit_brake_margin_rad = 0.10f;
static const float motor_gyro_exit_approach_offset_rad = 0.12f;
static const float motor_gyro_exit_direction_detect_count = 20.0f;
static const uint16_t motor_gyro_exit_direction_detect_timeout_cycles = 100U;
static const float motor_small_gyro_gimbal_yaw_rate_radps = 4.0f;
static const uint16_t motor_emergency_exit_stable_cycles = 25U;

static uint16_t motor_emergency_exit_mid_stable_count = 0U;
static uint8_t motor_emergency_stop_latched = 0U;

/* Chassis wheel-speed PID: edit here or tune live in the Keil Watch window. */
volatile Motor_PID_Parameters_t motor_speed_pid_parameters =
{
    20.0f,  /* kp */
    0.0f,  /* ki */
    0.0f,  /* kd */
    1000000.0f,  /* output_max: 20.0 -> CAN current command about 2000 */
    10.0f   /* integral_max */
};

/* Yaw angle PID: output 5.0 corresponds to CAN current command about 500. */
volatile Motor_PID_Parameters_t motor_yaw_pid_parameters =
{
    100.0f,  /* kp */
    0.0f,  /* ki */
    8.0f,  /* kd */
    1000000.00f,  /* output_max */
    500.0f  /* integral_max */
};

/* Small-gyro yaw PID: angle input is radians, output is CAN torque units. */
volatile Motor_PID_Parameters_t motor_inertial_yaw_pid_parameters =
{
    800.0f,  /* kp */
    0.0f,   /* ki */
    12.0f,   /* kd */
    1000000.0f, /* output_max: CAN current command about 10000 */
    0.5f    /* integral_max */
};

/* Small-gyro yaw feedforward, in gimbal output units per chassis rad/s. */
volatile float motor_inertial_yaw_feedforward_gain = 10.0f;

/* Pitch angle PID: output 5.0 corresponds to CAN current command about 500. */
volatile Motor_PID_Parameters_t motor_pitch_pid_parameters =
{
    0.0f,  /* kp */
    0.0f,  /* ki */
    0.0f,  /* kd */
    1000000.00f,  /* output_max */
    500.0f  /* integral_max */
};

Motor_Control_t motor_control;
volatile uint8_t motor_update_flag = 0U;

static const float motor_wheel_direction[MOTOR_WHEEL_COUNT] =
{
    MOTOR_DIRECTION_FL,
    MOTOR_DIRECTION_FR,
    MOTOR_DIRECTION_RL,
    MOTOR_DIRECTION_RR
};

static float Motor_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Motor_NormalizeRemote(int16_t value)
{
    if ((value > -MOTOR_REMOTE_DEADBAND) &&
        (value < MOTOR_REMOTE_DEADBAND)) {
        return 0.0f;
    }

    if (value > (int16_t)MOTOR_REMOTE_FULL_SCALE) {
        value = (int16_t)MOTOR_REMOTE_FULL_SCALE;
    } else if (value < -(int16_t)MOTOR_REMOTE_FULL_SCALE) {
        value = -(int16_t)MOTOR_REMOTE_FULL_SCALE;
    }

    return (float)value / MOTOR_REMOTE_FULL_SCALE;
}

static float Motor_Limit(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float Motor_CalculateCircularError(float target, float feedback)
{
    float error = target - feedback;
    const float half_range = MOTOR_ENCODER_RANGE * 0.5f;

    if (error > half_range) {
        error -= MOTOR_ENCODER_RANGE;
    } else if (error < -half_range) {
        error += MOTOR_ENCODER_RANGE;
    }
    return error;
}

uint8_t Motor_Control_DirectionsCalibrated(void)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++) {
        if (Motor_Abs(motor_wheel_direction[i]) < 0.5f) {
            return 0U;
        }
    }
    return 1U;
}

void Motor_Control_StopAll(Motor_State_t reason)
{
    uint8_t i;

    motor_control.state = reason;
    motor_control.vx_target = 0.0f;
    motor_control.vy_target = 0.0f;
    motor_control.wz_target = 0.0f;

    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++) {
        motor_control.wheel_target[i] = 0.0f;
        motor_control.wheel_output[i] = 0.0f;
        USER_PID_Reset(&motor_control.speed_pid[i]);
        Motor_Drv_SetWheelTorque(i, 0.0f);
    }

    Motor_Drv_SetFrictionWheelTorque(0U, 0.0f);
    Motor_Drv_SetFrictionWheelTorque(1U, 0.0f);
    Motor_Drv_SetGimbalAngle(0U, 0.0f);
    Motor_Drv_SetGimbalAngle(1U, 0.0f);
    USER_PID_Reset(&motor_control.gimbal.yaw_pid);
    USER_PID_Reset(&motor_control.gimbal.inertial_yaw_pid);
    USER_PID_Reset(&motor_control.gimbal.pitch_pid);
    motor_control.gimbal.yaw_output = 0.0f;
    motor_control.gimbal.pitch_output = 0.0f;
    motor_control.gimbal.exit_align_error_count = 0.0f;
    motor_control.gimbal.exit_align_remaining_count = 0.0f;
    motor_control.gimbal.exit_align_last_encoder_count = 0.0f;
    motor_control.gimbal.exit_align_direction_accumulator = 0.0f;
    motor_control.gimbal.exit_align_encoder_direction = 0.0f;
    motor_control.gimbal.exit_align_stable_count = 0U;
    motor_control.gimbal.exit_align_direction_detect_cycles = 0U;
    motor_control.gimbal.exit_align_direction_detected = 0U;
    motor_control.gimbal.exit_align_creep_mode = 0U;
    motor_control.gimbal.exit_align_initialized = 0U;
    motor_control.gimbal.inertial_hold_active = 0U;
    motor_control.gimbal.initialized = 0U;
}

void Motor_Control_Init(void)
{
    uint8_t i;

    motor_emergency_exit_mid_stable_count = 0U;
    motor_emergency_stop_latched = 0U;
    motor_control.update_count = 0U;
    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++) {
        motor_control.wheel_feedback[i] = 0.0f;
        USER_PID_Init(&motor_control.speed_pid[i],
                      motor_speed_pid_parameters.kp,
                      motor_speed_pid_parameters.ki,
                      motor_speed_pid_parameters.kd,
                      motor_speed_pid_parameters.output_max,
                      motor_speed_pid_parameters.integral_max);
    }

    USER_PID_Init(&motor_control.gimbal.yaw_pid,
                  motor_yaw_pid_parameters.kp,
                  motor_yaw_pid_parameters.ki,
                  motor_yaw_pid_parameters.kd,
                  motor_yaw_pid_parameters.output_max,
                  motor_yaw_pid_parameters.integral_max);
    USER_PID_SetDerivativeFilter(&motor_control.gimbal.yaw_pid,
                                 MOTOR_YAW_D_FILTER_ALPHA);
    USER_PID_Init(&motor_control.gimbal.inertial_yaw_pid,
                  motor_inertial_yaw_pid_parameters.kp,
                  motor_inertial_yaw_pid_parameters.ki,
                  motor_inertial_yaw_pid_parameters.kd,
                  motor_inertial_yaw_pid_parameters.output_max,
                  motor_inertial_yaw_pid_parameters.integral_max);
    USER_PID_SetDerivativeFilter(&motor_control.gimbal.inertial_yaw_pid,
                                 MOTOR_INERTIAL_YAW_D_FILTER_ALPHA);
    USER_PID_Init(&motor_control.gimbal.pitch_pid,
                  motor_pitch_pid_parameters.kp,
                  motor_pitch_pid_parameters.ki,
                  motor_pitch_pid_parameters.kd,
                  motor_pitch_pid_parameters.output_max,
                  motor_pitch_pid_parameters.integral_max);

    Motor_Control_StopAll(MOTOR_STATE_STOPPED);
}

static void Motor_CalculateWheelTargets(float vx, float vy, float wz)
{
    const float scale = MOTOR_INV_SQRT_2 / MOTOR_WHEEL_RADIUS_M;
    const float rotation = (MOTOR_HALF_LENGTH_M + MOTOR_HALF_WIDTH_M) * wz;

    /* +x forward, +y right, +wz clockwise when viewed from above. */
    motor_control.wheel_target[MOTOR_WHEEL_FL] =
        (vx + vy + rotation) * scale;
    motor_control.wheel_target[MOTOR_WHEEL_FR] =
        (vx - vy - rotation) * scale;
    motor_control.wheel_target[MOTOR_WHEEL_RL] =
        (vx - vy + rotation) * scale;
    motor_control.wheel_target[MOTOR_WHEEL_RR] =
        (vx + vy - rotation) * scale;
}

static void Motor_CalculateSmallGyroWheelTargets(float vx,
                                                  float vy,
                                                  float wz)
{
    const float scale = MOTOR_INV_SQRT_2 / MOTOR_WHEEL_RADIUS_M;
    const float rotation =
        (MOTOR_HALF_LENGTH_M + MOTOR_HALF_WIDTH_M) * wz * scale;
    const float wheel_speed_limit = MOTOR_MAX_VX_MPS * scale;
    float translation[MOTOR_WHEEL_COUNT];
    float rotation_target[MOTOR_WHEEL_COUNT];
    float translation_scale = 1.0f;
    float scale_limit;
    uint8_t i;

    translation[MOTOR_WHEEL_FL] = (vx + vy) * scale;
    translation[MOTOR_WHEEL_FR] = (vx - vy) * scale;
    translation[MOTOR_WHEEL_RL] = (vx - vy) * scale;
    translation[MOTOR_WHEEL_RR] = (vx + vy) * scale;

    rotation_target[MOTOR_WHEEL_FL] = rotation;
    rotation_target[MOTOR_WHEEL_FR] = -rotation;
    rotation_target[MOTOR_WHEEL_RL] = rotation;
    rotation_target[MOTOR_WHEEL_RR] = -rotation;

    /* Preserve small-gyro rotation and reduce only translation if saturated. */
    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++) {
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
    translation_scale = Motor_Limit(translation_scale, 0.0f, 1.0f);

    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++) {
        motor_control.wheel_target[i] =
            rotation_target[i] + translation_scale * translation[i];
    }
}

static void Motor_UpdatePIDParameters(USER_PID_t *pid,
                                      volatile Motor_PID_Parameters_t *parameters)
{
    pid->kp = parameters->kp;
    pid->ki = parameters->ki;
    pid->kd = parameters->kd;
    pid->output_max = parameters->output_max;
    pid->integral_max = parameters->integral_max;
}

static void Motor_ResetWheelPIDs(void)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++) {
        USER_PID_Reset(&motor_control.speed_pid[i]);
    }
}

static uint8_t Motor_WheelsStopped(void)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++) {
        if (Motor_Abs(wheel_motors[i].Rx_Data.Velocity) >
            MOTOR_GYRO_EXIT_WHEEL_STOP_RADPS) {
            return 0U;
        }
    }
    return 1U;
}

static float Motor_CalculateForwardEncoderDistance(float target,
                                                   float feedback,
                                                   float encoder_direction)
{
    float distance = (target - feedback) * encoder_direction;

    while (distance < 0.0f) {
        distance += MOTOR_ENCODER_RANGE;
    }
    while (distance >= MOTOR_ENCODER_RANGE) {
        distance -= MOTOR_ENCODER_RANGE;
    }
    return distance;
}

static float Motor_EstimateChassisWz(void)
{
    const float scale = MOTOR_INV_SQRT_2 / MOTOR_WHEEL_RADIUS_M;
    const float rotation_radius =
        MOTOR_HALF_LENGTH_M + MOTOR_HALF_WIDTH_M;
    float fl;
    float fr;
    float rl;
    float rr;
    float rotation_wheel_speed;

    fl = wheel_motors[MOTOR_WHEEL_FL].Rx_Data.Velocity *
         motor_wheel_direction[MOTOR_WHEEL_FL];
    fr = wheel_motors[MOTOR_WHEEL_FR].Rx_Data.Velocity *
         motor_wheel_direction[MOTOR_WHEEL_FR];
    rl = wheel_motors[MOTOR_WHEEL_RL].Rx_Data.Velocity *
         motor_wheel_direction[MOTOR_WHEEL_RL];
    rr = wheel_motors[MOTOR_WHEEL_RR].Rx_Data.Velocity *
         motor_wheel_direction[MOTOR_WHEEL_RR];
    rotation_wheel_speed = 0.25f * (fl - fr + rl - rr);

    return rotation_wheel_speed / (scale * rotation_radius);
}

static void Motor_PlanGyroExitTarget(float current_encoder)
{
    float current_wz;
    float encoder_direction;
    float stopping_distance_count;
    float brake_margin_count;

    encoder_direction =
        motor_control.gimbal.exit_align_encoder_direction;
    current_wz = Motor_Abs(Motor_EstimateChassisWz());
    if (current_wz < Motor_Abs(motor_control.wz_target)) {
        current_wz = Motor_Abs(motor_control.wz_target);
    }

    motor_control.gimbal.exit_align_remaining_count =
        Motor_CalculateForwardEncoderDistance(MOTOR_YAW_TARGET_COUNT,
                                              current_encoder,
                                              encoder_direction);
    stopping_distance_count =
        current_wz * current_wz /
        (2.0f * motor_gyro_exit_deceleration_radps2) *
        MOTOR_ENCODER_COUNT_PER_RAD;
    brake_margin_count =
        motor_gyro_exit_brake_margin_rad * MOTOR_ENCODER_COUNT_PER_RAD;

    /* If this occurrence is too close to stop at, target the next rotation. */
    if (motor_control.gimbal.exit_align_remaining_count <
        stopping_distance_count + brake_margin_count) {
        motor_control.gimbal.exit_align_remaining_count +=
            MOTOR_ENCODER_RANGE;
    }

    motor_control.gimbal.exit_align_stable_count = 0U;
    motor_control.gimbal.exit_align_creep_mode = 0U;
}

static void Motor_InitializeGyroExitAlignment(void)
{
    motor_control.gimbal.exit_align_remaining_count = MOTOR_ENCODER_RANGE;
    motor_control.gimbal.exit_align_last_encoder_count =
        (float)gimbal_motors[0].Rx_Data.Position;
    motor_control.gimbal.exit_align_direction_accumulator = 0.0f;
    motor_control.gimbal.exit_align_encoder_direction = 0.0f;
    motor_control.gimbal.exit_align_stable_count = 0U;
    motor_control.gimbal.exit_align_direction_detect_cycles = 0U;
    motor_control.gimbal.exit_align_direction_detected = 0U;
    motor_control.gimbal.exit_align_creep_mode = 0U;
    motor_control.gimbal.exit_align_initialized = 1U;
}

static void Motor_UpdateGyroExitProgress(void)
{
    float current_encoder;
    float encoder_delta;
    float encoder_progress;
    const float half_range = MOTOR_ENCODER_RANGE * 0.5f;

    current_encoder = (float)gimbal_motors[0].Rx_Data.Position;
    encoder_delta = current_encoder -
                    motor_control.gimbal.exit_align_last_encoder_count;

    if (encoder_delta < -half_range) {
        encoder_delta += MOTOR_ENCODER_RANGE;
    } else if (encoder_delta > half_range) {
        encoder_delta -= MOTOR_ENCODER_RANGE;
    }
    motor_control.gimbal.exit_align_last_encoder_count = current_encoder;

    if (motor_control.gimbal.exit_align_direction_detected == 0U) {
        if (motor_control.gimbal.exit_align_direction_detect_cycles <
            motor_gyro_exit_direction_detect_timeout_cycles) {
            motor_control.gimbal.exit_align_direction_detect_cycles++;
        }
        motor_control.gimbal.exit_align_direction_accumulator += encoder_delta;
        if (Motor_Abs(
                motor_control.gimbal.exit_align_direction_accumulator) >=
            motor_gyro_exit_direction_detect_count) {
            motor_control.gimbal.exit_align_encoder_direction =
                (motor_control.gimbal.exit_align_direction_accumulator > 0.0f)
                    ? 1.0f
                    : -1.0f;
            motor_control.gimbal.exit_align_direction_detected = 1U;
            Motor_PlanGyroExitTarget(current_encoder);
        }
        return;
    }

    encoder_progress =
        encoder_delta * motor_control.gimbal.exit_align_encoder_direction;

    if (encoder_progress > 0.0f) {
        motor_control.gimbal.exit_align_remaining_count -= encoder_progress;
        if (motor_control.gimbal.exit_align_remaining_count < 0.0f) {
            motor_control.gimbal.exit_align_remaining_count = 0.0f;
        }
    }
}

static float Motor_CalculateGyroExitAlignWz(float remaining_count)
{
    float remaining_rad;
    float wz;

    remaining_rad =
        remaining_count / MOTOR_ENCODER_COUNT_PER_RAD -
        motor_gyro_exit_approach_offset_rad;
    if (remaining_rad < 0.0f) {
        remaining_rad = 0.0f;
    }
    wz = sqrtf(2.0f * motor_gyro_exit_deceleration_radps2 *
               remaining_rad);
    wz = Motor_Limit(wz, 0.0f, MOTOR_SMALL_GYRO_WZ_RADPS);

    if ((wz > 0.0f) &&
        (wz < MOTOR_GYRO_EXIT_ALIGN_MIN_WZ_RADPS)) {
        wz = MOTOR_GYRO_EXIT_ALIGN_MIN_WZ_RADPS;
    }
    return wz;
}

static void Motor_SetRemoteChassisTargets(void)
{
    float remote_forward;
    float remote_right;
    float remote_clockwise;

    remote_forward = Motor_NormalizeRemote(rc_ctrl.rc.ch3);
    remote_right = Motor_NormalizeRemote(rc_ctrl.rc.ch2);
    remote_clockwise = Motor_NormalizeRemote(rc_ctrl.rc.ch0);
    motor_control.vx_target = remote_forward * MOTOR_MAX_VX_MPS;
    motor_control.vy_target = remote_right * MOTOR_MAX_VY_MPS;
    motor_control.wz_target = remote_clockwise * MOTOR_MAX_WZ_RADPS;
}

static void Motor_SetSmallGyroChassisTargets(void)
{
    float remote_forward;
    float remote_right;
    float gimbal_forward;
    float gimbal_right;
    float relative_encoder_count;
    float gimbal_clockwise_angle;
    float angle_cos;
    float angle_sin;

    remote_forward = Motor_NormalizeRemote(rc_ctrl.rc.ch3);
    remote_right = Motor_NormalizeRemote(rc_ctrl.rc.ch2);
    gimbal_forward = remote_forward * MOTOR_MAX_VX_MPS;
    gimbal_right = remote_right * MOTOR_MAX_VY_MPS;

    relative_encoder_count =
        Motor_CalculateCircularError(
            (float)gimbal_motors[0].Rx_Data.Position,
            MOTOR_YAW_TARGET_COUNT);
    gimbal_clockwise_angle =
        -relative_encoder_count * MOTOR_IMU_YAW_TO_ENCODER_DIRECTION /
        MOTOR_ENCODER_COUNT_PER_RAD;
    angle_cos = cosf(gimbal_clockwise_angle);
    angle_sin = sinf(gimbal_clockwise_angle);

    /* Convert the gimbal-relative command into the rotating chassis frame. */
    motor_control.vx_target =
        gimbal_forward * angle_cos - gimbal_right * angle_sin;
    motor_control.vy_target =
        gimbal_forward * angle_sin + gimbal_right * angle_cos;
    motor_control.wz_target = MOTOR_SMALL_GYRO_WZ_RADPS;
}

static void Motor_InitializeGimbalTargets(void)
{
    if (motor_control.gimbal.initialized == 0U) {
        motor_control.gimbal.yaw_target = MOTOR_YAW_TARGET_COUNT;
        motor_control.gimbal.pitch_target = MOTOR_PITCH_START_COUNT;
        USER_PID_Reset(&motor_control.gimbal.yaw_pid);
        USER_PID_Reset(&motor_control.gimbal.pitch_pid);
        motor_control.gimbal.inertial_hold_active = 0U;
        motor_control.gimbal.initialized = 1U;
    }
}

static void Motor_UpdateGimbalPitchTarget(void)
{
    float pitch_command;

    pitch_command = Motor_NormalizeRemote(rc_ctrl.rc.ch1);
    motor_control.gimbal.pitch_target +=
        pitch_command * MOTOR_PITCH_RATE_COUNT_S * MOTOR_CONTROL_PERIOD_S;
    motor_control.gimbal.pitch_target =
        Motor_Limit(motor_control.gimbal.pitch_target,
                    MOTOR_PITCH_MIN_COUNT,
                    MOTOR_PITCH_MAX_COUNT);
}

static void Motor_UpdateGimbalPitchControl(void)
{
    motor_control.gimbal.pitch_feedback =
        (float)gimbal_motors[1].Rx_Data.Position;
    motor_control.gimbal.pitch_error =
        motor_control.gimbal.pitch_target -
        motor_control.gimbal.pitch_feedback;
    Motor_UpdatePIDParameters(&motor_control.gimbal.pitch_pid,
                              &motor_pitch_pid_parameters);
    motor_control.gimbal.pitch_output =
        USER_PID_Calculate(&motor_control.gimbal.pitch_pid,
                           motor_control.gimbal.pitch_error,
                           0.0f,
                           MOTOR_CONTROL_PERIOD_S) * MOTOR_PITCH_DIRECTION;
}

static void Motor_UpdateGimbalControl(void)
{
    Motor_InitializeGimbalTargets();
    Motor_UpdateGimbalPitchTarget();
    if (motor_control.gimbal.inertial_hold_active != 0U) {
        /* Exit alignment has reached the normal relative yaw setpoint. */
        motor_control.gimbal.yaw_target = MOTOR_YAW_TARGET_COUNT;
        USER_PID_Reset(&motor_control.gimbal.yaw_pid);
    }
    motor_control.gimbal.inertial_hold_active = 0U;

    motor_control.gimbal.yaw_feedback =
        (float)gimbal_motors[0].Rx_Data.Position;
    motor_control.gimbal.yaw_error =
        Motor_CalculateCircularError(motor_control.gimbal.yaw_target,
                                     motor_control.gimbal.yaw_feedback);

    Motor_UpdatePIDParameters(&motor_control.gimbal.yaw_pid,
                              &motor_yaw_pid_parameters);
    if (Motor_Abs(motor_control.gimbal.yaw_error) <=
        MOTOR_YAW_DEADBAND_COUNT) {
        USER_PID_Reset(&motor_control.gimbal.yaw_pid);
        motor_control.gimbal.yaw_output = 0.0f;
    } else {
        motor_control.gimbal.yaw_output =
            USER_PID_Calculate(&motor_control.gimbal.yaw_pid,
                               motor_control.gimbal.yaw_error,
                               0.0f,
                               MOTOR_CONTROL_PERIOD_S) * MOTOR_YAW_DIRECTION;
    }
    Motor_UpdateGimbalPitchControl();

    Motor_Drv_SetGimbalAngle(0U, motor_control.gimbal.yaw_output);
    Motor_Drv_SetGimbalAngle(1U, motor_control.gimbal.pitch_output);
}

static void Motor_UpdateGimbalSmallGyroControl(void)
{
    float yaw_error_count;
    float yaw_control_error_rad;
    float yaw_pid_output = 0.0f;
    float yaw_feedforward_output;
    float yaw_remote_command;

    Motor_InitializeGimbalTargets();
    Motor_UpdateGimbalPitchTarget();
    motor_control.gimbal.yaw_feedback =
        (float)gimbal_motors[0].Rx_Data.Position;

    if (motor_control.gimbal.inertial_hold_active == 0U) {
        motor_control.gimbal.inertial_yaw_target =
            imu_attitude.yaw_continuous;
        motor_control.gimbal.inertial_encoder_reference =
            motor_control.gimbal.yaw_feedback;
        USER_PID_Reset(&motor_control.gimbal.inertial_yaw_pid);
        motor_control.gimbal.inertial_hold_active = 1U;
    }

    if (rc_ctrl.rc.s2 == RC_SW_DOWN) {
        yaw_remote_command = Motor_NormalizeRemote(rc_ctrl.rc.ch0);
        /* Positive ch0 is clockwise; Mahony yaw is positive CCW. */
        motor_control.gimbal.inertial_yaw_target -=
            yaw_remote_command * motor_small_gyro_gimbal_yaw_rate_radps *
            MOTOR_CONTROL_PERIOD_S;
    }

    motor_control.gimbal.inertial_yaw_feedback =
        imu_attitude.yaw_continuous;
#if MOTOR_IMU_MOUNTED_ON_GIMBAL
    motor_control.gimbal.inertial_yaw_error =
        motor_control.gimbal.inertial_yaw_target -
        motor_control.gimbal.inertial_yaw_feedback;
    yaw_control_error_rad =
        motor_control.gimbal.inertial_yaw_error *
        MOTOR_IMU_YAW_TO_ENCODER_DIRECTION;
    yaw_error_count =
        yaw_control_error_rad * MOTOR_ENCODER_COUNT_PER_RAD;
#else
    motor_control.gimbal.yaw_target =
        motor_control.gimbal.inertial_encoder_reference -
        (motor_control.gimbal.inertial_yaw_feedback -
         motor_control.gimbal.inertial_yaw_target) *
        MOTOR_ENCODER_COUNT_PER_RAD *
        MOTOR_IMU_YAW_TO_ENCODER_DIRECTION;
    yaw_error_count =
        Motor_CalculateCircularError(motor_control.gimbal.yaw_target,
                                     motor_control.gimbal.yaw_feedback);
    motor_control.gimbal.inertial_yaw_error =
        yaw_error_count / MOTOR_ENCODER_COUNT_PER_RAD;
    yaw_control_error_rad = motor_control.gimbal.inertial_yaw_error;
#endif
    motor_control.gimbal.yaw_error = yaw_error_count;

    Motor_UpdatePIDParameters(&motor_control.gimbal.inertial_yaw_pid,
                              &motor_inertial_yaw_pid_parameters);
    if (Motor_Abs(yaw_control_error_rad) <=
        MOTOR_INERTIAL_YAW_DEADBAND_RAD) {
        USER_PID_Reset(&motor_control.gimbal.inertial_yaw_pid);
    } else {
        yaw_pid_output =
            USER_PID_Calculate(&motor_control.gimbal.inertial_yaw_pid,
                               yaw_control_error_rad,
                               0.0f,
                               MOTOR_CONTROL_PERIOD_S) * MOTOR_YAW_DIRECTION;
    }
    yaw_feedforward_output =
        motor_inertial_yaw_feedforward_gain * motor_control.wz_target;
    motor_control.gimbal.yaw_output =
        Motor_Limit(yaw_pid_output + yaw_feedforward_output,
                    -MOTOR_GIMBAL_COMMAND_MAX,
                    MOTOR_GIMBAL_COMMAND_MAX);
    Motor_UpdateGimbalPitchControl();

    Motor_Drv_SetGimbalAngle(0U, motor_control.gimbal.yaw_output);
    Motor_Drv_SetGimbalAngle(1U, motor_control.gimbal.pitch_output);
}

void Motor_Control_Update_2ms(void)
{
    uint8_t i;
    float exit_align_error;
    Motor_State_t requested_state;

    motor_control.update_count++;

    if (Remote_IsOnline() == 0U) {
        motor_emergency_exit_mid_stable_count = 0U;
        Motor_Control_StopAll(MOTOR_STATE_REMOTE_OFFLINE);
        Motor_Drv_Send_All();
        return;
    }

    if (rc_ctrl.rc.s1 == RC_SW_DOWN) {
        if (motor_emergency_stop_latched == 0U) {
            Buzzer_PlayEmergencyDoubleBeep();
        }
        motor_emergency_stop_latched = 1U;
        motor_emergency_exit_mid_stable_count = 0U;
        Motor_Control_StopAll(MOTOR_STATE_EMERGENCY_STOP);
        Motor_Drv_Send_All();
        return;
    }

    if (motor_emergency_stop_latched != 0U) {
        if (rc_ctrl.rc.s1 != RC_SW_MID) {
            motor_emergency_exit_mid_stable_count = 0U;
            Motor_Control_StopAll(MOTOR_STATE_EMERGENCY_STOP);
            Motor_Drv_Send_All();
            return;
        }

        if (motor_emergency_exit_mid_stable_count <
            motor_emergency_exit_stable_cycles) {
            motor_emergency_exit_mid_stable_count++;
        }
        if (motor_emergency_exit_mid_stable_count <
            motor_emergency_exit_stable_cycles) {
            Motor_Control_StopAll(MOTOR_STATE_EMERGENCY_STOP);
            Motor_Drv_Send_All();
            return;
        }

        motor_emergency_stop_latched = 0U;
        motor_emergency_exit_mid_stable_count = 0U;
        Buzzer_PlayEmergencyDoubleBeep();
    }

    if (rc_ctrl.rc.s1 != RC_SW_MID) {
        Motor_Control_StopAll(MOTOR_STATE_STOPPED);
        Motor_Drv_Send_All();
        return;
    }

    if (Motor_Control_DirectionsCalibrated() == 0U) {
        Motor_Control_StopAll(MOTOR_STATE_DIRECTION_UNCALIBRATED);
        Motor_Drv_Send_All();
        return;
    }

    if (rc_ctrl.rc.s2 == RC_SW_DOWN) {
        if (IMU_Attitude_IsReady() == 0U) {
            Motor_Control_StopAll(MOTOR_STATE_IMU_NOT_READY);
            Motor_Drv_Send_All();
            return;
        }

        requested_state = MOTOR_STATE_SMALL_GYRO;
        motor_control.gimbal.exit_align_error_count = 0.0f;
        motor_control.gimbal.exit_align_stable_count = 0U;
        motor_control.gimbal.exit_align_direction_detect_cycles = 0U;
        motor_control.gimbal.exit_align_direction_detected = 0U;
        motor_control.gimbal.exit_align_creep_mode = 0U;
        motor_control.gimbal.exit_align_initialized = 0U;
        Motor_SetSmallGyroChassisTargets();
    } else if ((motor_control.state == MOTOR_STATE_SMALL_GYRO) ||
               (motor_control.state == MOTOR_STATE_GYRO_EXIT_ALIGN)) {
        requested_state = MOTOR_STATE_GYRO_EXIT_ALIGN;
        motor_control.vx_target = 0.0f;
        motor_control.vy_target = 0.0f;

        if (motor_control.gimbal.exit_align_initialized == 0U) {
            Motor_InitializeGyroExitAlignment();
        } else {
            Motor_UpdateGyroExitProgress();
        }

        exit_align_error =
            Motor_CalculateCircularError(
                MOTOR_YAW_TARGET_COUNT,
                (float)gimbal_motors[0].Rx_Data.Position);
        motor_control.gimbal.exit_align_error_count = exit_align_error;

        if (motor_control.gimbal.exit_align_direction_detected == 0U) {
            if (motor_control.gimbal.exit_align_direction_detect_cycles >=
                motor_gyro_exit_direction_detect_timeout_cycles) {
                motor_control.wz_target = 0.0f;
                if (Motor_WheelsStopped() != 0U) {
                    requested_state = MOTOR_STATE_RUNNING;
                    motor_control.gimbal.exit_align_initialized = 0U;
                    Motor_SetRemoteChassisTargets();
                }
            } else {
                /* Keep the original direction briefly while detecting sign. */
                motor_control.wz_target = MOTOR_SMALL_GYRO_WZ_RADPS;
            }
        } else {
            float approach_offset_count =
                motor_gyro_exit_approach_offset_rad *
                MOTOR_ENCODER_COUNT_PER_RAD;

            if (motor_control.gimbal.exit_align_creep_mode == 0U) {
                motor_control.gimbal.exit_align_stable_count = 0U;
                if (motor_control.gimbal.exit_align_remaining_count <=
                    approach_offset_count) {
                    motor_control.wz_target = 0.0f;
                    if (Motor_WheelsStopped() != 0U) {
                        motor_control.gimbal.exit_align_creep_mode = 1U;
                    }
                } else {
                    motor_control.wz_target =
                        Motor_CalculateGyroExitAlignWz(
                            motor_control.gimbal.exit_align_remaining_count);
                }
            } else {
                float current_wz = Motor_Abs(Motor_EstimateChassisWz());
                float creep_stop_count =
                    MOTOR_GYRO_EXIT_ALIGN_TOLERANCE_COUNT +
                    current_wz * current_wz /
                    (2.0f * motor_gyro_exit_deceleration_radps2) *
                    MOTOR_ENCODER_COUNT_PER_RAD;

                if (motor_control.gimbal.exit_align_remaining_count <=
                    creep_stop_count) {
                    motor_control.wz_target = 0.0f;
                    if (Motor_WheelsStopped() != 0U) {
                        if (motor_control.gimbal.exit_align_remaining_count <=
                            MOTOR_GYRO_EXIT_ALIGN_TOLERANCE_COUNT) {
                            if (motor_control.gimbal.exit_align_stable_count <
                                MOTOR_GYRO_EXIT_STABLE_CYCLES) {
                                motor_control.gimbal.exit_align_stable_count++;
                            }
                        } else {
                            motor_control.gimbal.exit_align_stable_count = 0U;
                        }
                    }
                } else {
                    motor_control.gimbal.exit_align_stable_count = 0U;
                    motor_control.wz_target =
                        MOTOR_GYRO_EXIT_ALIGN_MIN_WZ_RADPS;
                }

                if (motor_control.gimbal.exit_align_stable_count >=
                    MOTOR_GYRO_EXIT_STABLE_CYCLES) {
                    requested_state = MOTOR_STATE_RUNNING;
                    motor_control.gimbal.exit_align_stable_count = 0U;
                    motor_control.gimbal.exit_align_direction_detect_cycles =
                        0U;
                    motor_control.gimbal.exit_align_direction_detected = 0U;
                    motor_control.gimbal.exit_align_creep_mode = 0U;
                    motor_control.gimbal.exit_align_initialized = 0U;
                    Motor_SetRemoteChassisTargets();
                }
            }
        }
    } else {
        requested_state = MOTOR_STATE_RUNNING;
        Motor_SetRemoteChassisTargets();
    }

    if (motor_control.state != requested_state) {
        Motor_ResetWheelPIDs();
    }
    if (requested_state == MOTOR_STATE_SMALL_GYRO) {
        Motor_CalculateSmallGyroWheelTargets(motor_control.vx_target,
                                              motor_control.vy_target,
                                              motor_control.wz_target);
    } else {
        Motor_CalculateWheelTargets(motor_control.vx_target,
                                    motor_control.vy_target,
                                    motor_control.wz_target);
    }

    if ((requested_state == MOTOR_STATE_SMALL_GYRO) ||
        (requested_state == MOTOR_STATE_GYRO_EXIT_ALIGN)) {
        Motor_UpdateGimbalSmallGyroControl();
    } else {
        Motor_UpdateGimbalControl();
    }

    motor_control.state = requested_state;
    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++) {
        Motor_UpdatePIDParameters(&motor_control.speed_pid[i],
                                  &motor_speed_pid_parameters);
        motor_control.wheel_target[i] *= motor_wheel_direction[i];
        motor_control.wheel_feedback[i] = wheel_motors[i].Rx_Data.Velocity;
        motor_control.wheel_output[i] =
            USER_PID_Calculate(&motor_control.speed_pid[i],
                               motor_control.wheel_target[i],
                               motor_control.wheel_feedback[i],
                               MOTOR_CONTROL_PERIOD_S);
        Motor_Drv_SetWheelTorque(i, motor_control.wheel_output[i]);
    }

    Motor_Drv_Send_All();
}

/* RTOS mode: timing is handled by vTaskDelayUntil in MotorCtrlTask. */
/* Bare-metal mode: uncomment HAL_TIM_PeriodElapsedCallback below. */
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        motor_update_flag = 1U;
    }
}
*/
