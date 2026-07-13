#include "motor.h"
#include "Motor_Drv.h"
#include "Remote.h"
#include "tim.h"

#define MOTOR_INV_SQRT_2  0.70710678118f

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
    500.0f,  /* kp */
    0.0f,  /* ki */
    50.0f,  /* kd */
    1000000.00f,  /* output_max */
    500.0f  /* integral_max */
};

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
    USER_PID_Reset(&motor_control.gimbal.pitch_pid);
    motor_control.gimbal.yaw_output = 0.0f;
    motor_control.gimbal.pitch_output = 0.0f;
    motor_control.gimbal.initialized = 0U;
}

void Motor_Control_Init(void)
{
    uint8_t i;

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

static void Motor_UpdatePIDParameters(USER_PID_t *pid,
                                      volatile Motor_PID_Parameters_t *parameters)
{
    pid->kp = parameters->kp;
    pid->ki = parameters->ki;
    pid->kd = parameters->kd;
    pid->output_max = parameters->output_max;
    pid->integral_max = parameters->integral_max;
}

static void Motor_UpdateGimbalControl(void)
{
    float pitch_command;

    if (motor_control.gimbal.initialized == 0U) {
        motor_control.gimbal.yaw_target = MOTOR_YAW_TARGET_COUNT;
        motor_control.gimbal.pitch_target = MOTOR_PITCH_START_COUNT;
        USER_PID_Reset(&motor_control.gimbal.yaw_pid);
        USER_PID_Reset(&motor_control.gimbal.pitch_pid);
        motor_control.gimbal.initialized = 1U;
    }

    pitch_command = Motor_NormalizeRemote(rc_ctrl.rc.ch1);
    motor_control.gimbal.pitch_target +=
        pitch_command * MOTOR_PITCH_RATE_COUNT_S * MOTOR_CONTROL_PERIOD_S;
    motor_control.gimbal.pitch_target =
        Motor_Limit(motor_control.gimbal.pitch_target,
                    MOTOR_PITCH_MIN_COUNT,
                    MOTOR_PITCH_MAX_COUNT);

    motor_control.gimbal.yaw_feedback =
        (float)gimbal_motors[0].Rx_Data.Position;
    motor_control.gimbal.pitch_feedback =
        (float)gimbal_motors[1].Rx_Data.Position;
    motor_control.gimbal.yaw_error =
        Motor_CalculateCircularError(motor_control.gimbal.yaw_target,
                                     motor_control.gimbal.yaw_feedback);
    motor_control.gimbal.pitch_error =
        motor_control.gimbal.pitch_target -
        motor_control.gimbal.pitch_feedback;

    Motor_UpdatePIDParameters(&motor_control.gimbal.yaw_pid,
                              &motor_yaw_pid_parameters);
    Motor_UpdatePIDParameters(&motor_control.gimbal.pitch_pid,
                              &motor_pitch_pid_parameters);
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
    motor_control.gimbal.pitch_output =
        USER_PID_Calculate(&motor_control.gimbal.pitch_pid,
                           motor_control.gimbal.pitch_error,
                           0.0f,
                           MOTOR_CONTROL_PERIOD_S) * MOTOR_PITCH_DIRECTION;

    Motor_Drv_SetGimbalAngle(0U, motor_control.gimbal.yaw_output);
    Motor_Drv_SetGimbalAngle(1U, motor_control.gimbal.pitch_output);
}

void Motor_Control_Update_2ms(void)
{
    uint8_t i;
    float remote_forward;
    float remote_right;
    float remote_clockwise;

    motor_control.update_count++;

    if (Remote_IsOnline() == 0U) {
        Motor_Control_StopAll(MOTOR_STATE_REMOTE_OFFLINE);
        Motor_Drv_Send_All();
        return;
    }

    if (rc_ctrl.rc.s1 == RC_SW_DOWN) {
        Motor_Control_StopAll(MOTOR_STATE_EMERGENCY_STOP);
        Motor_Drv_Send_All();
        return;
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

    remote_forward = Motor_NormalizeRemote(rc_ctrl.rc.ch3);
    remote_right = Motor_NormalizeRemote(rc_ctrl.rc.ch2);
    remote_clockwise = Motor_NormalizeRemote(rc_ctrl.rc.ch0);
    motor_control.vx_target = remote_forward * MOTOR_MAX_VX_MPS;
    motor_control.vy_target = remote_right * MOTOR_MAX_VY_MPS;
    motor_control.wz_target = remote_clockwise * MOTOR_MAX_WZ_RADPS;
    Motor_CalculateWheelTargets(motor_control.vx_target,
                                motor_control.vy_target,
                                motor_control.wz_target);

    Motor_UpdateGimbalControl();

    motor_control.state = MOTOR_STATE_RUNNING;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        motor_update_flag = 1U;
    }
}
