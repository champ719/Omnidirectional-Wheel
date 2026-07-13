#include "motor.h"
#include "Motor_Drv.h"
#include "Remote.h"
#include "tim.h"

#define MOTOR_INV_SQRT_2  0.70710678118f

Motor_Control_t motor_control;

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
}

void Motor_Control_Init(void)
{
    uint8_t i;

    motor_control.update_count = 0U;
    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++) {
        motor_control.wheel_feedback[i] = 0.0f;
        USER_PID_Init(&motor_control.speed_pid[i],
                      MOTOR_SPEED_PID_KP,
                      MOTOR_SPEED_PID_KI,
                      MOTOR_SPEED_PID_KD,
                      MOTOR_SPEED_PID_OUT_MAX,
                      MOTOR_SPEED_PID_I_MAX);
    }

    Motor_Control_StopAll(MOTOR_STATE_STOPPED);
}

static void Motor_CalculateWheelTargets(float vx, float vy)
{
    const float scale = MOTOR_INV_SQRT_2 / MOTOR_WHEEL_RADIUS_M;

    /* 45-degree four-omni-wheel translation; +x forward, +y right. */
    motor_control.wheel_target[MOTOR_WHEEL_FL] = (vx + vy) * scale;
    motor_control.wheel_target[MOTOR_WHEEL_FR] = (vx - vy) * scale;
    motor_control.wheel_target[MOTOR_WHEEL_RL] = (vx - vy) * scale;
    motor_control.wheel_target[MOTOR_WHEEL_RR] = (vx + vy) * scale;
}

void Motor_Control_Update_2ms(void)
{
    uint8_t i;
    float remote_forward;
    float remote_right;

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

    remote_forward = Motor_NormalizeRemote(rc_ctrl.rc.ch1);
    remote_right = Motor_NormalizeRemote(rc_ctrl.rc.ch2);
    motor_control.vx_target = remote_forward * MOTOR_MAX_VX_MPS;
    motor_control.vy_target = remote_right * MOTOR_MAX_VY_MPS;
    Motor_CalculateWheelTargets(motor_control.vx_target,
                                motor_control.vy_target);

    motor_control.state = MOTOR_STATE_RUNNING;
    for (i = 0U; i < MOTOR_WHEEL_COUNT; i++) {
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
        Motor_Control_Update_2ms();
    }
}
