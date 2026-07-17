#include "Robot_Control.h"
#include "Chassis.h"
#include "Error.h"
#include "Gimbal.h"
#include "Motor_Drv.h"
#include "Remote.h"
#include "Rotate.h"
#include "imu_attitude.h"

typedef struct
{
    float remote_forward;
    float remote_right;
    float remote_clockwise;
    float remote_pitch;
    float yaw_feedback;
    float yaw_speed_feedback;
    float pitch_feedback;
    float pitch_speed_feedback;
    float wheel_feedback[CHASSIS_WHEEL_COUNT];
    uint8_t rotate_requested;
} Robot_Control_Input_t;

Robot_Control_t robot_control;

static Robot_Control_State_t Robot_Control_MapError(Error_Result_t error)
{
    switch (error) {
    case ERROR_RESULT_STOPPED:
        return ROBOT_STATE_STOPPED;
    case ERROR_RESULT_REMOTE_OFFLINE:
        return ROBOT_STATE_REMOTE_OFFLINE;
    case ERROR_RESULT_EMERGENCY_STOP:
        return ROBOT_STATE_EMERGENCY_STOP;
    case ERROR_RESULT_DIRECTION_UNCALIBRATED:
        return ROBOT_STATE_DIRECTION_UNCALIBRATED;
    case ERROR_RESULT_IMU_NOT_READY:
        return ROBOT_STATE_IMU_NOT_READY;
    case ERROR_RESULT_NONE:
    default:
        return ROBOT_STATE_RUNNING;
    }
}

static void Robot_Control_ReadInputs(Robot_Control_Input_t *input)
{
    uint8_t i;

    input->remote_forward = Remote_NormalizeChannel(rc_ctrl.rc.ch3);
    input->remote_right = Remote_NormalizeChannel(rc_ctrl.rc.ch2);
    input->remote_clockwise = Remote_NormalizeChannel(rc_ctrl.rc.ch0);
    input->remote_pitch = Remote_NormalizeChannel(rc_ctrl.rc.ch1);
    input->yaw_feedback = (float)gimbal_motors[0].Rx_Data.Position;
    input->yaw_speed_feedback = gimbal_motors[0].Rx_Data.Velocity;
    input->pitch_feedback = (float)gimbal_motors[1].Rx_Data.Position;
    input->pitch_speed_feedback = gimbal_motors[1].Rx_Data.Velocity;
    input->rotate_requested =
        (rc_ctrl.rc.s2 == RC_SW_DOWN) ? 1U : 0U;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) {
        input->wheel_feedback[i] = wheel_motors[i].Rx_Data.Velocity;
    }
}

static void Robot_Control_Stop(Robot_Control_State_t reason)
{
    robot_control.state = reason;
    Chassis_Ctrl_Stop();
    Gimbal_Ctrl_Stop();
    Motor_Drv_StopAll();
}

static void Robot_Control_UpdateNormal(const Robot_Control_Input_t *input)
{
    Chassis_Ctrl_SetRemoteTargets(input->remote_forward,
                                  input->remote_right,
                                  input->remote_clockwise);
    Chassis_Ctrl_CalculateWheelTargets(0U);
    Gimbal_Ctrl_UpdateNormal(input->remote_pitch,
                             input->yaw_feedback,
                             input->yaw_speed_feedback,
                             input->pitch_feedback,
                             input->pitch_speed_feedback,
                             ROBOT_CONTROL_PERIOD_S);
}

static Robot_Control_State_t Robot_Control_UpdateMode(
    const Robot_Control_Input_t *input,
    Robot_Control_State_t previous_state)
{
    if (input->rotate_requested != 0U) {
        Rotate_UpdateActive(input->remote_forward,
                            input->remote_right,
                            input->remote_clockwise,
                            input->remote_pitch,
                            input->yaw_feedback,
                            input->yaw_speed_feedback,
                            input->pitch_feedback,
                            input->pitch_speed_feedback,
                            imu_attitude.yaw_continuous,
                            ROBOT_CONTROL_PERIOD_S);
        return ROBOT_STATE_SMALL_GYRO;
    }

    if ((previous_state == ROBOT_STATE_SMALL_GYRO) ||
        (previous_state == ROBOT_STATE_GYRO_EXIT_ALIGN)) {
        if (Rotate_UpdateExit(input->remote_pitch,
                              input->yaw_feedback,
                              input->yaw_speed_feedback,
                              input->pitch_feedback,
                              input->pitch_speed_feedback,
                              imu_attitude.yaw_continuous,
                              input->wheel_feedback,
                              ROBOT_CONTROL_PERIOD_S) == 0U) {
            return ROBOT_STATE_GYRO_EXIT_ALIGN;
        }
    }

    Robot_Control_UpdateNormal(input);
    return ROBOT_STATE_RUNNING;
}

void Robot_Control_Init(void)
{
    Chassis_Ctrl_Init();
    Gimbal_Ctrl_Init();
    Error_Init();
    Motor_Drv_StopAll();
    robot_control.state = ROBOT_STATE_STOPPED;
}

void Robot_Control_Update_2ms(void)
{
    Robot_Control_Input_t input;
    Robot_Control_State_t previous_state;
    Robot_Control_State_t requested_state;
    Error_Result_t error;

    previous_state = robot_control.state;
    error = Error_GetResult();
    if (error != ERROR_RESULT_NONE) {
        Robot_Control_Stop(Robot_Control_MapError(error));
        return;
    }

    Robot_Control_ReadInputs(&input);
    requested_state = Robot_Control_UpdateMode(&input, previous_state);

    if (previous_state != requested_state) {
        Chassis_Ctrl_ResetWheelPIDs();
    }

    robot_control.state = requested_state;
    Chassis_Ctrl_UpdateOutputs(input.wheel_feedback,
                               ROBOT_CONTROL_PERIOD_S);
}
