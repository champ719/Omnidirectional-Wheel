#include "Robot_Control.h"
#include "Buzzer.h"
#include "Chassis.h"
#include "Error.h"
#include "FreeRTOS.h"
#include "Gimbal.h"
#include "Remote.h"
#include "USER_CAN.h"
#include "motor.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define ROBOT_JOYSTICK_YAW_SPEED_RADPS       10.0f//云台yaw目标角速度
#define ROBOT_JOYSTICK_PITCH_SPEED_RADPS      2.3f//云台pitch目标角速度
#define ROBOT_MOUSE_YAW_RAD_PER_COUNT         0.0015f//鼠标y轴灵敏度
#define ROBOT_MOUSE_PITCH_RAD_PER_COUNT       0.0010f//鼠标x轴灵敏度
#define ROBOT_MOUSE_MAX_DELTA_RAD              0.25f
#define ROBOT_KEYBOARD_NORMAL_SPEED_SCALE      0.60f
#define ROBOT_KEYBOARD_FAST_SPEED_SCALE        1.00f//shift加速
#define ROBOT_KEYBOARD_SLOW_SPEED_SCALE        0.30f//ctrl减速

Robot_Control_t robot_control;

static uint32_t robot_last_mouse_sequence;
static Robot_ControlSource_t robot_previous_source;

// 将数值限制在指定的最小值和最大值之间。
static float Robot_Control_Limit(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

// 判断指定按键是否处于按下状态。
static uint8_t Robot_Control_KeyPressed(uint16_t keys, uint16_t key)
{
    return ((keys & key) != 0U) ? 1U : 0U;
}

// 根据遥控器拨杆状态选择底盘控制模式。
static Robot_ChassisMode_t Robot_Control_JoystickMode(uint8_t switch_value)
{
    if (switch_value == RC_SW_UP) {
        return ROBOT_CHASSIS_GYRO_CCW;
    }
    if (switch_value == RC_SW_DOWN) {
        return ROBOT_CHASSIS_GYRO_CW;
    }
    return ROBOT_CHASSIS_FOLLOW;
}

// 根据键盘 Q/E 按键选择底盘控制模式。
static Robot_ChassisMode_t Robot_Control_KeyboardMode(uint16_t keys)
{
    uint8_t counterclockwise = Robot_Control_KeyPressed(keys, RC_KEY_Q);
    uint8_t clockwise = Robot_Control_KeyPressed(keys, RC_KEY_E);

    if (counterclockwise == clockwise) {
        return ROBOT_CHASSIS_FOLLOW;
    }
    return (counterclockwise != 0U) ?
        ROBOT_CHASSIS_GYRO_CCW : ROBOT_CHASSIS_GYRO_CW;
}

// 根据键盘 Shift/Ctrl 按键选择底盘速度比例。
static float Robot_Control_KeyboardSpeedScale(uint16_t keys)
{
    if (Robot_Control_KeyPressed(keys, RC_KEY_CTRL) != 0U) {
        return ROBOT_KEYBOARD_SLOW_SPEED_SCALE;
    }
    if (Robot_Control_KeyPressed(keys, RC_KEY_SHIFT) != 0U) {
        return ROBOT_KEYBOARD_FAST_SPEED_SCALE;
    }
    return ROBOT_KEYBOARD_NORMAL_SPEED_SCALE;
}

// 将前后和左右输入归一化，避免合成后的平移幅值超过限制。
static void Robot_Control_NormalizeTranslation(float *forward, float *right)
{
    float magnitude = sqrtf((*forward * *forward) + (*right * *right));

    if (magnitude > 1.0f) {
        *forward /= magnitude;
        *right /= magnitude;
    }
}

// 将遥控器摇杆输入转换为机器人控制指令。
static void Robot_Control_FromJoystick(const RC_Ctrl_t *remote)
{
    Robot_Command_t *command = &robot_control.command;

    command->source = ROBOT_CONTROL_SOURCE_JOYSTICK;
    command->chassis_mode = Robot_Control_JoystickMode(remote->rc.s2);
    command->forward = Remote_NormalizeChannel(remote->rc.ch3);
    command->right = Remote_NormalizeChannel(remote->rc.ch2);
    Robot_Control_NormalizeTranslation(&command->forward, &command->right);
    command->gimbal_yaw_delta_rad =
        Remote_NormalizeChannel(remote->rc.ch0) *
        ROBOT_JOYSTICK_YAW_SPEED_RADPS * ROBOT_CONTROL_PERIOD_S;
    command->gimbal_pitch_delta_rad =
        Remote_NormalizeChannel(remote->rc.ch1) *
        ROBOT_JOYSTICK_PITCH_SPEED_RADPS * ROBOT_CONTROL_PERIOD_S;
    command->speed_scale = 1.0f;
}

// 将键盘和鼠标输入转换为机器人控制指令。
static void Robot_Control_FromKeyboardMouse(const RC_Ctrl_t *remote)
{
    Robot_Command_t *command = &robot_control.command;
    uint16_t keys = remote->key.v;
    float forward =
        (float)Robot_Control_KeyPressed(keys, RC_KEY_W) -
        (float)Robot_Control_KeyPressed(keys, RC_KEY_S);
    float right =
        (float)Robot_Control_KeyPressed(keys, RC_KEY_D) -
        (float)Robot_Control_KeyPressed(keys, RC_KEY_A);

    Robot_Control_NormalizeTranslation(&forward, &right);
    command->source = ROBOT_CONTROL_SOURCE_KEYBOARD_MOUSE;
    command->chassis_mode = Robot_Control_KeyboardMode(keys);
    command->speed_scale = Robot_Control_KeyboardSpeedScale(keys);
    command->forward = forward * command->speed_scale;
    command->right = right * command->speed_scale;

    if (remote->update_sequence != robot_last_mouse_sequence) {
        command->gimbal_yaw_delta_rad = Robot_Control_Limit(
            (float)remote->mouse.x * ROBOT_MOUSE_YAW_RAD_PER_COUNT,
            -ROBOT_MOUSE_MAX_DELTA_RAD,
            ROBOT_MOUSE_MAX_DELTA_RAD);
        command->gimbal_pitch_delta_rad = Robot_Control_Limit(
            -(float)remote->mouse.y * ROBOT_MOUSE_PITCH_RAD_PER_COUNT,
            -ROBOT_MOUSE_MAX_DELTA_RAD,
            ROBOT_MOUSE_MAX_DELTA_RAD);
        robot_last_mouse_sequence = remote->update_sequence;
    }
}

// 读取遥控器快照并更新当前机器人控制指令。
static void Robot_Control_UpdateCommand(void)
{
    RC_Ctrl_t remote;
    Robot_ControlSource_t source;

    Remote_GetSnapshot(&remote);
    source = (remote.rc.s1 == RC_SW_MID) ?
        ROBOT_CONTROL_SOURCE_JOYSTICK :
        (remote.rc.s1 == RC_SW_UP) ?
        ROBOT_CONTROL_SOURCE_KEYBOARD_MOUSE :
        ROBOT_CONTROL_SOURCE_NONE;

    memset(&robot_control.command, 0, sizeof(robot_control.command));
    robot_control.command.source = source;
    robot_control.command.chassis_mode = ROBOT_CHASSIS_FOLLOW;
    robot_control.command.input_tick = remote.update_tick;
    robot_control.command.input_sequence = remote.update_sequence;

    if (source == ROBOT_CONTROL_SOURCE_JOYSTICK) {
        Robot_Control_FromJoystick(&remote);
        robot_last_mouse_sequence = remote.update_sequence;
    } else if (source == ROBOT_CONTROL_SOURCE_KEYBOARD_MOUSE) {
        if (robot_previous_source != ROBOT_CONTROL_SOURCE_KEYBOARD_MOUSE) {
            robot_last_mouse_sequence = remote.update_sequence;
        } else {
            Robot_Control_FromKeyboardMouse(&remote);
        }
    }

    robot_previous_source = source;
}

// 将系统错误结果转换为机器人控制状态。
static Robot_ControlState_t Robot_Control_MapError(Error_Result_t error)
{
    switch (error) {
    case ERROR_RESULT_STOPPED:
        return ROBOT_STATE_STOPPED;
    case ERROR_RESULT_REMOTE_OFFLINE:
        return ROBOT_STATE_REMOTE_OFFLINE;
    case ERROR_RESULT_EMERGENCY_STOP:
        return ROBOT_STATE_EMERGENCY_STOP;
    case ERROR_RESULT_IMU_NOT_READY:
        return ROBOT_STATE_IMU_NOT_READY;
    case ERROR_RESULT_MOTOR_FEEDBACK_TIMEOUT:
        return ROBOT_STATE_MOTOR_OFFLINE;
    case ERROR_RESULT_CAN_FAULT:
        return ROBOT_STATE_CAN_FAULT;
    case ERROR_RESULT_NONE:
    default:
        return ROBOT_STATE_RUNNING;
    }
}

// 初始化机器人控制模块及其底盘、云台和外设子模块。
void Robot_Control_Init(void)
{
    memset(&robot_control, 0, sizeof(robot_control));
    robot_control.state = ROBOT_STATE_STOPPED;
    robot_control.command.source = ROBOT_CONTROL_SOURCE_NONE;
    robot_control.command.chassis_mode = ROBOT_CHASSIS_FOLLOW;
    robot_previous_source = ROBOT_CONTROL_SOURCE_NONE;
    robot_last_mouse_sequence = 0U;

    Chassis_Init();
    Gimbal_Init();
    CAN_Init();
    Remote_Init();
    Buzzer_Init();
    Error_Init();
}

// 执行一轮机器人控制流程，包括输入处理、故障检查、底盘和云台控制以及电机输出。
void Robot_Control_Update(void)
{
    Error_Result_t error;

    Robot_Control_UpdateCommand();
    CAN_Service();
    Error_MonitorUpdate();
    error = Error_GetResult();

    if (error != ERROR_RESULT_NONE) {
        robot_control.state = Robot_Control_MapError(error);
        Chassis_ResetControl();
        Motor_STOP();
        if (error == ERROR_RESULT_EMERGENCY_STOP) {
            Error_TriggerEmergencyStop();
        }
        return;
    }

    if ((Remote_IsOnline() == 0U) ||
        (robot_control.command.source == ROBOT_CONTROL_SOURCE_NONE)) {
        robot_control.state = ROBOT_STATE_STOPPED;
        Chassis_ResetControl();
        Motor_STOP();
        return;
    }

    Chassis_Update();
    Gimbal_Update();
    robot_control.state =
        (robot_control.command.chassis_mode == ROBOT_CHASSIS_FOLLOW) ?
        ROBOT_STATE_RUNNING : ROBOT_STATE_SMALL_GYRO;
    Motor_UPDATE();
}

// FreeRTOS 电机任务回调，按固定周期执行机器人控制流程。
void OS_MotorCallback(void const *argument)
{
    TickType_t last_wake;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;) {
        Robot_Control_Update();
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS((uint32_t)(ROBOT_CONTROL_PERIOD_S * 1000.0f)));
    }
}
