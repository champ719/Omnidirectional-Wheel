#include "gimbal.h"
#include "Error.h"
#include "Filter.h"
#include "motor.h"
#include "PID.h"
#include "Remote.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "imu_temp_ctrl.h"
#include "task.h"
#include <math.h>

static const float gimbal_task_period_s = 0.002f;

Gimbal_t gimbal = {0};

/* 鼠标增量每收到一帧只能吃一次，2ms 的任务会把同一帧重复叠加 */
static uint32_t gimbal_last_mouse_sequence;
static float gimbal_last_yaw_angle;

/* 鼠标单帧增量抖动大，先过滑动平均再积分。滤波器只在有新帧时推进，
   所以窗口按“收到的帧数”算，和任务周期无关。 */
static AverFilter gimbal_mouse_yaw_filter;
static AverFilter gimbal_mouse_pitch_filter;
static volatile uint8_t gimbal_hold_yaw_pending;
static volatile uint32_t gimbal_hold_yaw_imu_sequence;
static volatile uint8_t gimbal_initialized;

/* 将角度限制到 [-pi, pi]。 */
static float wrap_pi(float angle)
{
  while (angle > 3.14159265f) {
    angle -= 6.28318531f;
  }
  while (angle < -3.14159265f) {
    angle += 6.28318531f;
  }
  return angle;
}

/* 将数值限制在指定闭区间内。 */
static float limit_range(float value, float minimum, float maximum)
{
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}

/* 初始化云台电机、零点、滤波器和当前目标角。 */
void Gimbal_Init(void)
{
  RC_Ctrl_t remote;
  float current_yaw;

  gimbal_initialized = 0U;

  Motor_Init(&gimbal.yaw_motor, 0x1FF, DJI_6020, 0.5f, 0.0f, 0.0f, 0.18f, 1.6f, 0.8f, 8.0f, 0.0f, 0.02f, 0.0f, 5.0f, 4.0f);
  Motor_Init(&gimbal.pitch_motor, 0x1FF, DJI_6020, 0.1f, 0.02f, 0.05f, 0.25f, 1.6f, 0.8f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.5f);

  gimbal.yaw.zero_angle = 1.04077f;
  /* 任务入口会等待 IMU 就绪；这里对齐当前朝向，使首次位置环误差为 0。 */
  current_yaw = IMU_Attitude_GetYawContinuousRad();
  gimbal.yaw.yaw_angle = current_yaw;
  gimbal.yaw.target_yaw_angle = current_yaw;
  gimbal_last_yaw_angle = current_yaw;
  gimbal.pitch.zero_angle = 5.1847f;
  gimbal.pitch.k_gravity_comp = 0.32f;
  gimbal.pitch.gravity_comp_offset = 0.55059f;

  Remote_GetSnapshot(&remote);
  gimbal_last_mouse_sequence = remote.update_sequence;
  Filter_InitAverFilter(&gimbal_mouse_yaw_filter, 5U);
  Filter_InitAverFilter(&gimbal_mouse_pitch_filter, 5U);
  gimbal_initialized = 1U;
  Motor_NotifyControlInitialized();
}

/* 返回云台控制模块是否完成初始化。 */
uint8_t Gimbal_IsInitialized(void)
{
  return gimbal_initialized;
}

/* 急停解除后等待新 IMU 数据并保持当前偏航角。 */
void Gimbal_HoldCurrentYawAfterEmergencyStop(void)
{
  gimbal_hold_yaw_imu_sequence = IMU_Attitude_GetUpdateSequence();
  gimbal_hold_yaw_pending = 1U;
  gimbal.yaw.target_yaw_w = 0.0f;
  gimbal.yaw_motor.give_current = 0.0f;
  PID_Clear(&gimbal.yaw_motor.pid_position);
  PID_Clear(&gimbal.yaw_motor.pid_speed);
}

/* 本拍的云台增量：摇杆按角速度 × 任务周期积分，鼠标按帧增量。 */
static void Gimbal_ReadCommand(float *yaw_delta, float *pitch_delta)
{
  RC_Ctrl_t remote;

  Remote_GetSnapshot(&remote);

  if (Rocker_Ctrl != 0U) {

    *yaw_delta = Remote_NormalizeChannel(remote.rc.ch0) * 4.0f * gimbal_task_period_s * (-1.0f);
    *pitch_delta = Remote_NormalizeChannel(remote.rc.ch1) * 0.0f * gimbal_task_period_s;
    gimbal_last_mouse_sequence = remote.update_sequence;

    Filter_AverClear(&gimbal_mouse_yaw_filter);
    Filter_AverClear(&gimbal_mouse_pitch_filter);
    return;
  }

  if (remote.update_sequence == gimbal_last_mouse_sequence) {
    return;
  }
  gimbal_last_mouse_sequence = remote.update_sequence;

  /* 先按帧增量缩放再过滑动平均，最后卡一道硬上限挡住异常大帧。
     滤波器对增量的直流增益为 1，积分下来总角度不会被拉偏。 */
  *yaw_delta = limit_range(Filter_AverCalc(&gimbal_mouse_yaw_filter, -(float)remote.mouse.x * 0.0030f), -0.25f, 0.25f);
  *pitch_delta = limit_range(Filter_AverCalc(&gimbal_mouse_pitch_filter, -(float)remote.mouse.y * 0.0030f), -0.25f, 0.25f);
}

/* 计算 yaw 电流，并在无输入且居中时阻断累计零漂。 */
static void Yaw_Speed_Calc(float yaw_angle_change, float yaw_measurement_change)
{
  if ((yaw_angle_change == 0.0f) && (Chassis_IsFollowCentered() != 0U)) {
    gimbal.yaw.target_yaw_angle += yaw_measurement_change;
    gimbal.yaw.target_yaw_w = 0.0f;
    gimbal.yaw_motor.give_current = 0.0f;
    PID_Clear(&gimbal.yaw_motor.pid_position);
    PID_Clear(&gimbal.yaw_motor.pid_speed);
    return;
  }

  if (yaw_angle_change != 0.0f) {
    gimbal.yaw.target_yaw_angle += yaw_angle_change;
  }

  PID_SingleCalc(&gimbal.yaw_motor.pid_position, gimbal.yaw.target_yaw_angle, gimbal.yaw.yaw_angle);
  gimbal.yaw.target_yaw_w = gimbal.yaw_motor.pid_position.output;
  PID_SingleCalc(&gimbal.yaw_motor.pid_speed, gimbal.yaw_motor.pid_position.output, gimbal.yaw.fb_yaw_w);

  gimbal.yaw_motor.give_current = gimbal.yaw_motor.pid_speed.output;
}

/* 计算俯仰轴位置环、速度环和重力补偿电流。 */
static void Pitch_Speed_Calc(float angle_change)
{
  gimbal.pitch.target_pitch_angle += angle_change;

  if (gimbal.pitch.target_pitch_angle > 0.5345) {
    gimbal.pitch.target_pitch_angle = 0.5345;
  }
  if (gimbal.pitch.target_pitch_angle < -0.3627) {
    gimbal.pitch.target_pitch_angle = -0.3627;
  }

  gimbal.pitch.fb_pitch_w = 0.0f;
  gimbal.pitch.target_pitch_w = 0.0f;
  gimbal.pitch.gravity_comp = 0.0f;
  gimbal.pitch_motor.give_current = 0.0f;
  PID_Clear(&gimbal.pitch_motor.pid_position);
  PID_SingleCalc(&gimbal.pitch_motor.pid_position, gimbal.pitch.target_pitch_angle, gimbal.pitch.pitch_angle);
  gimbal.pitch.fb_pitch_w = gimbal.pitch_motor.fb_speed;
  gimbal.pitch.target_pitch_w = gimbal.pitch_motor.pid_position.output;
  PID_SingleCalc(&gimbal.pitch_motor.pid_speed, gimbal.pitch.target_pitch_w, gimbal.pitch.fb_pitch_w);
  gimbal.pitch.gravity_comp = 0.2f * gimbal.pitch.gravity_comp + gimbal.pitch.k_gravity_comp * 0.8f * cosf(gimbal.pitch.pitch_angle - gimbal.pitch.gravity_comp_offset);
  gimbal.pitch_motor.give_current = -gimbal.pitch.gravity_comp + gimbal.pitch_motor.pid_speed.output;
}

/* 更新云台反馈并执行一拍偏航、俯仰控制。 */
void Gimbal_Update(void)
{
  float gyro_body[3];
  float yaw_delta = 0.0f;
  float pitch_delta = 0.0f;
  float yaw_measurement_change;

  Gimbal_ReadCommand(&yaw_delta, &pitch_delta);

  IMU_Attitude_GetGyroBody(gyro_body);
  gimbal.gyro.yaw_w = gyro_body[2];
  gimbal.gyro.pitch_w = gyro_body[1];
  gimbal.pitch.pitch_angle = wrap_pi(gimbal.pitch_motor.total_angle - gimbal.pitch.zero_angle);
  gimbal.yaw.yaw_angle = IMU_Attitude_GetYawContinuousRad();
  yaw_measurement_change = gimbal.yaw.yaw_angle - gimbal_last_yaw_angle;
  gimbal_last_yaw_angle = gimbal.yaw.yaw_angle;
  gimbal.yaw.fb_yaw_w = -gimbal.gyro.yaw_w;
  gimbal.yaw.machine_yaw_angle = wrap_pi(gimbal.yaw_motor.fb_angle - gimbal.yaw.zero_angle);

  Yaw_Speed_Calc(yaw_delta, yaw_measurement_change);
  Pitch_Speed_Calc(pitch_delta);
}

/************************freertos任务****************************/

/**
 * @brief 云台任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 * @note 阻塞等待IMU完成初始化和静态校准，再初始化云台并进入2ms控制环。
 *       算出的give_current由MotorTask统一发送。
 */
void OS_GimbalCallback(void const *argument)
{
  TickType_t last_wake;
  const TickType_t period = pdMS_TO_TICKS(2U);

  (void)argument;

  IMU_Attitude_WaitUntilReady();
  Gimbal_Init();
  last_wake = xTaskGetTickCount();
  for (;;) {
    TickType_t now = xTaskGetTickCount();

    if ((now - last_wake) > (period * 2U)) {
      last_wake = now;
    }
    /* 故障状态下不发运动指令，只清输出 */
    if (Error_GetResult() != ERROR_RESULT_NONE) {
      gimbal.yaw_motor.give_current = 0.0f;
      gimbal.pitch_motor.give_current = 0.0f;
    } else if (gimbal_hold_yaw_pending != 0U) {
      if (IMU_Attitude_GetUpdateSequence() != gimbal_hold_yaw_imu_sequence) {
        RC_Ctrl_t remote;
        float current_yaw = IMU_Attitude_GetYawContinuousRad();

        gimbal.yaw.yaw_angle = current_yaw;
        gimbal.yaw.target_yaw_angle = current_yaw;
        gimbal_last_yaw_angle = current_yaw;
        gimbal.yaw.target_yaw_w = 0.0f;
        gimbal.yaw.fb_yaw_w = 0.0f;
        gimbal.yaw_motor.give_current = 0.0f;

        PID_Clear(&gimbal.yaw_motor.pid_position);
        PID_Clear(&gimbal.yaw_motor.pid_speed);
        Remote_GetSnapshot(&remote);
        gimbal_last_mouse_sequence = remote.update_sequence;
        Filter_AverClear(&gimbal_mouse_yaw_filter);
        Filter_AverClear(&gimbal_mouse_pitch_filter);
        gimbal_hold_yaw_pending = 0U;
      }
    } else {
      Gimbal_Update();
    }
    vTaskDelayUntil(&last_wake, period);
  }
}
