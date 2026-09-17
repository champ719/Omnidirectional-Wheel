#include "gimbal.h"
#include "Error.h"
#include "Filter.h"
#include "motor.h"
#include "PID.h"
#include "Remote.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "imu_temp_ctrl.h"
#include <math.h>

#define GIMBAL_JOYSTICK_YAW_SPEED_RADPS    6.0f   //云台yaw目标角速度
#define GIMBAL_JOYSTICK_PITCH_SPEED_RADPS  0.0f   //云台pitch目标角速度
#define GIMBAL_MOUSE_YAW_RAD_PER_COUNT     0.0015f
#define GIMBAL_MOUSE_PITCH_RAD_PER_COUNT   0.0010f
#define GIMBAL_MOUSE_MAX_DELTA_RAD         0.25f
#define GIMBAL_MOUSE_FILTER_SIZE           5U     //鼠标增量滑动平均窗口，按帧计数
#define GIMBAL_TASK_PERIOD_S               0.002f

volatile Gimbal_t gimbal;
float test ;

/* 鼠标增量每收到一帧只能吃一次，2ms 的任务会把同一帧重复叠加 */
static uint32_t gimbal_last_mouse_sequence;

/* 鼠标单帧增量抖动大，先过滑动平均再积分。滤波器只在有新帧时推进，
   所以窗口按“收到的帧数”算，和任务周期无关。 */
static AverFilter gimbal_mouse_yaw_filter;
static AverFilter gimbal_mouse_pitch_filter;

static float wrap_pi(float a)
{
  while (a > 3.14159265f)  a -= 6.28318531f;
  while (a < -3.14159265f) a += 6.28318531f;
  return a;
}

static float limit_range(float value, float minimum, float maximum)
{
  if (value > maximum) return maximum;
  if (value < minimum) return minimum;
  return value;
}

void Gimbal_Init(void)
{
  Motor_Init(&gimbal.yaw_motor, 0x1FF, DJI_6020,
    0.5f, 0.0f, 0.0f, 0.18f, 1.6f, 0.8f,
    8.0f, 0.0f, 0.02f, 0.0f,5.0f, 4.0f);

  Motor_Init(&gimbal.pitch_motor, 0x1FF, DJI_6020,
    0.1f, 0.02f, 0.05f, 0.25f, 1.6f, 0.8f,
    0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.5f);

  gimbal.yaw.zero_angle = 1.04077f;
  /* 显式对齐到当前朝向，使上电瞬间位置环误差为 0。原来靠 .bss 清零恰好等于
     IMU 上电时的 YawTotalAngle = 0 才对得上，是巧合；IMU 初始化方式一变就会
     上电抖一下。IMU 未就绪时本函数返回 0，与原来的行为一致。 */
  gimbal.yaw.target_yaw_angle = IMU_Attitude_GetYawContinuousRad();
  gimbal.pitch.zero_angle = 5.1847f;
  gimbal.pitch.target_pitch_angle = 0.0f;
  gimbal.pitch.target_pitch_w = 0.0f;
  gimbal.pitch.k_gravity_comp = 0.32f;
  gimbal.pitch.gravity_comp_offset = 0.55059f;
  gimbal.pitch.gravity_comp = 0.0f;
  gimbal_last_mouse_sequence = 0U;
  Filter_InitAverFilter(&gimbal_mouse_yaw_filter, GIMBAL_MOUSE_FILTER_SIZE);
  Filter_InitAverFilter(&gimbal_mouse_pitch_filter, GIMBAL_MOUSE_FILTER_SIZE);
}

/* 本拍的云台增量：摇杆按角速度 × 任务周期积分，鼠标按帧增量。 */
static void Gimbal_ReadCommand(float *yaw_delta, float *pitch_delta)
{
  RC_Ctrl_t remote;

  Remote_GetSnapshot(&remote);

  if (Rocker_Ctrl != 0U) {
    /* 摇杆居中时 Remote_NormalizeChannel 返回精确的 0，不需要额外死区 */
    *yaw_delta = Remote_NormalizeChannel(remote.rc.ch0) *
                 GIMBAL_JOYSTICK_YAW_SPEED_RADPS * GIMBAL_TASK_PERIOD_S * (-1.0f); 
    *pitch_delta = Remote_NormalizeChannel(remote.rc.ch1) *
                   GIMBAL_JOYSTICK_PITCH_SPEED_RADPS * GIMBAL_TASK_PERIOD_S;
    gimbal_last_mouse_sequence = remote.update_sequence;
    /* 清掉窗口里上一次键鼠模式的残留，下次切回来从零起算 */
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
  *yaw_delta = limit_range(
      Filter_AverCalc(&gimbal_mouse_yaw_filter,
                      (float)remote.mouse.x * GIMBAL_MOUSE_YAW_RAD_PER_COUNT),
      -GIMBAL_MOUSE_MAX_DELTA_RAD,
      GIMBAL_MOUSE_MAX_DELTA_RAD);
  *pitch_delta = limit_range(
      Filter_AverCalc(&gimbal_mouse_pitch_filter,
                      -(float)remote.mouse.y * GIMBAL_MOUSE_PITCH_RAD_PER_COUNT),
      -GIMBAL_MOUSE_MAX_DELTA_RAD,
      GIMBAL_MOUSE_MAX_DELTA_RAD);
}

static void Yaw_Speed_Calc(float yaw_angle_change)
{
  
  if(yaw_angle_change != 0.0f)
    gimbal.yaw.target_yaw_angle += yaw_angle_change;

  PID_SingleCalc(&gimbal.yaw_motor.pid_position, gimbal.yaw.target_yaw_angle, gimbal.yaw.yaw_angle);
  gimbal.yaw.target_yaw_w = gimbal.yaw_motor.pid_position.output;
  PID_SingleCalc(&gimbal.yaw_motor.pid_speed, gimbal.yaw_motor.pid_position.output, gimbal.yaw.fb_yaw_w);

  gimbal.yaw_motor.give_current = gimbal.yaw_motor.pid_speed.output ;
}

static void Pitch_Speed_Calc(float angle_change)
{
  gimbal.pitch.target_pitch_angle += angle_change ;

  if (gimbal.pitch.target_pitch_angle >  PITCH_MAX_ANGLE)
    gimbal.pitch.target_pitch_angle =  PITCH_MAX_ANGLE;
  if (gimbal.pitch.target_pitch_angle < PITCH_MIN_ANGLE)
    gimbal.pitch.target_pitch_angle = PITCH_MIN_ANGLE;

  gimbal.pitch.fb_pitch_w = 0.0f;
  gimbal.pitch.target_pitch_w = 0.0f;
  gimbal.pitch.gravity_comp = 0.0f;
  gimbal.pitch_motor.give_current = 0.0f;

  /* 不驱动也要清积分，避免重新使能时把积攒的量一次性打出去 */
  PID_Clear(&gimbal.pitch_motor.pid_position);
  PID_Clear(&gimbal.pitch_motor.pid_speed);

  /* 以下为电机可用时的原控制律，暂不执行：
  PID_SingleCalc(&gimbal.pitch_motor.pid_position, gimbal.pitch.target_pitch_angle, gimbal.pitch.pitch_angle);
  gimbal.pitch.fb_pitch_w = gimbal.pitch_motor.fb_speed;
  gimbal.pitch.target_pitch_w = gimbal.pitch_motor.pid_position.output;
  PID_SingleCalc(&gimbal.pitch_motor.pid_speed, gimbal.pitch.target_pitch_w, gimbal.pitch.fb_pitch_w);
  gimbal.pitch.gravity_comp = 0.2f * gimbal.pitch.gravity_comp + gimbal.pitch.k_gravity_comp * 0.8f * cosf(gimbal.pitch.pitch_angle - gimbal.pitch.gravity_comp_offset);
  gimbal.pitch_motor.give_current = -gimbal.pitch.gravity_comp + gimbal.pitch_motor.pid_speed.output;
  */
}

void Gimbal_Update(void)
{
  float gyro_body[3];
  float yaw_delta = 0.0f;
  float pitch_delta = 0.0f;

  Gimbal_ReadCommand(&yaw_delta, &pitch_delta);

  IMU_Attitude_GetGyroBody(gyro_body);
  gimbal.gyro.yaw_w = gyro_body[2];
  gimbal.gyro.pitch_w = gyro_body[1];
  gimbal.pitch.pitch_angle = wrap_pi(gimbal.pitch_motor.total_angle - gimbal.pitch.zero_angle);
  gimbal.yaw.yaw_angle = IMU_Attitude_GetYawContinuousRad();
  gimbal.yaw.fb_yaw_w = - gimbal.gyro.yaw_w;
  gimbal.yaw.machine_yaw_angle = wrap_pi(gimbal.yaw_motor.fb_angle - gimbal.yaw.zero_angle);

  Yaw_Speed_Calc(yaw_delta);
  Pitch_Speed_Calc(pitch_delta);
}

/************************freertos任务****************************/

/**
 * @brief 云台任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 * @note 先等 1.2s 让 IMU 上电稳定，再初始化云台并进 2ms 控制环。
 *       算出的 give_current 由 MotorTask 发 CAN。
 */
void OS_GimbalCallback(void const *argument)
{
  (void)argument;

  osDelay(1200);
  Gimbal_Init();
  for (;;)
  {
    /* 故障状态下不发运动指令，只清输出 */
    if (Error_GetResult() != ERROR_RESULT_NONE) {
      gimbal.yaw_motor.give_current = 0.0f;
      gimbal.pitch_motor.give_current = 0.0f;
    } else {
      Gimbal_Update();
    }
    osDelay(2);
  }
}
