#ifndef CHASSIS_H
#define CHASSIS_H

#include "USER_PID.h"
#include <stdint.h>

#define CHASSIS_WHEEL_RADIUS_M              0.075f                 /* 底盘车轮半径，单位：m。 */
#define CHASSIS_HALF_LENGTH_M               0.300f                 /* 底盘中心到前后轮轴的距离，单位：m。 */
#define CHASSIS_HALF_WIDTH_M                0.300f                 /* 底盘中心到左右车轮的距离，单位：m。 */
#define CHASSIS_MAX_VX_MPS                  6.0f                   /* 底盘前后方向最大速度，单位：m/s。 */
#define CHASSIS_MAX_VY_MPS                  6.0f                   /* 底盘左右方向最大速度，单位：m/s。 */
#define CHASSIS_MAX_WZ_RADPS                3.0f                   /* 底盘最大旋转角速度，单位：rad/s。 */
#define CHASSIS_SMALL_GYRO_WZ_RADPS         6.0f                   /* 小陀螺模式的固定旋转角速度。 */
#define CHASSIS_DIRECTION_FL                (+1.0f)                /* 左前轮安装方向修正系数。 */
#define CHASSIS_DIRECTION_FR                (-1.0f)                /* 右前轮安装方向修正系数。 */
#define CHASSIS_DIRECTION_RL                (+1.0f)                /* 左后轮安装方向修正系数。 */
#define CHASSIS_DIRECTION_RR                (-1.0f)                /* 右后轮安装方向修正系数。 */

typedef enum
{
    CHASSIS_WHEEL_FL = 0,
    CHASSIS_WHEEL_FR,
    CHASSIS_WHEEL_RL,
    CHASSIS_WHEEL_RR,
    CHASSIS_WHEEL_COUNT
} Chassis_WheelIndex_t;

typedef struct
{
    volatile float vx_target;
    volatile float vy_target;
    volatile float wz_target;
    volatile float wheel_target[CHASSIS_WHEEL_COUNT];
    volatile float wheel_feedback[CHASSIS_WHEEL_COUNT];
    volatile float wheel_output[CHASSIS_WHEEL_COUNT];
    volatile float exit_align_error_count;
    volatile float exit_align_remaining_count;
    volatile float exit_align_last_encoder_count;
    volatile float exit_align_direction_accumulator;
    volatile float exit_align_encoder_direction;
    volatile float exit_align_wz_direction;
    volatile uint16_t exit_align_direction_detect_cycles;
    volatile uint16_t exit_align_stable_count;
    volatile uint8_t exit_align_direction_detected;
    volatile uint8_t exit_align_initialized;
    USER_PID_t speed_pid[CHASSIS_WHEEL_COUNT];
} Chassis_Control_t;

extern Chassis_Control_t chassis_control;

void Chassis_Ctrl_Init(void);
void Chassis_Ctrl_Stop(void);
void Chassis_Ctrl_ResetWheelPIDs(void);
void Chassis_Ctrl_ResetRotateExit(void);
void Chassis_Ctrl_PrepareRotate(void);
uint8_t Chassis_Ctrl_DirectionsCalibrated(void);
void Chassis_Ctrl_SetRemoteTargets(float remote_forward,
                                   float remote_right,
                                   float remote_clockwise);
void Chassis_Ctrl_SetRotateTargets(float remote_forward,
                                   float remote_right,
                                   float yaw_feedback);
uint8_t Chassis_Ctrl_UpdateRotateExit(float yaw_feedback,
                                      const float wheel_feedback[CHASSIS_WHEEL_COUNT]);
void Chassis_Ctrl_CalculateWheelTargets(uint8_t preserve_rotation);
void Chassis_Ctrl_UpdateOutputs(const float wheel_feedback[CHASSIS_WHEEL_COUNT],
                                float dt);

#endif
