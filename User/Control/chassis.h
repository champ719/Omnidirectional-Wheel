#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "motor.h"
#include "Slope.h"
#include "Power_Control.h"

#define LF  0U
#define RF  1U
#define LB  2U
#define RB  3U

typedef enum
{
    ChassisMode_Follow = 0, /* 底盘跟随云台模式。 */
    ChassisMode_SpinLeft,   /* 小陀螺逆时针模式。 */
    ChassisMode_SpinRight,  /* 小陀螺顺时针模式。 */
} Chassis_Mode_e;

/* 人控/AI 控来源。本车只有人控，pattern 恒为 Chassis_control，
   保留结构是为了和参考工程的状态机分支对齐。 */
typedef enum
{
    Chassis_control,
} Chassis_Pattern_e;

typedef struct _Chassis
{
    /* 底盘尺寸信息。 */
    struct Info
    {
        float wheelbase;   /* 轴距，单位 m。 */
        float wheeltrack;  /* 轮距，单位 m。 */
        float wheelRadius; /* 轮半径，单位 m。 */
        float offsetX;     /* 重心在 x 轴上的偏移，单位 m。 */
        float offsetY;     /* 重心在 y 轴上的偏移，单位 m。 */
        float R;           /* 车轮到中心的距离，单位 m。 */
        float rpm_ratio;   /* 线速度到输出轴角速度的换算系数，等于 1/轮半径。 */
    } info;

    Motor_t motor_3508[4];

    /* 底盘移动状态。 */
    struct Move
    {
        float vx; /* 当前 x 方向速度，单位 m/s。 */
        float vy; /* 当前 y 方向速度，单位 m/s。 */
        float vw; /* 当前旋转角速度，单位 rad/s。 */
        float maxVx;
        float maxVy;
        float maxVw;
        float real_vx;
        float real_vy;
        float real_vw;
        PID real_xPID;
        PID real_yPID;
        PID real_wPID;
        float maxPower;
        Slope xSlope;
        Slope ySlope;
        Slope outputSlope;
        Slope chargeSlope;
        Slope spinSlope;
        uint8_t fastMode;
    } move;

    struct
    {
        int8_t key_w;
        int8_t key_a;
        int8_t key_s;
        int8_t key_d;
    } key;

    /* 底盘旋转状态。 */
    struct
    {
        PID pid;                /* 根据 relativeAngle 计算底盘旋转速度。 */
        float relativeAngle;    /* 云台相对底盘偏航角，单位 rad。 */
        Chassis_Mode_e mode;    /* 跟随或小陀螺模式。 */
        float ratio;            /* 小陀螺旋转速度比例。 */
    } rotate;

    struct
    {
        float pitchTiltAngle;
        float rollTiltAngle;
    } angle;
    Chassis_Pattern_e pattern;

    /* 功率计/超级电容反馈（CAN 0x213），写入见 USER_CAN.c。 */
    struct
    {
        volatile float voltage; /* 四轮功率计母线电压，单位 V。 */
        volatile float current; /* 四轮功率计总电流，单位 A。 */
        volatile float power;   /* 四轮总功率滤波值，单位 W。 */
        volatile uint32_t update_tick;
        volatile uint32_t update_sequence;
        volatile uint8_t received;
    } power_fb;

    /* 功率预测模型参数与状态，定义见 Power_Control.c。 */
    ChassisPowerControl_t power_prediction;
} Chassis_t;

extern Chassis_t chassis;

/* 初始化底盘控制。 */
void Chassis_Init(void);
/* 返回底盘是否完成初始化。 */
uint8_t Chassis_IsInitialized(void);
/* 判断底盘是否处于跟随模式的中心死区。 */
uint8_t Chassis_IsFollowCentered(void);
/* 清空底盘运动控制状态。 */
void Chassis_ResetControl(void);
/* 更新底盘坐标系下的运动目标。 */
void Chassis_UpdateMove(void);
/* 执行一拍底盘控制。 */
void Task_Chassis_Callback(void);

/* FreeRTOS 底盘任务入口：2 ms 一轮，解算轮速写入 motor_3508[].target_speed。 */
void OS_ChassisCallback(void const *argument);

#endif
