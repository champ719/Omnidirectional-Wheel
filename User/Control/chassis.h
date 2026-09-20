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
	ChassisMode_Follow = 0,	// 底盘跟随云台模式
	ChassisMode_SpinLeft,	// 小陀螺，逆时针
	ChassisMode_SpinRight,	// 小陀螺，顺时针
} Chassis_Mode_e;

/* 人控/AI 控来源。本车只有人控，pattern 恒为 Chassis_control，
   保留结构是为了和参考工程的状态机分支对齐。 */
typedef enum
{
	Chassis_control,
} Chassis_Pattern_e;


typedef struct _Chassis
{
	// 底盘尺寸信息
	struct Info
	{
		float wheelbase;	// 轴距
		float wheeltrack;	// 轮距
		float wheelRadius;	// 轮半径
		float offsetX;		// 重心在xy轴上的偏移
		float offsetY;
		float R;			// 轮子到中心的距离
		float rpm_ratio;	// 轮子线速度(m/s) ↔ 输出轴角速度(rad/s) 的换算系数 = 1/轮半径
	} info;
	// 4个电机
	Motor_t motor_3508[4];
	// 底盘移动信息
	struct Move
	{
		float vx; // 当前左右平移速度 mm/s
		float vy; // 当前前后移动速度 mm/s
		float vw; // 当前旋转速度 rad/s

		float maxVx, maxVy, maxVw; // 三个分量最大速度

		float real_vx;//根据当前轮速解算实际车速
		float real_vy;
		float real_vw;
		PID real_xPID, real_yPID, real_wPID; // 速度pid

		float maxPower;
		Slope xSlope, ySlope, outputSlope, chargeSlope, spinSlope; // 斜坡
		uint8_t fastMode; // 快速模式
	} move;
	
	struct
	{
		int8_t key_w;
		int8_t key_a;
		int8_t key_s;
		int8_t key_d;
	} key;

	// 旋转相关信息
	struct
	{
		PID pid;				// 旋转PID，由relativeAngle计算底盘旋转速度
		float relativeAngle;	// 云台与底盘的偏离角，单位 rad，取自 gimbal.yaw.machine_yaw_angle
		Chassis_Mode_e mode;		// 底盘模式 小陀螺或者底盘跟随
		float ratio;				// 旋转速度系数 占最大速度的多少
	} rotate;
	struct
	{
		float pitchTiltAngle; //底盘和地面的倾斜角
		float rollTiltAngle;
	}angle;
	Chassis_Pattern_e pattern;

	/* 功率计/超级电容反馈（CAN 0x213），写入见 USER_CAN.c */
	struct
	{
		float voltage;	// 母线电压 V
		float current;	// 母线电流 A
		float power;	// 一阶滤波后的功率 W
	} power_fb;

	// 功率预测模型参数与状态，见 Power_Control.c
	ChassisPowerControl_t power_prediction;
} Chassis_t;


extern volatile Chassis_t chassis;

void Chassis_Init(void);
uint8_t Chassis_IsInitialized(void);
void Chassis_ResetControl(void);
void Chassis_UpdateMove(void);
void Task_Chassis_Callback(void);

/* FreeRTOS 底盘任务入口：2ms 一轮，解算轮速写入 motor_3508[].target_speed */
void OS_ChassisCallback(void const *argument);

#endif
