#include "chassis.h"
#include "Error.h"
#include "motor.h"
#include "Remote.h"
#include "cmsis_os.h"
#include "gimbal.h"
#include <math.h>

#define CHASSIS_DEG_TO_RAD    0.01745329252f
#define CHASSIS_RAD_TO_DEG    57.2957795f
#define CHASSIS_COS45         0.707106f
#define CHASSIS_TASK_PERIOD_S 0.002f

#define CHASSIS_KEYBOARD_NORMAL_SCALE 0.60f
#define CHASSIS_KEYBOARD_FAST_SCALE   1.00f //shift加速
#define CHASSIS_KEYBOARD_SLOW_SCALE   0.30f //ctrl减速

volatile Chassis_t chassis;

/* 上一拍的按键位图，用来抓 Q/E 的下降沿。持续按住只切一次模式。 */
static uint16_t chassis_last_keys;

void Chassis_Init(void)
{
    //底盘尺寸信息
    chassis.info.wheelRadius = 0.075f;
    chassis.info.R           = 0.2687f;
    chassis.info.rpm_ratio   = (chassis.info.wheelRadius > 0.0f)
                             ? (1.0f / chassis.info.wheelRadius)
                             : 0.0f;
    chassis.info.offsetX     = 0.0f;
    chassis.info.offsetY     = 0.0f;

    chassis.info.wheelbase   = 0.380f;
    chassis.info.wheeltrack  = 0.380f;

    chassis.move.maxVx       = 3.0f;
    chassis.move.maxVy       = chassis.move.maxVx;
    chassis.move.maxVw       = (chassis.info.R > 0.0f)
                             ? (chassis.move.maxVx * CHASSIS_COS45 / chassis.info.R)
                             : 0.0f;

    chassis_last_keys        = 0U;

    Motor_Init(&chassis.motor_3508[LF], 0x200, DJI_3508,
    0.4f, 0.2f, 0.7f, 0.2f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    Motor_Init(&chassis.motor_3508[RF], 0x200, DJI_3508,
    0.8f, 0.1f, 0.6f, 0.0f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    Motor_Init(&chassis.motor_3508[LB], 0x200, DJI_3508,
    0.8f, 0.1f, 0.6f, 0.0f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);
    Motor_Init(&chassis.motor_3508[RB], 0x200, DJI_3508,
    0.8f, 0.1f, 0.6f, 0.0f, 20.0f, 6.0f,
    0.1f, 0.0f, 0.0f, 0.0f, 25.0f, 8.0f);

    PID_Init(&chassis.rotate.pid, 0.15f, 0.0f, 0.5f, 3.0f, 5.0f);
    //PID_Init(&chassis.move.real_xPID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    //PID_Init(&chassis.move.real_yPID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    PID_Init(&chassis.move.real_wPID, 5.0f, 0.0f, 0.05f, 0.0f, 0.0f);

    Slope_Init(&chassis.move.xSlope,
               CHASSIS_TRANSLATION_ACCEL_MPS2 * CHASSIS_TASK_PERIOD_S, 0.0f);
    Slope_Init(&chassis.move.ySlope,
               CHASSIS_TRANSLATION_ACCEL_MPS2 * CHASSIS_TASK_PERIOD_S, 0.0f);
    Slope_Init(&chassis.move.spinSlope,
               CHASSIS_ROTATION_ACCEL_RADPS2 * CHASSIS_TASK_PERIOD_S, 0.0f);

    PowerControl_Init();
    Chassis_ResetControl();
}

void Chassis_ResetControl(void)
{
    Slope_Reset(&chassis.move.xSlope, 0.0f);
    Slope_Reset(&chassis.move.ySlope, 0.0f);
    Slope_Reset(&chassis.move.spinSlope, 0.0f);

    chassis.move.vx = 0.0f;
    chassis.move.vy = 0.0f;
    chassis.move.vw = 0.0f;
    PID_Clear(&chassis.rotate.pid);

    for (uint8_t index = 0U; index < 4U; index++) {
      chassis.motor_3508[index].target_speed = 0.0f;
      chassis.motor_3508[index].give_current = 0.0f;
      PID_Clear(&chassis.motor_3508[index].pid_speed);
    }
    PowerControl_Reset();
}

/* 单 yaw：云台的 yaw 电机就装在底盘上，它的机械角本身就是云台相对底盘的偏角。
   gimbal.yaw.machine_yaw_angle 已经是相对零位的弧度值，换算成度给旋转矩阵和
   跟随环用。yaw 掉线时按 0 处理，跟随环停止修正而不是乱转。 */
static void Chassis_UpdateAngle(void)
{
    if (Motor_IsOnline(&gimbal.yaw_motor) != 0U) {
        chassis.rotate.relativeAngle =
            gimbal.yaw.machine_yaw_angle * CHASSIS_RAD_TO_DEG;
    } else {
        chassis.rotate.relativeAngle = 0.0f;
    }
}

/* 把遥控输入统一成 ±1 的 forward(前为正)/right(右为正)。摇杆和键鼠两条路都
   收敛到这里，Chassis_UpdateMove 就不用再分支。 */
static void Chassis_ReadCommand(float *forward, float *right)
{
    RC_Ctrl_t remote;
    float f;
    float r;
    float magnitude;

    Remote_GetSnapshot(&remote);

    if (Rocker_Ctrl != 0U) {
        f = Remote_NormalizeChannel(remote.rc.ch2) * (-1.0f);
        r = Remote_NormalizeChannel(remote.rc.ch3);
    } else {
        uint16_t keys = remote.key.v;
        float scale;

        f = (float)((keys & RC_KEY_W) != 0U) - (float)((keys & RC_KEY_S) != 0U);
        r = (float)((keys & RC_KEY_D) != 0U) - (float)((keys & RC_KEY_A) != 0U);

        if ((keys & RC_KEY_CTRL) != 0U) {
            scale = CHASSIS_KEYBOARD_SLOW_SCALE;
        } else if ((keys & RC_KEY_SHIFT) != 0U) {
            scale = CHASSIS_KEYBOARD_FAST_SCALE;
        } else {
            scale = CHASSIS_KEYBOARD_NORMAL_SCALE;
        }
        f *= scale;
        r *= scale;
    }

    /* 斜向合成时限制幅值，避免 45° 方向速度超出上限 */
    magnitude = sqrtf((f * f) + (r * r));
    if (magnitude > 1.0f) {
        f /= magnitude;
        r /= magnitude;
    }

    *forward = f;
    *right = r;
}

/* 小陀螺的转向：vw 为正时四轮同向正转，俯视车体为逆时针，所以左转取正、
   右转取负。 */
static float Chassis_SpinDirection(void)
{
    return (chassis.rotate.mode == ChassisMode_SpinLeft) ? 1.0f : -1.0f;
}

/* 键鼠模式下 Q/E 切旋转模式，抓下降沿，按住不会反复触发。
   同一个键再按一次回跟随，按另一个键直接换转向。 */
static void Chassis_UpdateModeKey(void)
{
    RC_Ctrl_t remote;
    uint16_t keys;
    uint16_t pressed;

    Remote_GetSnapshot(&remote);
    keys = remote.key.v;
    pressed = (uint16_t)(keys & (uint16_t)(~chassis_last_keys));
    chassis_last_keys = keys;

    /* 摇杆模式下由左拨杆 s2 直接选择底盘模式。 */
    if (Rocker_Ctrl != 0U) {
        Chassis_Mode_e requested_mode;

        if (remote.rc.s2 == RC_SW_MID) {
            requested_mode = ChassisMode_Follow;
        } else if (remote.rc.s2 == RC_SW_UP) {
            requested_mode = ChassisMode_SpinLeft;
        } else if (remote.rc.s2 == RC_SW_DOWN) {
            requested_mode = ChassisMode_SpinRight;
        } else {
            return;
        }

        if (requested_mode != chassis.rotate.mode) {
            PID_Clear(&chassis.rotate.pid);
            chassis.rotate.mode = requested_mode;
        }
        return;
    }

    switch (chassis.rotate.mode) {
        case ChassisMode_Follow:
            if ((pressed & RC_KEY_Q) != 0U) {
                PID_Clear(&chassis.rotate.pid);
                chassis.rotate.mode = ChassisMode_SpinLeft;
            } else if ((pressed & RC_KEY_E) != 0U) {
                PID_Clear(&chassis.rotate.pid);
                chassis.rotate.mode = ChassisMode_SpinRight;
            }
            break;
        case ChassisMode_SpinLeft:
            if ((pressed & RC_KEY_Q) != 0U) {
                chassis.rotate.mode = ChassisMode_Follow;
            } else if ((pressed & RC_KEY_E) != 0U) {
                chassis.rotate.mode = ChassisMode_SpinRight;
            }
            break;
        case ChassisMode_SpinRight:
            if ((pressed & RC_KEY_E) != 0U) {
                chassis.rotate.mode = ChassisMode_Follow;
            } else if ((pressed & RC_KEY_Q) != 0U) {
                chassis.rotate.mode = ChassisMode_SpinLeft;
            }
            break;
        default:
            break;
    }
}

/*旋转状态机*/
static void Chassis_HandleFollow(void) //底盘跟随模式
{
    Slope_SetTarget(&chassis.move.spinSlope, 0);
    float angle = chassis.rotate.relativeAngle;
    if(angle >= 180)
        angle -= 360;
    if(angle < -180)
        angle += 360;
    float deadzone = 0.1f;
    float pid_angle = 0.0f;
    if (angle > deadzone)
    {
        pid_angle = angle - deadzone;
    }
    else if (angle < -deadzone)
    {
        pid_angle = angle + deadzone;
    }
    else
    {
        pid_angle = 0.0f;
        chassis.rotate.pid.integral = 0.0f;
    }
    PID_SingleCalc(&chassis.rotate.pid, 0, pid_angle);
    chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;
    LIMIT(chassis.move.vw,-chassis.move.maxVw,chassis.move.maxVw);
}

static void Chassis_HandleSpin(void) //小陀螺模式
{
    if(chassis.pattern == Chassis_control)
    {
        if(ABS(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + ABS(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy > 0.05f)
            chassis.rotate.ratio = 0.4f;
        else
            chassis.rotate.ratio = 1.0f;
    }
    else
    {
        chassis.rotate.ratio = 0.5f;
    }
    Slope_SetTarget(&chassis.move.spinSlope,
                    chassis.move.maxVw * chassis.rotate.ratio * Chassis_SpinDirection());
	chassis.move.vw =chassis.move.spinSlope.value;
}

/* 把三个命令斜坡各推进一拍。Slope_SetTarget 只写目标，值要靠 NextVal 走。 */
static void Chassis_UpdateSlope(void)
{
    (void)Slope_NextVal(&chassis.move.xSlope);
    (void)Slope_NextVal(&chassis.move.ySlope);
    (void)Slope_NextVal(&chassis.move.spinSlope);
}

/*更新移动数据*/
void Chassis_UpdateMove(void)
{
	float gimbalAngleSin=sinf(chassis.rotate.relativeAngle*CHASSIS_DEG_TO_RAD);
	float gimbalAngleCos=cosf(chassis.rotate.relativeAngle*CHASSIS_DEG_TO_RAD);
    float maxVx = chassis.move.maxVx;
    float maxVy = chassis.move.maxVy;
    float forward;
    float right;

    Chassis_ReadCommand(&forward, &right);

    Chassis_UpdateSlope();
    if((chassis.rotate.mode == ChassisMode_SpinLeft) ||
       (chassis.rotate.mode == ChassisMode_SpinRight))
    {
        /* 只收窄本拍的可用上限。就地改 maxVx/maxVy 会每周期连乘，几百拍后衰减到 0 */
        maxVx *= chassis.rotate.ratio;
        maxVy *= chassis.rotate.ratio;
    }

    Slope_SetTarget(&chassis.move.xSlope, forward * maxVx);
    Slope_SetTarget(&chassis.move.ySlope, right * maxVy);

	chassis.move.vx=-(Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin);
	chassis.move.vy=(-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos);
}

void Task_Chassis_Callback(void)
{
    /* 故障状态下不发运动指令，只清斜坡和目标值 */
    if (Error_GetResult() != ERROR_RESULT_NONE) {
        Chassis_ResetControl();
        return;
    }

    Chassis_UpdateAngle();
    Chassis_UpdateModeKey();

    switch(chassis.rotate.mode) //更新两种旋转模式状态机
    {
        case ChassisMode_Follow:
            Chassis_HandleFollow();
            break;
        case ChassisMode_SpinLeft:
        case ChassisMode_SpinRight:
            Chassis_HandleSpin();
            break;
        default:
            break;
	}

    Chassis_UpdateMove();


    /***全向轮解算各轮子转速****/
    float cos45 = CHASSIS_COS45;

    //先反解车当前真实速度
    float real_wheel_v[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        real_wheel_v[i] = chassis.motor_3508[i].fb_speed / chassis.info.rpm_ratio;
    }
    //解算当前实际速度
    chassis.move.real_vx = (real_wheel_v[0] + real_wheel_v[1] - real_wheel_v[2] - real_wheel_v[3]) / (4.0f * cos45);
    chassis.move.real_vy = (real_wheel_v[0] - real_wheel_v[1] + real_wheel_v[2] - real_wheel_v[3]) / (4.0f * cos45);
    chassis.move.real_vw = (real_wheel_v[0] + real_wheel_v[1] + real_wheel_v[2] + real_wheel_v[3]) / (4.0f * chassis.info.R);

    PID_SingleCalc(&chassis.move.real_xPID, chassis.move.vx, chassis.move.real_vx);//PID进行修正
    PID_SingleCalc(&chassis.move.real_yPID, chassis.move.vy, chassis.move.real_vy);
    PID_SingleCalc(&chassis.move.real_wPID, chassis.move.vw, chassis.move.real_vw);

    //单级pid修正后的速度
    float ctrl_vx, ctrl_vy, ctrl_vw;
    ctrl_vx = chassis.move.vx + chassis.move.real_xPID.output;
    ctrl_vy = chassis.move.vy + chassis.move.real_yPID.output;
    ctrl_vw = chassis.move.vw + chassis.move.real_wPID.output;

    //发送给各个电机
    float wheel_v[4];
    wheel_v[0] = (ctrl_vx + ctrl_vy) * cos45 + ctrl_vw * chassis.info.R;    //左前
    wheel_v[1] = (ctrl_vx - ctrl_vy) * cos45 + ctrl_vw * chassis.info.R;    //右前
    wheel_v[2] = (-ctrl_vx + ctrl_vy) * cos45 + ctrl_vw * chassis.info.R;    //左后
    wheel_v[3] = (-ctrl_vx - ctrl_vy) * cos45 + ctrl_vw * chassis.info.R;    //右后

    //轮子线速度(m/s) → 输出轴角速度(rad/s)，与 fb_speed 同量纲
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis.motor_3508[i].target_speed = wheel_v[i] * chassis.info.rpm_ratio;
    }

    /* 目标轮速 → 电流。轮速环必须跑在功率控制之前，否则没有电流可缩放。 */
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_SingleCalc(&chassis.motor_3508[i].pid_speed,
                       chassis.motor_3508[i].target_speed,
                       chassis.motor_3508[i].fb_speed);
        chassis.motor_3508[i].give_current = chassis.motor_3508[i].pid_speed.output;
    }

    /* 按预测总功率统一缩放四轮电流，放在本拍控制量的最后一步 */
    PowerControl_Apply();
}

/**
 * @brief 底盘任务入口。
 * @param argument FreeRTOS 任务参数，当前未使用。
 * @note 先等 1.5s 让 IMU 和电调上电稳定，再初始化底盘并进 2ms 控制环。
 *       解算出的目标轮速写进 motor_3508[].target_speed，由 MotorTask 发 CAN。
 */
void OS_ChassisCallback(void const * argument)
{
	(void)argument;

	osDelay(1500);
	Chassis_Init();
    for(;;)
    {
		Task_Chassis_Callback();
        osDelay(2);
    }
}
