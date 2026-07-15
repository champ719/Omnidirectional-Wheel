#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "USER_PID.h"

#define MOTOR_CONTROL_PERIOD_S       0.002f
#define MOTOR_WHEEL_RADIUS_M         0.075f
#define MOTOR_HALF_LENGTH_M          0.300f
#define MOTOR_HALF_WIDTH_M           0.300f
#define MOTOR_MAX_VX_MPS             6.0f
#define MOTOR_MAX_VY_MPS             6.0f
#define MOTOR_MAX_WZ_RADPS           4.0f
#define MOTOR_SMALL_GYRO_WZ_RADPS    MOTOR_MAX_WZ_RADPS
#define MOTOR_REMOTE_FULL_SCALE      660.0f
#define MOTOR_REMOTE_DEADBAND        0

#define MOTOR_YAW_TARGET_COUNT       1400.0f
#define MOTOR_YAW_DEADBAND_COUNT     30.0f
#define MOTOR_YAW_D_FILTER_ALPHA     0.8f
#define MOTOR_INERTIAL_YAW_DEADBAND_RAD  0.0035f
#define MOTOR_INERTIAL_YAW_D_FILTER_ALPHA  0.2f
#define MOTOR_GIMBAL_COMMAND_MAX             163.84f
#define MOTOR_YAW_FF_MEASURE_DELAY_CYCLES    1000U
#define MOTOR_YAW_FF_LPF_ALPHA               0.002f
#define MOTOR_YAW_FF_AUTO_LEARN_ALPHA        0.0004f
#define MOTOR_YAW_FF_MIN_WZ_RADPS            1.0f
#define MOTOR_YAW_FF_LEARN_MAX_ERROR_RAD     0.20f
#define MOTOR_YAW_FF_GAIN_MAX                20.0f
#define MOTOR_PITCH_START_COUNT      5500.0f
#define MOTOR_PITCH_MIN_COUNT        5000.0f
#define MOTOR_PITCH_MAX_COUNT        6000.0f
#define MOTOR_PITCH_RATE_COUNT_S     1000.0f
#define MOTOR_ENCODER_RANGE          8192.0f
#define MOTOR_ENCODER_COUNT_PER_RAD  (MOTOR_ENCODER_RANGE / 6.28318530718f)

/*
 * 1: BMI088 is fixed to the gimbal; its yaw directly represents ground yaw.
 * 0: BMI088 is fixed to the chassis; chassis yaw is converted to an encoder
 *    target that counter-rotates the gimbal.
 */
#ifndef MOTOR_IMU_MOUNTED_ON_GIMBAL
#define MOTOR_IMU_MOUNTED_ON_GIMBAL  1U
#endif

/* Direction mapping from positive IMU yaw to positive yaw-encoder counts. */
#define MOTOR_IMU_YAW_TO_ENCODER_DIRECTION  (+1.0f)

/* Small-gyro exit: rotate the chassis until yaw returns to its normal offset. */
#define MOTOR_GYRO_EXIT_ALIGN_KP_RADPS_PER_COUNT  0.0030f
#define MOTOR_GYRO_EXIT_ALIGN_MAX_WZ_RADPS        2.0f
#define MOTOR_GYRO_EXIT_ALIGN_MIN_WZ_RADPS        0.4f
#define MOTOR_GYRO_EXIT_ALIGN_TOLERANCE_COUNT     30.0f
#define MOTOR_GYRO_EXIT_WHEEL_STOP_RADPS          1.0f
#define MOTOR_GYRO_EXIT_STABLE_CYCLES             50U
/* Flip to -1.0f if chassis alignment drives the encoder error larger. */
#define MOTOR_GYRO_EXIT_CHASSIS_DIRECTION          (+1.0f)

/* Set to +1.0f or -1.0f after checking each gimbal motor direction. */
#define MOTOR_YAW_DIRECTION          1.0f
#define MOTOR_PITCH_DIRECTION        1.0f

/*
 * Wheel motor installation directions. A zero direction keeps all wheel
 * motors stopped through the calibration guard.
 */
#define MOTOR_DIRECTION_FL          (+1.0f)
#define MOTOR_DIRECTION_FR          (-1.0f)
#define MOTOR_DIRECTION_RL          (+1.0f)
#define MOTOR_DIRECTION_RR          (-1.0f)

typedef enum
{
    MOTOR_WHEEL_FL = 0,
    MOTOR_WHEEL_FR,
    MOTOR_WHEEL_RL,
    MOTOR_WHEEL_RR,
    MOTOR_WHEEL_COUNT
} Motor_WheelIndex_t;

typedef enum
{
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_REMOTE_OFFLINE,
    MOTOR_STATE_EMERGENCY_STOP,
    MOTOR_STATE_DIRECTION_UNCALIBRATED,
    MOTOR_STATE_SMALL_GYRO,
    MOTOR_STATE_IMU_NOT_READY,
    MOTOR_STATE_GYRO_EXIT_ALIGN
} Motor_State_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float output_max;
    float integral_max;
} Motor_PID_Parameters_t;

typedef struct
{
    volatile float yaw_target;
    volatile float yaw_feedback;
    volatile float yaw_error;
    volatile float yaw_output;
    volatile float pitch_target;
    volatile float pitch_feedback;
    volatile float pitch_error;
    volatile float pitch_output;
    volatile float inertial_yaw_target;
    volatile float inertial_yaw_feedback;
    volatile float inertial_yaw_error;
    volatile float inertial_encoder_reference;
    volatile float exit_align_error_count;
    volatile float exit_align_remaining_count;
    volatile float exit_align_last_encoder_count;
    volatile float exit_align_direction_accumulator;
    volatile float exit_align_encoder_direction;
    volatile uint16_t exit_align_stable_count;
    volatile uint16_t exit_align_direction_detect_cycles;
    volatile uint8_t exit_align_direction_detected;
    volatile uint8_t exit_align_creep_mode;
    volatile uint8_t exit_align_initialized;
    volatile uint8_t inertial_hold_active;
    volatile uint8_t initialized;
    USER_PID_t yaw_pid;
    USER_PID_t inertial_yaw_pid;
    USER_PID_t pitch_pid;
} Motor_GimbalControl_t;

typedef struct
{
    volatile Motor_State_t state;
    volatile uint32_t update_count;
    volatile float vx_target;
    volatile float vy_target;
    volatile float wz_target;
    volatile float wheel_target[MOTOR_WHEEL_COUNT];
    volatile float wheel_feedback[MOTOR_WHEEL_COUNT];
    volatile float wheel_output[MOTOR_WHEEL_COUNT];
    USER_PID_t speed_pid[MOTOR_WHEEL_COUNT];
    Motor_GimbalControl_t gimbal;
} Motor_Control_t;

extern Motor_Control_t motor_control;
extern volatile Motor_PID_Parameters_t motor_speed_pid_parameters;
extern volatile Motor_PID_Parameters_t motor_yaw_pid_parameters;
extern volatile Motor_PID_Parameters_t motor_inertial_yaw_pid_parameters;
extern volatile Motor_PID_Parameters_t motor_pitch_pid_parameters;
extern volatile float motor_inertial_yaw_feedforward_gain;

extern volatile uint8_t motor_update_flag;

void Motor_Control_Init(void);
void Motor_Control_Update_2ms(void);
void Motor_Control_StopAll(Motor_State_t reason);
uint8_t Motor_Control_DirectionsCalibrated(void);

#endif
