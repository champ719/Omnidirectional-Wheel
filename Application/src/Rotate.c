#include "Rotate.h"
#include "Chassis.h"
#include "Gimbal.h"

void Rotate_Init(void)
{
    Chassis_Ctrl_ResetRotateExit();
}

void Rotate_UpdateActive(float remote_forward,
                         float remote_right,
                         float remote_yaw,
                         float remote_pitch,
                         float yaw_feedback,
                         float yaw_speed_feedback,
                         float pitch_feedback,
                         float pitch_speed_feedback,
                         float imu_yaw_continuous,
                         float dt)
{
    Chassis_Ctrl_PrepareRotate();
    Chassis_Ctrl_SetRotateTargets(remote_forward,
                                  remote_right,
                                  yaw_feedback);
    Chassis_Ctrl_CalculateWheelTargets(1U);
    Gimbal_Ctrl_UpdateRotate(remote_yaw,
                             remote_pitch,
                             1U,
                             yaw_feedback,
                             yaw_speed_feedback,
                             pitch_feedback,
                             pitch_speed_feedback,
                             imu_yaw_continuous,
                             chassis_control.wz_target,
                             dt);
}

uint8_t Rotate_UpdateExit(float remote_pitch,
                          float yaw_feedback,
                          float yaw_speed_feedback,
                          float pitch_feedback,
                          float pitch_speed_feedback,
                          float imu_yaw_continuous,
                          const float wheel_feedback[4],
                          float dt)
{
    if (Chassis_Ctrl_UpdateRotateExit(yaw_feedback,
                                      wheel_feedback) != 0U) {
        return 1U;
    }

    Chassis_Ctrl_CalculateWheelTargets(0U);
    Gimbal_Ctrl_UpdateRotate(0.0f,
                             remote_pitch,
                             0U,
                             yaw_feedback,
                             yaw_speed_feedback,
                             pitch_feedback,
                             pitch_speed_feedback,
                             imu_yaw_continuous,
                             chassis_control.wz_target,
                             dt);
    return 0U;
}
