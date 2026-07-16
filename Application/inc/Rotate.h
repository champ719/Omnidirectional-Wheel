#ifndef ROTATE_H
#define ROTATE_H

#include <stdint.h>

void Rotate_Init(void);
void Rotate_UpdateActive(float remote_forward,
                         float remote_right,
                         float remote_yaw,
                         float remote_pitch,
                         float yaw_feedback,
                         float yaw_speed_feedback,
                         float pitch_feedback,
                         float pitch_speed_feedback,
                         float imu_yaw_continuous,
                         float dt);
uint8_t Rotate_UpdateExit(float remote_pitch,
                          float yaw_feedback,
                          float yaw_speed_feedback,
                          float pitch_feedback,
                          float pitch_speed_feedback,
                          float imu_yaw_continuous,
                          const float wheel_feedback[4],
                          float dt);

#endif
