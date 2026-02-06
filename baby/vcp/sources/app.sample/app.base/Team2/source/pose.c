// pose.c
#include "pose.h"
#include <math.h>

// 🔽 다른 모듈 결과만 사용
#include "encoder.h"   // Encoder_GetDeltaDistanceM()
#include "imu.h"       // IMU_GetYawRad()

static Pose s_pose;
static int s_yaw_override_enabled = 0;
static float s_yaw_override_rad = 0.0f;

/**
 * @brief Pose 초기화
 */
void Pose_Init(float x0, float y0, float yaw0)
{
    s_pose.x   = x0;
    s_pose.y   = y0;
    s_pose.yaw = yaw0;
}

/**
 * @brief Pose 업데이트 (odometry)
 */
void Pose_Update(void)
{
    // 1️⃣ 현재 방향 (IMU or debug override)
    if (s_yaw_override_enabled) {
        s_pose.yaw = s_yaw_override_rad;
    } else {
        IMU_Data_t imu;
        if (IMU_GetData(&imu) == 0) {
            s_pose.yaw = imu.yaw * (3.1415926535f / 180.0f);
        }
    }

    // 2️⃣ 이번 루프 이동 거리 (Encoder, cm -> m)
    float d_m = Encoder_GetDeltaDistanceCm() * 0.01f;

    // 3️⃣ 좌표 적분 (핵심)
    s_pose.x += d_m * cosf(s_pose.yaw);
    s_pose.y += d_m * sinf(s_pose.yaw);

}

/**
 * @brief 현재 pose 반환
 */
int Pose_Get(Pose *out)
{
    if (!out) return 1;
    *out = s_pose;
    return 0;
}

void Pose_DebugSetYawOverride(float yaw_rad, int enabled)
{
    s_yaw_override_rad = yaw_rad;
    s_yaw_override_enabled = (enabled != 0);
}

void Pose_DebugClearYawOverride(void)
{
    s_yaw_override_enabled = 0;
}
