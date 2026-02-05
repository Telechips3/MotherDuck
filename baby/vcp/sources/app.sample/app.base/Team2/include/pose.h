// pose.h
#ifndef POSE_H
#define POSE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;    // meter
    float y;    // meter
    float yaw;  // rad
} Pose;

/**
 * @brief Pose 초기화 (출발점)
 */
void Pose_Init(float x0, float y0, float yaw0);

/**
 * @brief 현재 pose 갱신 (IMU + Encoder 사용)
 *        주기적으로 호출해야 함
 */
void Pose_Update(void);

/**
 * @brief 현재 pose 값 얻기
 */
int Pose_Get(Pose *out);

/**
 * @brief 디버그용 yaw override 설정 (라디안)
 * @note  enabled=1이면 IMU 대신 override 값 사용
 */
void Pose_DebugSetYawOverride(float yaw_rad, int enabled);

/**
 * @brief 디버그용 yaw override 비활성화
 */
void Pose_DebugClearYawOverride(void);

#ifdef __cplusplus
}
#endif

#endif // POSE_H
