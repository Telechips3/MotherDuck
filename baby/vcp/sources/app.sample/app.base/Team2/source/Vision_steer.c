// Vision_steer.c
#include <math.h>
#include <stdint.h>

#include "follow_steer_module.h"
#include "Vision_steer.h"

#define STEER_GAIN  2.0f

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// aruco_x_norm_q15: [-32768, +32767] 가정 (중앙 0)
// aruco_dist_mm: 사용 안 해도 됨 (인터페이스 유지용)
// delta_max_rad: 서보가 버티는 최대 조향각(라디안) 예: 20deg = 0.349066f
int steer_from_aruco_q15(int16 aruco_x_norm_q15,
                           uint16 aruco_dist_mm,
                          float *steer_angle_rad)
{
    (void)aruco_dist_mm; // 현재 요구사항에서는 미사용

    // Q15 -> [-1, 1] 근사
    float x = (float)aruco_x_norm_q15 / 32768.0f;

    // 혹시 입력이 범위 밖으로 튀면 방어
    x = clampf(x, -1.0f, +1.0f);

    // 중앙->끝 : 최대조향각으로 선형 매핑 + 포화
    float delta = (x * STEER_GAIN) * STREER_LIMIT;
    delta = clampf(delta, -STREER_LIMIT, +STREER_LIMIT);
    *steer_angle_rad = delta;

    return 0;
}
