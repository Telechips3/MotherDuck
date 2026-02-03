#ifndef _STEER_H_
#define _STEER_H_

#include "../team2_header.h"

#define STEER_PERIOD_NS       20000000  // 50Hz

// 서보 제어 범위 (단위: ns)
#define STEER_MIN_NS          500000    // 최대 왼쪽 (-45도)
#define STEER_NEUTRAL_NS      1500000   // 정중앙 (0도)
#define STEER_MAX_NS          2500000   // 최대 오른쪽 (+45도)

// [추가] 한 번의 입력으로 변화시킬 펄스 폭 (조향 감도)
// 이 값을 키우면 조향이 민감해지고, 줄이면 아주 부드러워집니다.
#define STEER_STEP_NS         30000     

#define STEER_PWM_CH          PDM_CHANNEL_4         // PDM CH1

void control_steering_step(uint32 cmd);

#endif