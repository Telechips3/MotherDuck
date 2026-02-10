#ifndef _STEER_H_
#define _STEER_H_

#include "../team2_header.h"

#define STEER_PERIOD_NS       20000000  

// 서보 제어 범위 (표준 1.0ms ~ 2.0ms)
// 만약 각도가 더 필요하면 500000 ~ 2500000까지 늘릴 수 있습니다.
#define STEER_MIN_NS          500000       
#define STEER_NEUTRAL_NS      1700000      // 1.5ms가 국룰입니다!
#define STEER_MAX_NS          2500000      

#define SERVO_RANGE_NS    (STEER_MAX_NS - STEER_MIN_NS) // 2,000,000ns
#define MAX_STEER_RAD           1.0f       



// 조향 감도 (0.01ms씩 이동)
// 너무 느리면 이 값을 20000~50000으로 키우세요.
#define STEER_STEP_NS         100

#define STEER_PWM_CH          PDM_CHANNEL_4         // PDM CH1

void control_steering_step(uint32 cmd);
void control_steering_absolute(int16_t x_norm_q15);
void Control_Steering_Custom(float steering_rad);

#endif /* _STEER_H_ */