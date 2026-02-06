#ifndef _SPEED_H_  // 중복 포함 방지 시작
#define _SPEED_H_

#include "../team2_header.h"

#define PERIOD_NS           500000          // 5ms 주기
#define DUTY_SLOW_NS        50000           // 30% 속도

#define DUTY_STOP_NS        0                // 정지
#define MOTOR_PWM_CH        0                // PDM 채널 0 (GPIO A10)

void process_acc_system(float current_dist_cm);
void process_acc_with_encoder(float current_dist_cm);  // ← 엔코더 기반 폐루프
void control_motor_drive(uint32 cmd);

#endif