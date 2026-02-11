#ifndef _SPEED_H_  // 중복 포함 방지 시작
#define _SPEED_H_

#include "../team2_header.h"

#define PERIOD_NS           500000          // 5ms 주기
#define DUTY_SLOW_NS        50000           // 30% 속도

#define KICK_DUTY_MAX       (PERIOD_NS)
#define MOVING_THRESHOLD    1.5f
#define KICK_TIMEOUT_MS     500

#define DUTY_STOP_NS        0                // 정지
#define MOTOR_PWM_CH        0                // PDM 채널 0 (GPIO A10)

#define BASE_SPEED_PWM      0.2f           // 기본 PWM

#define DUTY_FARFAR         0.4f
#define DUTY_FAR            0.6f
#define DUTY_MID            0.5f
#define DUTY_NEAR           0.35f
#define DUTY_SLOW           0.2f

#define ACC_DIST_FARFAR     250.0f
#define ACC_DIST_FAR        175.0f
#define ACC_DIST_MID        125.0f
#define ACC_DIST_NEAR       75.0f

#define ACC_DIST_EMER       35.0f            // 후진

#define SAFE_DISTANCE_CM    50.0f           // 차간거리
#define ACC_DEAD_ZONE       3.0f            // 히스테리스 데드존


void process_acc_system(float current_dist_cm);
void process_acc_with_encoder(float current_dist_cm);  // ← 엔코더 기반 폐루프
void control_motor_drive(uint32 cmd);
void control_motor_manual(float ratio, uint8_t direction);
void getCurrentSpeed(float* current_speed);

#endif