#ifndef _SPEED_H_  // 중복 포함 방지 시작
#define _SPEED_H_

#include "../team2_header.h"

#define PERIOD_NS           500000          // 5ms 주기
#define DUTY_SLOW_NS        50000           // 30% 속도

#define DUTY_STOP_NS        0                // 정지
#define MOTOR_PWM_CH        0                // PDM 채널 0 (GPIO A10)

#define BASE_SPEED_PWM      0.005f           // 기본 PWM

#define DUTY_FARFAR         5.0f
#define DUTY_FAR            4.0f
#define DUTY_MID            3.0f
#define DUTY_NEAR           2.0f
#define DUTY_EMER           1.0f

#define ACC_DIST_FARFAR     125.0f
#define ACC_DIST_FAR        100.0f
#define ACC_DIST_MID        75.0f
#define ACC_DIST_NEAR       50.0f

#define SAFE_DISTANCE_CM    5.0f
#define ACC_DEAD_ZONE       2.0f


void process_acc_system(float current_dist_cm);
void process_acc_with_encoder(float current_dist_cm);  // ← 엔코더 기반 폐루프
void control_motor_drive(uint32 cmd);
void control_motor_manual(float ratio, uint8_t direction);
void getCurrentSpeed(float* current_speed);

#endif