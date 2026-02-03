/* speed.h */
#ifndef _SPEED_H_
#define _SPEED_H_

#include "../team2_header.h"

// --- PWM 주주기 ---
#define PERIOD_NS           1000000          // 1kHz (모터용)
#define SERVO_PERIOD_NS     20000000         // 20ms (서보용)

// --- 모터 속도 단계 (0 ~ PERIOD_NS) ---
#define SPEED_STEP          50000            // 가속/감속 단계 (5%)
#define SPEED_MAX           800000           // 최대 속도 (80%)
#define SPEED_MIN           0                // 정지

// --- 서보 조향 단계 (1.0ms ~ 2.0ms) ---
#define STEER_STEP          100000           // 조향 변화 단계 (0.1ms)
#define SERVO_CENTER_NS     1500000          // 중립
#define SERVO_LEFT_LIMIT    1000000          // 왼쪽 끝
#define SERVO_RIGHT_LIMIT   2000000          // 오른쪽 끝

void control_motor_drive(uint32 cmd);

#endif