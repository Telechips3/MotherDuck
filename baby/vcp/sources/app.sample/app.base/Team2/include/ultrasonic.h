#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include "../team2_header.h"

// --- 설정값 ---
#define ULTRA_DISTANCE_MAX    400
#define ULTRA_DISTANCE_MIN    2
#define BUZ_NEAR_LIMIT       10
#define BUZ_MID_LIMIT        20
#define BUZ_FAR_LIMIT        30
#define ULTRA_TIMEOUT_MS     10
// --- API 함수 ---
void Ultrasonic_Init(void);
void Buzzer_Init(void);
uint32_t Ultrasonic_GetDistance_cm(void);
void Buzzer_Set(uint8_t on);
// 태스크 생성 함수
SALRetCode_t UltrasonicTaskCreate(void);

#endif