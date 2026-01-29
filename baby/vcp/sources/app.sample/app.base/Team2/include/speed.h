#ifndef _SPEED_H_  // 중복 포함 방지 시작
#define _SPEED_H_

#include "../team2_header.h"

// 핀 및 속도 설정
#define MOTOR_IN1           GPIO_GPB(2)      // L298N IN1
#define MOTOR_IN2           GPIO_GPB(3)      // L298N IN2
#define MOTOR_ENA_CH        0                // L298N ENA (PDM CH0 - GPIO A10)

#define PERIOD_NS           1000000          // 1kHz
#define DUTY_SLOW_NS        500000           // 30% 속도
#define DUTY_STOP_NS        0                // 정지
#define MOTOR_PWM_CH        0                // PDM 채널 0 (GPIO A10)
#define PERIOD_NS           1000000          // 주기: 1ms (1kHz)


void speed_pwm(void);

#endif