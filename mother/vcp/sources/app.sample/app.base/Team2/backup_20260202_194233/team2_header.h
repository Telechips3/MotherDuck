#ifndef _TEAM2_COMMON_H_  // 중복 포함 방지 시작
#define _TEAM2_COMMON_H_

#include <sal_api.h> // BSP 기본 API 포함 (필요 시)
#include <gpio.h>
#include <pdm.h>
#include <gpsb.h>
#include <stdint.h>
#include "gic.h"

// team2_header.h 수정

// BTS7960은 IN1/IN2 대신 두 개의 PWM 채널을 사용합니다.
#define MOTOR_RPWM_CH       0                // PDM CH0 (GPIO A10) - 정회전 PWM
#define MOTOR_LPWM_CH       1                // PDM CH1 (GPIO A11) - 역회전 PWM

// Enable 핀 (필요 시 GPIO로 제어, 상시 사용 시 VCC 5V에 연결)
#define MOTOR_R_EN          GPIO_GPA(4)      
#define MOTOR_L_EN          GPIO_GPA(8)
//Interrupt Example
#define EIT (GIC_EXT4)
#define MY_GPIO (GPIO_GPB(2))

#endif // _TEAM2_COMMON_H_ 끝