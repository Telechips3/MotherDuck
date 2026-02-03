#ifndef _TEAM2_COMMON_H_  // 중복 포함 방지 시작
#define _TEAM2_COMMON_H_

#include <sal_api.h> // BSP 기본 API 포함 (필요 시)
#include <gpio.h>
#include <pdm.h>
#include <gpsb.h>
#include <stdint.h>
#include "gic.h"

//speed.h - L298N 1개로 2개 모터 제어 + 서보모터 조향
// 왼쪽 모터 (OUT1-OUT2, ENA)
#define MOTOR_LEFT_IN1      GPIO_GPA(4)      // L298N IN1
#define MOTOR_LEFT_IN2      GPIO_GPA(8)      // L298N IN2
#define MOTOR_LEFT_ENA_CH   0                // L298N ENA (PDM CH0 - GPA10)

// 오른쪽 모터 (OUT3-OUT4, ENB)
#define MOTOR_RIGHT_IN3     GPIO_GPA(5)      // L298N IN3
#define MOTOR_RIGHT_IN4     GPIO_GPA(9)      // L298N IN4
#define MOTOR_RIGHT_ENB_CH  1                // L298N ENB (PDM CH1 - GPA11)

// 서보모터 (조향)
#define SERVO_STEER_CH      7                // 서보모터 (PDM CH2 - GPA12)



//Interrupt Example
#define EIT (GIC_EXT4)
#define MY_GPIO (GPIO_GPB(2))

#endif // _TEAM2_COMMON_H_ 끝