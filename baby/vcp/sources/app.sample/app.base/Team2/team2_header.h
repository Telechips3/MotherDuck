#ifndef _TEAM2_COMMON_H_  // 중복 포함 방지 시작
#define _TEAM2_COMMON_H_

#include <sal_api.h> // BSP 기본 API 포함 (필요 시)
#include <gpio.h>
#include <pdm.h>
#include <gpsb.h>
#include <stdint.h>
#include "gic.h"

//speed.h
#define MOTOR_IN1           GPIO_GPA(4)      // L298N IN1
#define MOTOR_IN2           GPIO_GPA(8)      // L298N IN2
#define MOTOR_ENA_CH        0                // L298N ENA (PDM CH0 - GPIO A10)



//Interrupt Example
#define EIT (GIC_EXT4)
#define MY_GPIO (GPIO_GPB(2))

#endif // _TEAM2_COMMON_H_ 끝
