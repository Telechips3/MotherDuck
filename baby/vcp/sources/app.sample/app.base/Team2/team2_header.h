#ifndef _TEAM2_COMMON_H_  // 중복 포함 방지 시작
#define _TEAM2_COMMON_H_

#include <sal_api.h> // BSP 기본 API 포함 (필요 시)
#include <gpio.h>
#include <pdm.h>
#include <gpsb.h>
#include <stdint.h>
#include "gic.h"

#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>

#include <FreeRTOS.h>
#include <task.h>

//speed.h
#define MOTOR_IN1           GPIO_GPA(4)      // L298N IN1
#define MOTOR_IN2           GPIO_GPA(8)      // L298N IN2
#define MOTOR_ENA_CH        0                // L298N ENA (PDM CH0 - GPIO A10)

// GPIO
/* ===== Encoder GPIO ===== */
#define ENC_A_GPIO          GPIO_GPA(22)
#define ENC_B_GPIO          GPIO_GPA(21)

/* ===== Motor GPIO ===== */
#define MOTOR_IN1_GPIO      GPIO_GPA(5)
#define MOTOR_IN2_GPIO      GPIO_GPA(9)

#define BUZZER_GPIO         GPIO_GPA(19)

//spi.h
#define SPI_CS_GPIO     GPIO_GPB(5)
#define SPI_SCLK_GPIO   GPIO_GPB(4)
#define SPI_MOSI_GPIO   GPIO_GPB(6)
#define SPI_MISO_GPIO   GPIO_GPB(7)

//Interrupt Example
#define EIT (GIC_EXT4)
#define MY_GPIO (GPIO_GPB(2))

#endif // _TEAM2_COMMON_H_ 끝
