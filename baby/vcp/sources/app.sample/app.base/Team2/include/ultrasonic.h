// SPDX-License-Identifier: Apache-2.0
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <sal_api.h>
#include <gpio.h>
#include <FreeRTOS.h>
#include <task.h>

/* GPIO */
#define ULTRA_TRIG_GPIO    GPIO_GPA(23)
#define ULTRA_ECHO_GPIO    GPIO_GPA(24)
#define ULTRA_ECHO_TIMEOUT 0xFFFFFFFF
#define ULTRA_CNT_TO_US_K  1.0f

void UltrasonicTask(void *pArg);

#endif
