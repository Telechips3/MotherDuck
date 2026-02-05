// SPDX-License-Identifier: Apache-2.0
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <sal_api.h>
#include <gpio.h>
#include <FreeRTOS.h>
#include <task.h>


#define ULTRA_CNT_TO_US_K  1.0f

void UltrasonicTask(void *pArg);
SALRetCode_t UltrasonicTaskCreate(void);

#endif
