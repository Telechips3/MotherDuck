// SPDX-License-Identifier: Apache-2.0
#ifndef BUZZER_H
#define BUZZER_H

#include <sal_api.h>
#include <gpio.h>
#include <FreeRTOS.h>
#include <task.h>

// GPIO
#define BUZZER_GPIO GPIO_GPA(19)

void BuzzerTask(void *pArg);
SALRetCode_t BuzzerTaskCreate(void);

#endif
