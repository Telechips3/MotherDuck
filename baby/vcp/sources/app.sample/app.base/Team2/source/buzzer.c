// SPDX-License-Identifier: Apache-2.0
#include "buzzer.h"

/* ultrasonic task에서 갱신 */
extern volatile uint32 g_ultraDistanceCm;

static void Buzzer_Init(void)
{
    GPIO_Config(BUZZER_GPIO,
        GPIO_OUTPUT | GPIO_NOPULL | GPIO_DS(4) | GPIO_FUNC(0));
    GPIO_Set(BUZZER_GPIO, 0);
}

void BuzzerTask(void *pArg)
{
    (void)pArg;
    Buzzer_Init();

    uint32 buz_on = 0;
    TickType_t next = 0;

    while (1)
    {
        uint32 d = g_ultraDistanceCm;
        TickType_t now = xTaskGetTickCount();

        if (d == 0 || d >= 30)
        {
            GPIO_Set(BUZZER_GPIO, 0);
            buz_on = 0;
            next = 0;
        }
        else if (d < 10)
        {
            GPIO_Set(BUZZER_GPIO, 1);
        }
        else
        {
            TickType_t period = (d < 20)
                ? pdMS_TO_TICKS(100)
                : pdMS_TO_TICKS(300);

            if (next == 0) next = now;

            if ((int32)(now - next) >= 0)
            {
                buz_on ^= 1;
                GPIO_Set(BUZZER_GPIO, buz_on);
                next = now + period;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
