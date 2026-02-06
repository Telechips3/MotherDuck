// SPDX-License-Identifier: Apache-2.0
#include "../team2_header.h"
#include "ultrasonic.h"
#include <app_cfg.h>
#include <debug.h>

static void Buzzer_Init(void)
{
    GPIO_Config(BUZZER_GPIO,
        GPIO_OUTPUT | GPIO_NOPULL | GPIO_DS(4) | GPIO_FUNC(0));
    GPIO_Set(BUZZER_GPIO, 0);
}

/* 내부 함수 */
static uint32 Ultrasonic_GetEchoCount(void);

static void Ultrasonic_Init(void)
{
    GPIO_Config(ULTRA_TRIG_GPIO,
        GPIO_OUTPUT | GPIO_NOPULL | GPIO_DS(4) | GPIO_FUNC(0));

    GPIO_Config(ULTRA_ECHO_GPIO,
        GPIO_INPUT | GPIO_NOPULL | GPIO_INPUTBUF_EN | GPIO_FUNC(0));

    GPIO_Set(ULTRA_TRIG_GPIO, 0);

}

static uint32 Ultrasonic_GetDistance_cm(void)
{
    uint32 cnt;

    GPIO_Set(ULTRA_TRIG_GPIO, 1);
    for (volatile int i = 0; i < 300; i++);
    GPIO_Set(ULTRA_TRIG_GPIO, 0);

    cnt = Ultrasonic_GetEchoCount();
    if (cnt == ULTRA_ECHO_TIMEOUT) return 0;

    float echo_us = cnt * ULTRA_CNT_TO_US_K;
    uint32 dist_cm = (uint32)(echo_us / 58.0f);

    if (dist_cm < 2 || dist_cm > 400) return 0;
    return dist_cm;
}

static uint32 Ultrasonic_GetEchoCount(void)
{
    uint32 cnt = 0, guard = 0;

    while (GPIO_Get(ULTRA_ECHO_GPIO) == 0)
        if (++guard > 3000000) return ULTRA_ECHO_TIMEOUT;

    while (GPIO_Get(ULTRA_ECHO_GPIO) == 1)
        if (++cnt > 3000000) return ULTRA_ECHO_TIMEOUT;

    return cnt;
}

/* ================= Task ================= */
void UltrasonicTask(void *pArg)
{
    (void)pArg;
    Ultrasonic_Init();
    Buzzer_Init();
    
    uint32 buz_on = 0;
    TickType_t next = 0;

    while (1)
    {
        uint32 d  = Ultrasonic_GetDistance_cm();
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

        SAL_TaskSleep(500);
    }
}

SALRetCode_t UltrasonicTaskCreate(void)
{
    static uint32 ultraTaskID;
    static uint32 ultraTaskStk[ACFG_TASK_NORMAL_STK_SIZE];
    return SAL_TaskCreate(&ultraTaskID,
                          (const uint8 *)"Ultrasonic Task",
                          UltrasonicTask,
                          ultraTaskStk,
                          ACFG_TASK_NORMAL_STK_SIZE,
                          SAL_PRIO_APP_CFG,
                          NULL);
}
