// SPDX-License-Identifier: Apache-2.0
#include "ultrasonic.h"
#include <debug.h>

/* main.c에 있는 전역 변수 사용 */
extern volatile uint32 g_ultraDistanceCm;

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

    while (1)
    {
        g_ultraDistanceCm = Ultrasonic_GetDistance_cm();
        mcu_printf("[ULTRA] %d cm\n", g_ultraDistanceCm);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
