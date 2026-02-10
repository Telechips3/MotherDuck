// SPDX-License-Identifier: Apache-2.0
#include "../team2_header.h"
#include "ultrasonic.h"
#include <app_cfg.h>
#include <debug.h>

static uint32 ultraTaskID;
static uint32 ultraTaskStk[ULTRASONIC_TASK_STACK_SIZE];

// --- [Low Level] 부저 제어 ---
void Buzzer_Init(void) {
    GPIO_Config(BUZZER_GPIO, GPIO_OUTPUT | GPIO_NOPULL | GPIO_DS(4) | GPIO_FUNC(0));
    GPIO_Set(BUZZER_GPIO, 0);
}

void Buzzer_Set(uint8_t on) {
    GPIO_Set(BUZZER_GPIO, on ? 1 : 0);
}

// --- [Low Level] 초음파 센서 ---
void Ultrasonic_Init(void) {
    GPIO_Config(ULTRA_TRIG_GPIO, GPIO_OUTPUT | GPIO_NOPULL | GPIO_DS(4) | GPIO_FUNC(0));
    GPIO_Config(ULTRA_ECHO_GPIO, GPIO_INPUT | GPIO_NOPULL | GPIO_INPUTBUF_EN | GPIO_FUNC(0));
    GPIO_Set(ULTRA_TRIG_GPIO, 0);
}

static uint32 Ultrasonic_GetEchoCount(void) {
    uint32 cnt = 0, guard = 0;
    // Echo High가 올 때까지 대기
    while (GPIO_Get(ULTRA_ECHO_GPIO) == 0) {
        if (++guard > 1000000) return ULTRA_ECHO_TIMEOUT;
    }
    // Echo High 유지 시간 측정
    while (GPIO_Get(ULTRA_ECHO_GPIO) == 1) {
        if (++cnt > 1000000) return ULTRA_ECHO_TIMEOUT;
    }
    return cnt;
}

uint32 Ultrasonic_GetDistance_cm(void) {
    GPIO_Set(ULTRA_TRIG_GPIO, 1);
    for (volatile int i = 0; i < 300; i++); // 10us Trigger
    GPIO_Set(ULTRA_TRIG_GPIO, 0);

    uint32 cnt = Ultrasonic_GetEchoCount();
    if (cnt == ULTRA_ECHO_TIMEOUT) return 0;

    float echo_us = cnt * ULTRA_CNT_TO_US_K;
    uint32 dist_cm = (uint32)(echo_us / 58.0f);

    if (dist_cm < ULTRA_DISTANCE_MIN || dist_cm > ULTRA_DISTANCE_MAX) return 0;
    return dist_cm;
}

// --- [Task] Buzzer Logic ---
void UltrasonicTask(void *pArg) {
    (void)pArg;
    Ultrasonic_Init();
    Buzzer_Init();
    
    uint8_t buz_state = 0;
    TickType_t next_toggle = 0;

    while (1) {
        uint32 d = Ultrasonic_GetDistance_cm();
        TickType_t now = xTaskGetTickCount();

        if (d == 0 || d >= BUZ_FAR_LIMIT) {
            Buzzer_Set(0);
            buz_state = 0;
            next_toggle = 0;
        } 
        else if (d < BUZ_NEAR_LIMIT) {
            Buzzer_Set(1); // 10cm 미만 : 상시 ON
        } 
        else {
            // 거리별 비프음 주기 설정
            TickType_t period = (d < BUZ_MID_LIMIT) ? pdMS_TO_TICKS(100) : pdMS_TO_TICKS(300);

            if (next_toggle == 0) next_toggle = now;

            if (now >= next_toggle) {
                buz_state ^= 1;
                Buzzer_Set(buz_state);
                next_toggle = now + period;
            }
        }
        SAL_TaskSleep(ULTRASONIC_TASK_SLEEP_MS);
    }
}

SALRetCode_t UltrasonicTaskCreate(void)
{
    return SAL_TaskCreate(&ultraTaskID,
                          (const uint8 *)"Ultrasonic Task",
                          UltrasonicTask,
                          ultraTaskStk,
                          ULTRASONIC_TASK_STACK_SIZE,
                          ULTRASONIC_TASK_PRIORITY,
                          NULL);
}
