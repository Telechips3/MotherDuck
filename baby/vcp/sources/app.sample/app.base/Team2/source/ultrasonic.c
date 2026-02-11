// SPDX-License-Identifier: Apache-2.0
#include "../team2_header.h"
#include "ultrasonic.h"
#include <app_cfg.h>
#include <debug.h>

// static uint32 ultraTaskID;
// static uint32 ultraTaskStk[ULTRASONIC_TASK_STACK_SIZE];

static void Delay_us(uint32_t us)
{
    volatile uint32_t count = us * 10; // MCU 속도에 맞춰 튜닝 필요
    while (count--) {
        __asm("nop");
    }
}

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
    uint32 cnt = 0;
    // STM32 F429ZI (180MHz) 기준, 1ms 대기용 루프 횟수 (대략적인 값)
    // 실제 환경에서 오실로스코프로 찍어보면서 튜닝하는 게 국룰임
    const uint32 LOOP_PER_MS = 30000; 
    uint32 timeout_cnt = 0;

    // 1. Echo High가 올 때까지 대기 (Start Wait)
    // -> 루프 돌리면서 30ms 넘어가면 컷!
    while (GPIO_Get(ULTRA_ECHO_GPIO) == 0) {
        timeout_cnt++;
        // 대략 30ms 타임아웃 체크
        if (timeout_cnt > (ULTRA_TIMEOUT_MS * LOOP_PER_MS)) {
            return ULTRA_ECHO_TIMEOUT;
        }
    }

    // 2. Echo High 유지 시간 측정 (Measure Duration)
    // -> 여기서 1us 딜레이 루프를 돌려서 cnt가 곧 us가 되게 함
    while (GPIO_Get(ULTRA_ECHO_GPIO) == 1) {
        // 1us 딜레이용 for 루프 (F429ZI 기준 약 15~20회 루프가 1us 정도임)
        // 컴파일러 최적화 방지를 위해 i는 volatile 필수!
        for (volatile int i = 0; i < 20; i++); 
        
        cnt++; 

        // 30000us (30ms) 넘어가면  타임아웃
        if (cnt > (ULTRA_TIMEOUT_MS * 1000)) { 
            return ULTRA_ECHO_TIMEOUT;
        }
    }
    
    return cnt;
}

uint32 Ultrasonic_GetDistance_cm(void) {
    // 1. Trigger 발사 (10us)
    GPIO_Set(ULTRA_TRIG_GPIO, 1);
    
    // 만약 Delay_us 함수가 있다면 Delay_us(10); 이 훨씬 정확해.
    // 없다면 기존 for 루프 유지해도 됨.
    for (volatile int i = 0; i < 300; i++); 
    
    GPIO_Set(ULTRA_TRIG_GPIO, 0);

    // 2. Echo 시간 측정 (단위: us)
    // 아까 우리가 GetEchoCount 안에서 Delay_us(1)를 썼으니까, 
    // 리턴된 cnt는 이미 '마이크로초(us)' 단위임!
    uint32 echo_us = Ultrasonic_GetEchoCount();
    
    // 3. 타임아웃 체크
    if (echo_us == ULTRA_ECHO_TIMEOUT) return ULTRA_ECHO_TIMEOUT;

    // 4. 거리 계산 (cm = us / 58)
    // 🚨 [수정 포인트] 복잡한 변환 상수(K) 싹 다 갖다 버리고 바로 나눔!
    uint32 dist_cm = echo_us / 58;

    // 5. 범위 체크
    if (dist_cm < ULTRA_DISTANCE_MIN || dist_cm > ULTRA_DISTANCE_MAX) return 0;
    
    return dist_cm;
}

// // --- [Task] Buzzer Logic ---
// void UltrasonicTask(void *pArg) {
//     (void)pArg;
//     Ultrasonic_Init();
//     Buzzer_Init();
    
//     uint8_t buz_state = 0;
//     TickType_t next_toggle = 0;

//     while (1) {
//         uint32 d = Ultrasonic_GetDistance_cm();
//         TickType_t now = xTaskGetTickCount();

//         if (d == 0 || d >= BUZ_FAR_LIMIT) {
//             Buzzer_Set(0);
//             buz_state = 0;
//             next_toggle = 0;
//         } 
//         else if (d < BUZ_NEAR_LIMIT) {
//             Buzzer_Set(1); // 10cm 미만 : 상시 ON
//         } 
//         else {
//             // 거리별 비프음 주기 설정
//             TickType_t period = (d < BUZ_MID_LIMIT) ? pdMS_TO_TICKS(100) : pdMS_TO_TICKS(300);

//             if (next_toggle == 0) next_toggle = now;

//             if (now >= next_toggle) {
//                 buz_state ^= 1;
//                 Buzzer_Set(buz_state);
//                 next_toggle = now + period;
//             }
//         }
//         SAL_TaskSleep(ULTRASONIC_TASK_SLEEP_MS);
//     }
// }

// SALRetCode_t UltrasonicTaskCreate(void)
// {
//     return SAL_TaskCreate(&ultraTaskID,
//                           (const uint8 *)"Ultrasonic Task",
//                           UltrasonicTask,
//                           ultraTaskStk,
//                           ULTRASONIC_TASK_STACK_SIZE,
//                           ULTRASONIC_TASK_PRIORITY,
//                           NULL);
// }
