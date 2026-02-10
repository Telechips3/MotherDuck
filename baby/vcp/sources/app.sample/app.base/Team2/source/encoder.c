// SPDX-License-Identifier: Apache-2.0
/*
***************************************************************************************************
*
*   FileName : encoder.c
*
***************************************************************************************************
*/

#include "../team2_header.h"
#include "../include/encoder.h"
#include <gpio.h>
#include <app_cfg.h>
#include <debug.h>
#include "../include/speed.h"

/* ===== Internal State ===== */
volatile int32 s_encCnt = 0;
//static uint8_t s_prev_state = 0;

static int32 s_prevCnt = 0;
static uint32 s_prevTickMs = 0;

static float s_distanceCm = 0.0f;
static float s_speedCms = 0.0f;

static uint8 s_lastA = 0;
static uint8 s_lastB = 0;
static uint32 s_lastTransMs = 0;
static uint8 s_prevA = 0;
static uint32 s_lastCountMs = 0;

/* task variables
 static uint32 encTaskID;
 static uint32 encTaskStk[ENCODER_TASK_STACK_SIZE];
*/

// static inline uint32 get_tick_ms(void)
// {
//     uint32 ret;
//     (uint32)(SAL_GetTickCount(&ret) * portTICK_PERIOD_MS);
//     return ret;
// }
static inline uint32 get_tick_ms(void)
{
    uint32 tick = 0;
    (void)SAL_GetTickCount(&tick);
    return tick * portTICK_PERIOD_MS;
}

static void Encoder_ISR_Handler(void *args)
{
    uint8 curA = GPIO_Get(ENC_A_GPIO);
    uint8 curB = GPIO_Get(ENC_B_GPIO);

    // A falling edge에서만 호출된다는 전제
    if (curA == curB) {
        s_encCnt++;
    } else {
        s_encCnt--;
    }
    
}

// static void Encoder_ISR_Handler(void *args)
// {
//     uint8_t curA = GPIO_Get(ENC_A_GPIO);
//     uint8_t curB = GPIO_Get(ENC_B_GPIO);
//     uint8_t cur_state = (curA << 1) | curB;

//     // 3. 이전 상태와 현재 상태를 조합 (4비트 데이터)
//     // [이전A][이전B][현재A][현재B]
//     uint8_t state_transition = (s_prev_state << 2) | cur_state;

//     // 4. 상태 변화에 따른 카운트 (4체배 로직)
//     switch (state_transition)
//     {
//     // 정회전 케이스 (00->01, 01->11, 11->10, 10->00)
//     case 0b0001:
//     case 0b0111:
//     case 0b1110:
//     case 0b1000:
//         s_encCnt++;
//         break;

//     // 역회전 케이스 (00->10, 10->11, 11->01, 01->00)
//     case 0b0010:
//     case 0b1011:
//     case 0b1101:
//     case 0b0100:
//         s_encCnt--;
//         break;

//     default:
//         break;
//     }

//     //
//     // switch (state_transition)
//     // {
//     // // 1. 정회전(CW) 케이스: A가 먼저 변함
//     // case 0b0010: // 00 -> 10 (A Rising)
//     // case 0b1011: // 10 -> 11 (B Rising - 이때 A는 High!)
//     // case 0b1101: // 11 -> 01 (A Falling)
//     // case 0b0100: // 01 -> 00 (B Falling)
//     //     s_encCnt++;
//     //     break;

//     // // 2. 역회전(CCW) 케이스: B가 먼저 변함
//     // case 0b0001: // 00 -> 01 (B Rising)
//     // case 0b0111: // 01 -> 11 (A Rising)
//     // case 0b1110: // 11 -> 10 (B Falling)
//     // case 0b1000: // 10 -> 00 (A Falling)
//     //     s_encCnt--;
//     //     break;

//     // default:
//     //     // 변화 없음 또는 노이즈(00->11 등)
//     //     break;

//     // }
//     s_prev_state = cur_state;
//     //mcu_printf("encoder Cnt: %d\n", s_encCnt);
// }

/* ===== Init ===== */
void Encoder_Init(void)
{
    mcu_printf("[ENC] encoder Init started\n");
    GPIO_Config(ENC_A_GPIO, GPIO_INPUT | GPIO_PULLUP | GPIO_INPUTBUF_EN | GPIO_FUNC(0));
    GPIO_Config(ENC_B_GPIO, GPIO_INPUT | GPIO_PULLUP | GPIO_INPUTBUF_EN | GPIO_FUNC(0));

    GPIO_IntExtSet(EIT_ENCODER_A, ENC_A_GPIO);
    (void)GIC_IntVectSet(EIT_ENCODER_A, GIC_PRIORITY_NO_MEAN, GIC_INT_TYPE_EDGE_FALLING, (GICIsrFunc)(Encoder_ISR_Handler), (void *)0);
    (void)GIC_IntSrcEn(EIT_ENCODER_A);

    s_encCnt = 0;
    s_prevCnt = 0;
    s_prevTickMs = get_tick_ms();
    s_lastA = GPIO_Get(ENC_A_GPIO);
    s_lastB = GPIO_Get(ENC_B_GPIO);
    s_lastTransMs = get_tick_ms();
    s_prevA = s_lastA;
    s_lastCountMs = s_lastTransMs;

    mcu_printf("[ENC] Init done\n");
}

/* ===== Update (call frequently, e.g. 1ms~10ms) ===== */
void Encoder_Update(void)
{
    uint8 curA = GPIO_Get(ENC_A_GPIO);
    uint8 curB = GPIO_Get(ENC_B_GPIO);
    uint32 nowMs = get_tick_ms();
    if ((nowMs - s_lastTransMs) >= 2)
    {
        // Count only A rising edges (slower, less sensitive)
        if (s_prevA == 0 && curA == 1)
        {
            if ((nowMs - s_lastCountMs) >= ENC_MIN_PULSE_MS)
            {
                (void)SAL_CoreCriticalEnter();
                s_encCnt++;
                (void)SAL_CoreCriticalExit();
                s_lastCountMs = nowMs;
            }
            s_lastTransMs = nowMs;
        }
    }

    s_lastA = curA;
    s_lastB = curB;
    s_prevA = curA;
}

/* ===== Speed / Distance calculation (recommend 1s period) ===== */
void Encoder_CalcSpeed(void)
{
    uint32 nowMs = get_tick_ms();
    uint32 dtMs = nowMs - s_prevTickMs;

    (void)SAL_CoreCriticalEnter();
    int32 diffCnt = s_encCnt - s_prevCnt;
    (void)SAL_CoreCriticalExit();

    if (dtMs == 0)
        return;

    // Count on A rising edges => 1 count per pulse
    float cm_per_count = (PI * WHEEL_DIAMETER_CM) / ENCODER_CPR;

    s_distanceCm += diffCnt * cm_per_count;
    s_speedCms = (diffCnt * cm_per_count) / ((float)dtMs / 1000.0f);

    s_prevCnt = s_encCnt;
    s_prevTickMs = nowMs;
    /*===========debug==============*/
    // int dist_x10 = (int)(s_distanceCm * 10.0f);
    // int speed_x10 = (int)(s_speedCms * 10.0f);
    // mcu_printf("[ENC] CNT=%d DIST=%d.%d cm SPEED=%d.%d cm/s A=%d B=%d\n",
    //            s_encCnt,
    //            dist_x10 / 10, dist_x10 % 10,
    //            speed_x10 / 10, speed_x10 % 10,
    //            s_lastA, s_lastB);
}

/* ===== Task ===== */
/* not use Task in encoder (legecy)
void EncoderTask(void *pArg)
{
    (void)pArg;
    Encoder_Init();

    uint32 lastCalcMs = get_tick_ms();

    while (1)
    {
        SAL_TaskSleep(ENCODER_TASK_SLEEP_MS);
    }
}

SALRetCode_t EncoderTaskCreate(void)
{
    SALRetCode_t err = SAL_TaskCreate(&encTaskID,
                                      (const uint8 *)"Encoder Task",
                                      EncoderTask,
                                      encTaskStk,
                                      ENCODER_TASK_STACK_SIZE,
                                      ENCODER_TASK_PRIORITY,
                                      NULL);
    mcu_printf("Encoder task create: %d\n", (int)err);
    return err;
}
*/

/* ===== Getter ===== */
int32 Encoder_GetCount(void)
{
    int32 ret;
    (void)SAL_CoreCriticalEnter();
    ret = s_encCnt;
    (void)SAL_CoreCriticalExit();

    return ret;
}

float Encoder_GetDistanceCm(void)
{
    float ret;
    (void)SAL_CoreCriticalEnter();
    ret = s_distanceCm;
    (void)SAL_CoreCriticalExit();
    return ret;
}

float Encoder_GetSpeedCms(void)
{
    float ret;
    (void)SAL_CoreCriticalEnter();
    ret = s_speedCms;
    (void)SAL_CoreCriticalExit();
    return ret;
}

void Encoder_ResetCount(void)
{
    (void)SAL_CoreCriticalEnter();
    s_encCnt = 0;
    s_prevCnt = 0;
    s_distanceCm = 0.0f;
    s_speedCms = 0.0f;
    (void)SAL_CoreCriticalExit();
}



float Encoder_GetDeltaDistanceCm(void)
{
    static int32 prevCnt_local = 0;
    int32 cnt;
    (void)SAL_CoreCriticalEnter();
    cnt = s_encCnt;
    (void)SAL_CoreCriticalExit();

    int32 diff = cnt - prevCnt_local;
    prevCnt_local = cnt;

    float cm_per_count = (PI * WHEEL_DIAMETER_CM) / ENCODER_CPR;
    static uint32 cnt1 = 0;
    cnt1++;
    if ((cnt1 % 50) == 0) {
        mcu_printf("[ENC] cnt=%d diff=%d\n", cnt, diff);
    }
    return diff * cm_per_count;
}