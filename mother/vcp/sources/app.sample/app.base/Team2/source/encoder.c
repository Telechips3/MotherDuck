// SPDX-License-Identifier: Apache-2.0
#include "../include/encoder.h"
#include "../team2_header.h"
#include <gpio.h>
#include <bsp.h>
#include <debug.h>

/* ===== Internal Variable ===== */
volatile int32 g_enc_count = 0;
static uint8_t s_prev_state = 0;

static uint32 encTaskID;
static uint32 encTaskStk[ENCODER_TASK_STACK_SIZE];

static void Encoder_ISR_Handler(void *args)
{
    uint8_t curA = GPIO_Get(ENC_A_GPIO);
    uint8_t curB = GPIO_Get(ENC_B_GPIO);
    uint8_t cur_state = (curA << 1) | curB;

    // 3. 이전 상태와 현재 상태를 조합 (4비트 데이터)
    // [이전A][이전B][현재A][현재B]
    uint8_t state_transition = (s_prev_state << 2) | cur_state;

    // 4. 상태 변화에 따른 카운트 (4체배 로직)
    switch (state_transition)
    {
    // 정회전 케이스 (00->01, 01->11, 11->10, 10->00)
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
        g_enc_count++;
        break;

    // 역회전 케이스 (00->10, 10->11, 11->01, 01->00)
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
        g_enc_count--;
        break;

    default:
        break;
    }

    //
    switch (state_transition)
    {
    // 1. 정회전(CW) 케이스: A가 먼저 변함
    case 0b0010: // 00 -> 10 (A Rising)
    case 0b1011: // 10 -> 11 (B Rising - 이때 A는 High!)
    case 0b1101: // 11 -> 01 (A Falling)
    case 0b0100: // 01 -> 00 (B Falling)
        g_enc_count++;
        break;

    // 2. 역회전(CCW) 케이스: B가 먼저 변함
    case 0b0001: // 00 -> 01 (B Rising)
    case 0b0111: // 01 -> 11 (A Rising)
    case 0b1110: // 11 -> 10 (B Falling)
    case 0b1000: // 10 -> 00 (A Falling)
        g_enc_count--;
        break;

    default:
        // 변화 없음 또는 노이즈(00->11 등)
        break;

        s_prev_state = cur_state;
        mcu_printf("encoder Cnt: %d\n", g_enc_count);
    }
}

/* ===== Init ===== */
void Encoder_Init(void)
{
    GPIO_Config(ENC_A_GPIO, GPIO_INPUT | GPIO_PULLUP | GPIO_INPUTBUF_EN | GPIO_FUNC(0));
    GPIO_Config(ENC_B_GPIO, GPIO_INPUT | GPIO_PULLUP | GPIO_INPUTBUF_EN | GPIO_FUNC(0));

    GPIO_IntExtSet(EIT_ENCODER, ENC_A_GPIO);
    (void)GIC_IntVectSet(EIT_ENCODER, GIC_PRIORITY_NO_MEAN, GIC_INT_TYPE_EDGE_FALLING, (GICIsrFunc)(Encoder_ISR_Handler), (void *)0);
    (void)GIC_IntSrcEn(EIT_ENCODER);

    mcu_printf("[ENC] Init done\n");
}

/* ===== Encoder Task ===== */
void EncoderTask(void *pArg)
{
    (void)pArg;
    Encoder_Init();

    while (1)
    {
        SAL_TaskSleep(100);
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

/* ===== Getter / Setter ===== */
int32 Encoder_GetCount(void)
{
    return g_enc_count;
}

void Encoder_ResetCount(void)
{
    g_enc_count = 0;
}
