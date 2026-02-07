// SPDX-License-Identifier: Apache-2.0
#include "encoder.h"
#include <gpio.h>     
#include <bsp.h> 
#include <debug.h>

/* ===== Internal Variable ===== */
static volatile int32 s_encCnt = 0;

/* ===== Encoder Init ===== */
void Encoder_Init(void)
{
    GPIO_Config(ENC_A_GPIO,
        GPIO_INPUT | GPIO_PULLUP | GPIO_INPUTBUF_EN | GPIO_FUNC(0));

    GPIO_Config(ENC_B_GPIO,
        GPIO_INPUT | GPIO_PULLUP | GPIO_INPUTBUF_EN | GPIO_FUNC(0));

}

/* ===== Encoder Task ===== */
void EncoderTask(void *pArg)
{
    (void)pArg;
    Encoder_Init();

    int prev_a = GPIO_Get(ENC_A_GPIO);
    int prev_b = GPIO_Get(ENC_B_GPIO);
    int prev_state = (prev_a << 1) | prev_b;

    while (1)
    {
        int a = GPIO_Get(ENC_A_GPIO);
        int b = GPIO_Get(ENC_B_GPIO);
        int state = (a << 1) | b;

        if (state != prev_state)
        {
            int diff = (prev_state << 2) | state;

            switch (diff)
            {
                // CW
                case 0b0001:
                case 0b0111:
                case 0b1110:
                case 0b1000:
                    s_encCnt++;
                    break;

                // CCW
                case 0b0010:
                case 0b1011:
                case 0b1101:
                case 0b0100:
                    s_encCnt--;
                    break;

                default:
                    // invalid transition
                    break;
            }

            prev_state = state;
        }

        mcu_printf("[ENC] A=%d B=%d CNT=%d\n", a, b, (int)s_encCnt);
        SAL_TaskSleep(5);
    }
}

/* ===== Getter / Setter ===== */
int32 Encoder_GetCount(void)
{
    return s_encCnt;
}

void Encoder_ResetCount(void)
{
    s_encCnt = 0;
}
