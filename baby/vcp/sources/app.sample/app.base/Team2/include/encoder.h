#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <sal_api.h>
#include <gpio.h>
#include "FreeRTOS.h"
#include "task.h"

/* ===== Encoder GPIO ===== */
#define ENC_A_GPIO   GPIO_GPA(9)
#define ENC_B_GPIO   GPIO_GPA(8)

extern volatile int32 g_enc_count;

/* API */
void Encoder_Init(void);
void EncoderTask(void *pArg);
int32 Encoder_GetCount(void);
void Encoder_ResetCount(void);


#endif /* _ENCODER_H_ */
