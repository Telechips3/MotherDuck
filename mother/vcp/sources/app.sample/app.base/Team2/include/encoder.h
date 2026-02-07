#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <sal_api.h>
#include <gpio.h>
#include "FreeRTOS.h"
#include "task.h"

extern volatile int32 s_encCnt;

/* API */
void Encoder_Init(void);
SALRetCode_t EncoderTaskCreate(void);
void EncoderTask(void *pArg);
int32 Encoder_GetCount(void);
void Encoder_ResetCount(void);

#endif /* _ENCODER_H_ */
