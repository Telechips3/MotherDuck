#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <sal_api.h>
#include <gpio.h>
#include "FreeRTOS.h"
#include "task.h"

/* ===== Encoder GPIO ===== */
// #define ENC_A_GPIO   GPIO_GPA(22)
// #define ENC_B_GPIO   GPIO_GPA(21)

/* ===== Motor GPIO ===== */
// #define MOTOR_IN1_GPIO   GPIO_GPA(5)
// #define MOTOR_IN2_GPIO   GPIO_GPA(9)

extern volatile int32 s_encCnt;

/* ===== Encoder → Distance / Speed ===== */
#define WHEEL_DIAMETER_CM   6.5f // Wheel diameter (cm)
#define ENCODER_CPR         11.0f      // Pulses per rev (A phase)
#define PI                  3.141592f
#define ENC_MIN_PULSE_MS    5

/* API */
void Encoder_Init(void);
void Encoder_Update(void);
void Encoder_CalcSpeed(void);
void EncoderTask(void *pArg);
SALRetCode_t EncoderTaskCreate(void);
int32 Encoder_GetCount(void);
float Encoder_GetDistanceCm(void);
float Encoder_GetSpeedCms(void);
void Encoder_ResetCount(void);
float Encoder_GetDeltaDistanceCm(void);

#endif /* _ENCODER_H_ */
