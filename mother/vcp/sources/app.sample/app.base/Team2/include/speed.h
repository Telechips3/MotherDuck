/* speed.h */
#ifndef _SPEED_H_
#define _SPEED_H_

#include "../team2_header.h"

#define PERIOD_NS           1000000          // 1kHz
#define DUTY_SLOW_NS        450000           // 30% 속도
#define DUTY_STOP_NS        0                // 정지
#define MOTOR_PWM_CH        0                // PDM 채널 0 (GPIO A10)
#define PERIOD_NS           1000000          // 주기: 1ms (1kHz)


void control_motor_drive(uint32 cmd);

#endif