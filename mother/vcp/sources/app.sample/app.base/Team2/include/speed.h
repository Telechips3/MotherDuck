/* speed.h */
#ifndef _SPEED_H_
#define _SPEED_H_

#include "../team2_header.h"

// 명령 코드 정의
#define CMD_FORWARD         0x00
#define CMD_LEFT            0x01
#define CMD_BACKWARD        0x02
#define CMD_RIGHT           0x03

// 조합 명령
#define CMD_FORWARD_LEFT    0x04  // W+A
#define CMD_FORWARD_RIGHT   0x05  // W+D
#define CMD_BACKWARD_LEFT   0x06  // S+A
#define CMD_BACKWARD_RIGHT  0x07  // S+D

#define CMD_SPEED_BASE      0x10  // 0x10~0x18

// PWM 설정
#define PERIOD_NS           1000000

// 속도 단계별 Duty (10% ~ 90%)
#define DUTY_LEVEL_1_NS     100000
#define DUTY_LEVEL_2_NS     200000
#define DUTY_LEVEL_3_NS     300000
#define DUTY_LEVEL_4_NS     400000
#define DUTY_LEVEL_5_NS     500000
#define DUTY_LEVEL_6_NS     600000
#define DUTY_LEVEL_7_NS     700000
#define DUTY_LEVEL_8_NS     800000
#define DUTY_LEVEL_9_NS     900000
#define DUTY_STOP_NS        0

#define MOTOR_PWM_CH        0

void control_motor_drive(uint32 cmd, uint32 is_steer_cmd);
void control_motor_speed(uint32 speed_level);

#endif