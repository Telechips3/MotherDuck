#ifndef _STEER_H_
#define _STEER_H_

#include "../team2_header.h"

// 명령 코드 (speed.h와 일치)
#define CMD_LEFT            0x01
#define CMD_RIGHT           0x03
#define CMD_FORWARD_LEFT    0x04
#define CMD_FORWARD_RIGHT   0x05
#define CMD_BACKWARD_LEFT   0x06
#define CMD_BACKWARD_RIGHT  0x07

#define STEER_PERIOD_NS       20000000
#define STEER_MIN_NS          500000
#define STEER_NEUTRAL_NS      1500000
#define STEER_MAX_NS          2500000
#define STEER_STEP_NS         40000
#define STEER_PWM_CH          PDM_CHANNEL_7

void control_steering_step(uint32 cmd);

#endif