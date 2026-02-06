// Waypoint_RX.h
#ifndef WAYPOINT_RX_H
#define WAYPOINT_RX_H

#include "decide_mode.h"

// 모듈 초기화 (UDS bind). path가 NULL이면 기본 경로 사용
void waypoint_rx_init(const char* path);

// waypoint 데이터를 수신하여 msg의 leader_x_mm, leader_y_mm 채움
// 리턴: 0(신규 데이터 수신), 1(데이터 없음/에러)
uint8_t RX_waypoint(to_vcp_msg_t* msg);

#endif
