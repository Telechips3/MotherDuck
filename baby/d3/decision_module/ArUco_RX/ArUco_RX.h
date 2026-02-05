// ArUco_RX.h
#ifndef ARUCO_RX_H
#define ARUCO_RX_H

#include "decide_mode.h"

// 모듈 초기화 및 소켓 연결 시도
void aruco_rx_init(void);

// 데이터를 수신하여 msg 구조체의 aruco_dist_mm, aruco_x_norm_q15를 채움
// 리턴: 0(신규 데이터 수신), 1(데이터 없음/에러)
uint8_t RX_aruco_marker(to_vcp_msg_t *msg);

#endif