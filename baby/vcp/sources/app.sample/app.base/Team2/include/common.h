#pragma once
#include <stdint.h>

typedef enum {
	LEADER_OK = 0,
	LEADER_DEGRADED = 1,
	LEADER_FAULT = 2,
	LEADER_ESTOP = 3
} leader_state_t;

typedef enum {
    REASON_NONE = 0,
    REASON_CPU_STALE = 1,
    REASON_LEADER_ESTOP = 2,
    REASON_LEADER_FAULT = 3,
    REASON_WP_TIMEOUT = 4,
    REASON_ARUCO_TIMEOUT = 5,
    REASON_WP_RECOVERED = 6,
    REASON_ARUCO_RECOVERED = 7,
    REASON_BOTH_LOST = 8, 
    REASON_INIT = 9,
    REASON_TRANSMITION = 10,
    REASON_NULL = 11
} mode_reason_t;

typedef struct {
	uint32 seq;
	uint32 cpuime_ms;  // CPU monotonic ms -> 메시지 자체의 timestamp를 확인해서 VCP에서 핸들링할 수 있게

	uint8 mode;
	uint8 leader_state;

	// ArUco
	uint8  aruco_valid;   // aruco 마커 감지 됐는지 안됐는지 
	uint16 aruco_age_ms;  // 받은지 얼마나 됐는지(신선도) -> age로 timeout 판단
	int16  aruco_dist_mm; // 정수 추천 (mm)
	int16  aruco_x_norm_q15; // -32768~32767 (=-1.0~+1.0)

	// Waypoint
	uint8  wp_valid;      // waypoint 수신 됐는지
	uint16 wp_age_ms;     // waypoint 얼마나 됐는지
	int32  leader_x_mm;   // mm (정수표현 위해 mm단위)
	int32  leader_y_mm;

	//Debug
	uint8  reason;        // Mode가 왜 바뀌었는지
} rx_msg_t;