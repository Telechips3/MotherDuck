#ifndef _VCP_PROTOCOL_H_
#define _VCP_PROTOCOL_H_

#include "sal_api.h"

#pragma pack(push, 1) // 패딩 제거 (리눅스와 동일하게 설정)
typedef struct {
    uint32 seq;
    uint32 cpu_time_ms;
    uint8  mode;
    uint8  leader_state;
    uint8  aruco_valid;
    uint16 aruco_age_ms;
    int16  aruco_dist_mm;
    int16  aruco_x_norm_q15;
    uint8  wp_valid;
    uint16 wp_age_ms;
    int32  leader_x_mm;
    int32  leader_y_mm;
    float  tune_kp;
    float  tune_ki;
    float  tune_kd;
    uint8  reason;
} to_vcp_msg_t;

typedef struct {
    uint8        magic;
    to_vcp_msg_t vcp_msg;
    uint16       crc16;
} to_vcp_spi_msg_t;
#pragma pack(pop)

#endif