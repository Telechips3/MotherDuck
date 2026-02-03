#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "decide_mode.h"

// ====== reason code (debug) =====
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

// ===== Payload msg ==============
to_vcp_msg_t msg = {0};


void init_decide_mode_msg(void) {
    memset(&msg, 0, sizeof(msg));
}

uint64_t GetTickCount() 
{
    struct timespec ts;
    
    // CLOCK_MONOTONIC: 시간 수정돼도 영향 안 받는 절대 시간
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        // 에러 나면 0 리턴하거나 로그 찍으셈
        perror("clock_gettime failed");
        return 0;
    }

    // 초(s) * 1000 + 나노초(ns) / 1,000,000 = 밀리초(ms)
    return (uint64_t)ts.tv_sec * 1000 + (ts.tv_nsec / 1000000);
}

int main(void){
    uint8_t seq_cnt = 0;

    while(1){

    init_decide_mode_msg();

    msg.seq = seq_cnt++;
    //aruco marker input
    
    // waypoint input

    
    decide_mode_step(&msg);

    msg.cpu_time_ms = (uint32_t)(GetTickCount() & 0xFFFFFFFF);
    printf("mode=%d reason=%d\n", msg.mode, msg.reason);
    


    }

    return 0;
}