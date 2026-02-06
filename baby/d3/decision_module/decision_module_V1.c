#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <inttypes.h>

#include "decide_mode.h"
#include "ArUco_RX.h"
#include "Waypoint_RX.h"


// ===== Payload msg ==============
to_vcp_msg_t msg = {0};

void init_decide_mode_msg(void) {
    memset(&msg, 0, sizeof(msg));
}

uint32_t GetTickCount(void) 
{
    struct timespec ts;
    
    // CLOCK_MONOTONIC: 시간 수정돼도 영향 안 받는 절대 시간
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        // 에러 나면 0 리턴하거나 로그 찍으셈
        perror("clock_gettime failed");
        return 0;
    }

    // 초(s) * 1000 + 나노초(ns) / 1,000,000 = 밀리초(ms)
    uint64_t ms = (uint64_t)ts.tv_sec * 1000ull + (uint64_t)(ts.tv_nsec / 1000000ull);
    return (uint32_t)ms; // 여기서만 32비트로 잘라서 roll-over 자연스럽게
}

int main(int argc, char** argv){
    uint32_t seq = 0;
    init_decide_mode_msg();

    aruco_rx_init();    // ArUco 소켓 통신 초기화 호출
    const char* wp_path = (argc >= 2) ? argv[1] : NULL;
    waypoint_rx_init(wp_path); // Waypoint UDS 수신 초기화

    while(1){
    uint8_t ret = 0;        // Handler
    uint32_t time_now = 0;
    
    time_now = GetTickCount();

    msg.cpu_time_ms = time_now;
    msg.seq = seq++;
    
    //aruco marker input
    uint8_t aruco_ret = RX_aruco_marker(&msg);
    if (aruco_ret == 0) {
        // 새로운 데이터를 성공적으로 받은 경우
    } else {
        // 데이터를 못 받았거나 연결에 문제가 있는 경우
    }
    
    // waypoint input
    ret = RX_waypoint(&msg);
    if(ret){
        fprintf(stderr,"[seq : %u| time : %u] Failed to receive waypoint data\n",msg.seq, time_now);
    }

    ret = decide_mode_step(&msg);

    time_now = GetTickCount();
    printf("[seq : %u| time : %u] mode=%u reason=%u leader_state : %u\n", 
            msg.seq, time_now, msg.mode, msg.reason, msg.leader_state);
    printf("[ArUco status] aruco_valid : %u, aruco_age_ms : %u, aruco_dist_mm : %d, aruco_x_norm_q15 : %d\n",
          msg.aruco_valid, msg.aruco_age_ms, msg.aruco_dist_mm, msg.aruco_x_norm_q15);
    printf("[Waypoint status] wp_valid: %u, wp_age_ms : %u, leader_x_mm : %d, leader_y_mm : %d\n",
          msg.wp_valid, msg.wp_age_ms, msg.leader_x_mm, msg.leader_y_mm);
    
    // 실제 통신 인터페이스 모듈(SPI)

    usleep(50000); // 50ms delay
    }

    return 0;
}
