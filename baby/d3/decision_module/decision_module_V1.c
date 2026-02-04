#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <inttypes.h>

#include "decide_mode.h"


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

uint8_t RX_waypoint(to_vcp_msg_t *msg){

    return 0;
}

uint8_t RX_aruco_marker(to_vcp_msg_t *msg){


    return 0;
}



int main(void){
    uint32_t seq = 0;
    init_decide_mode_msg();
    static uint32_t last_aruco_rx_time;
    static uint32_t last_wp_rx_time;

    while(1){
    uint8_t ret = 0;        // Handler
    uint32_t time_now = 0;
    
    time_now = GetTickCount();

    msg.cpu_time_ms = time_now;
    msg.seq = seq++;
    
    //aruco marker input
    ret = RX_aruco_marker(&msg);
    if(ret){
        fprintf(stderr,"[seq : %u| time : %u] Failed to receive aruco marker data\n",msg.seq, time_now);
    }

    if(!ret){
        msg.aruco_valid = !ret;
        last_aruco_rx_time = time_now;
    }
    msg.aruco_age_ms = (uint16_t)(time_now - last_aruco_rx_time);   // timeout check , 16bit casting
    
    // waypoint input
    ret = RX_waypoint(&msg);
    if(ret){
        fprintf(stderr,"[seq : %u| time : %u] Failed to receive waypoint data\n",msg.seq, time_now);
    }
    if(!ret){
        msg.wp_valid = !ret;
        last_wp_rx_time = time_now;
    }
    msg.wp_age_ms = (uint16_t)(time_now - last_wp_rx_time);         // timeout check , 16bit casting

    ret = decide_mode_step(&msg);

    time_now = GetTickCount();
    printf("[seq : %u| time : %u] mode=%u reason=%u leader_state : %u\n", 
            msg.seq, time_now, msg.mode, msg.reason, msg.leader_state);
    printf("[ArUco status] aruco_valid : %u, aruco_age_ms : %u, aruco_dist_mm : %d, aruco_x_norm_q15 : %d",
          msg.aruco_valid, msg.aruco_age_ms, msg.aruco_dist_mm, msg.aruco_x_norm_q15);
    printf("[Waypoint status] wp_valid: %u, wp_age_ms : %u, leader_x_mm : %d, leader_y_mm : %d",
          msg.wp_valid, msg.wp_age_ms, msg.leader_x_mm, msg.leader_y_mm);
    
    usleep(50000); // 50ms delay
    }

    return 0;
}