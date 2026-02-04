// test_decide_mode.c
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "decide_mode.h"

// decide_mode.c에 있는 reason code랑 숫자 맞춰서 복붙(테스트용)
// (외부에 노출 안 한 상태라 테스트에서만 로컬로 둠)

static const char* mode_str(uint8_t m) {
    switch ((ctrl_mode_t)m) {
    case MODE_ESTOP:          return "ESTOP";
    case MODE_STOP_AND_HOLD:  return "STOP";
    case MODE_FOLLOW_WAYPOINT:return "WAYPOINT";
    case MODE_FOLLOW_VISION:  return "VISION";
    default:                  return "???";
    }
}

static const char* leader_str(uint8_t s) {
    switch ((leader_state_t)s) {
    case LEADER_OK:       return "OK";
    case LEADER_DEGRADED: return "DEGRADED";
    case LEADER_FAULT:    return "FAULT";
    case LEADER_ESTOP:    return "ESTOP";
    default:              return "???";
    }
}

static const char* reason_str(uint8_t r) {
    switch ((mode_reason_t)r) {
    case REASON_NONE:            return "NONE";
    case REASON_CPU_STALE:       return "CPU_STALE";
    case REASON_LEADER_ESTOP:    return "LEADER_ESTOP";
    case REASON_LEADER_FAULT:    return "LEADER_FAULT";
    case REASON_WP_TIMEOUT:      return "WP_TIMEOUT";
    case REASON_ARUCO_TIMEOUT:   return "ARUCO_TIMEOUT";
    case REASON_WP_RECOVERED:    return "WP_RECOVERED";
    case REASON_ARUCO_RECOVERED: return "ARUCO_RECOVERED";
    case REASON_BOTH_LOST:       return "BOTH_LOST";
    case REASON_INIT:            return "INIT";
    case REASON_TRANSMITION:     return "TRANSMITION";
    default:                     return "???";
    }
}

#define ASSERT_EQ_U8(got, exp, msg) do { \
    if ((uint8_t)(got) != (uint8_t)(exp)) { \
        fprintf(stderr, "[ASSERT FAIL] %s | got=%u exp=%u\n", (msg), (unsigned)(got), (unsigned)(exp)); \
        exit(1); \
    } \
} while(0)

static void print_if_changed(int step, uint8_t prev_mode, const to_vcp_msg_t* m) {
    if (prev_mode != m->mode || m->reason != REASON_NONE) {
        printf("[t=%03d] leader=%-7s wp(%u,age=%u) ar(%u,age=%u)  => mode=%-8s reason=%s\n",
            step,
            leader_str(m->leader_state),
            (unsigned)m->wp_valid, (unsigned)m->wp_age_ms,
            (unsigned)m->aruco_valid, (unsigned)m->aruco_age_ms,
            mode_str(m->mode),
            reason_str(m->reason)
        );
    }
}

// 편의: fresh/timeout 만드는 헬퍼
static void set_wp(to_vcp_msg_t* m, int fresh) {
    m->wp_valid = fresh ? 1 : 0;
    m->wp_age_ms = fresh ? 0 : 999; // timeout 확실히 넘기기
    m->leader_x_mm = 1000;
    m->leader_y_mm = 2000;
}

static void set_aruco(to_vcp_msg_t* m, int fresh) {
    m->aruco_valid = fresh ? 1 : 0;
    m->aruco_age_ms = fresh ? 0 : 999;
    m->aruco_dist_mm = 800;
    m->aruco_x_norm_q15 = 0;
}

int main(void) {
    to_vcp_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.seq = 0;
    msg.cpu_time_ms = 0;
    msg.mode = MODE_STOP_AND_HOLD;      // 시작값
    msg.leader_state = LEADER_OK;

    // ====== 시나리오 ======
    // 1) 초기 1스텝: INIT 찍히고 STOP 유지
    // 2) waypoint fresh 5회 -> WAYPOINT 진입
    // 3) waypoint 끊김 3회 + aruco는 fresh 유지 -> VISION 폴백
    // 4) 둘 다 끊김 3회쯤 -> STOP
    // 5) aruco만 3회 fresh -> VISION 복귀
    // 6) waypoint 다시 5회 fresh -> WAYPOINT로 우선 복귀
    // 7) leader FAULT -> STOP 강제
    // 8) leader ESTOP 1회 -> ESTOP 래치
    // 9) leader OK로 돌아와도 ESTOP 유지 (래치 확인)

    int step = 0;

    // ---- Phase A: init ----
    {
        uint8_t prev = msg.mode;
        decide_mode_step(&msg);
        print_if_changed(step, prev, &msg);

        // init이면 STOP이거나 (너 코드상 STOP로 세팅)
        ASSERT_EQ_U8(msg.mode, MODE_STOP_AND_HOLD, "init mode should be STOP");
        step++;
    }

    // ---- Phase B: wp good 5회 -> WAYPOINT ----
    set_aruco(&msg, 0);
    for (int i = 0; i < 6; i++) { // 5회 이상 보장
        uint8_t prev = msg.mode;
        set_wp(&msg, 1);
        msg.leader_state = LEADER_OK;
        decide_mode_step(&msg);
        print_if_changed(step, prev, &msg);
        step++;
    }
    ASSERT_EQ_U8(msg.mode, MODE_FOLLOW_WAYPOINT, "should enter WAYPOINT after wp_good");

    // ---- Phase C: wp lost 3회 + aruco fresh 3회쯤 -> VISION 폴백 ----
    for (int i = 0; i < 5; i++) {
        uint8_t prev = msg.mode;
        set_wp(&msg, 0);
        set_aruco(&msg, 1);
        decide_mode_step(&msg);
        print_if_changed(step, prev, &msg);
        step++;
    }
    ASSERT_EQ_U8(msg.mode, MODE_FOLLOW_VISION, "should fallback to VISION when wp_bad and ar_good");

    // ---- Phase D: both lost -> STOP ----
    for (int i = 0; i < 5; i++) {
        uint8_t prev = msg.mode;
        set_wp(&msg, 0);
        set_aruco(&msg, 0);
        decide_mode_step(&msg);
        print_if_changed(step, prev, &msg);
        step++;
    }
    ASSERT_EQ_U8(msg.mode, MODE_STOP_AND_HOLD, "both lost -> STOP");

    // ---- Phase E: aruco only recovered 3회 -> VISION ----
    for (int i = 0; i < 4; i++) {
        uint8_t prev = msg.mode;
        set_wp(&msg, 0);
        set_aruco(&msg, 1);
        decide_mode_step(&msg);
        print_if_changed(step, prev, &msg);
        step++;
    }
    ASSERT_EQ_U8(msg.mode, MODE_FOLLOW_VISION, "aruco recovered -> VISION");

    // ---- Phase F: wp recovered 5회 -> WAYPOINT 우선 복귀 ----
    for (int i = 0; i < 6; i++) {
        uint8_t prev = msg.mode;
        set_wp(&msg, 1);
        set_aruco(&msg, 1);
        decide_mode_step(&msg);
        print_if_changed(step, prev, &msg);
        step++;
    }
    ASSERT_EQ_U8(msg.mode, MODE_FOLLOW_WAYPOINT, "wp recovered should switch to WAYPOINT");

    // ---- Phase G: leader FAULT -> STOP 강제 ----
    {
        uint8_t prev = msg.mode;
        msg.leader_state = LEADER_FAULT;
        decide_mode_step(&msg);
        print_if_changed(step, prev, &msg);
        step++;

        ASSERT_EQ_U8(msg.mode, MODE_STOP_AND_HOLD, "leader fault should force STOP");
    }

    // ---- Phase H: leader ESTOP -> ESTOP 래치 ----
    {
        uint8_t prev = msg.mode;
        msg.leader_state = LEADER_ESTOP;
        decide_mode_step(&msg);
        print_if_changed(step, prev, &msg);
        step++;

        ASSERT_EQ_U8(msg.mode, MODE_ESTOP, "leader estop should force ESTOP");
    }

    // ---- Phase I: leader OK로 복귀해도 ESTOP 유지(래치) ----
    {
        uint8_t prev = msg.mode;
        msg.leader_state = LEADER_OK;
        // 신호는 살아있다고 쳐도
        set_wp(&msg, 1);
        set_aruco(&msg, 1);
        decide_mode_step(&msg);
        print_if_changed(step, prev, &msg);
        step++;

        ASSERT_EQ_U8(msg.mode, MODE_ESTOP, "estop latched should persist");
    }

    printf("\nALL TESTS PASSED.\n");
    return 0;
}
