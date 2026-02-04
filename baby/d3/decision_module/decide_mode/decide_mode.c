// decide_mode.c
#include <stdint.h>
#include "decide_mode.h"   // 너가 to_vcp_msg_t, enums를 선언한 헤더

#define WP_TIMEOUT_MS_DEFAULT 400
#define ARUCO_TIMEOUT_MS_DEFAULT 200
#define CPU_MSG_STALE_MS_DEFAULT 200
#define WP_OK_TO_USE_CNT_DEFAULT 5
#define WP_LOST_TO_DROP_CNT_DEFAULT 3
#define ARC_OK_TO_USE_CNT_DEFAULT 3
#define ARC_LOST_TO_DROP_CNT_DEFAULT 3
#define LATCH_ESTOP_DEFAULT 1

// ===== parameters (tune these) =====
typedef struct {
    uint16_t wp_timeout_ms;
    uint16_t aruco_timeout_ms;
    uint16_t cpu_msg_stale_ms;     // cpu_time_ms 기반 stale 판단용(옵션)

    uint8_t  wp_ok_to_use_cnt;     // e.g. 5
    uint8_t  wp_lost_to_drop_cnt;  // e.g. 3
    uint8_t  arc_ok_to_use_cnt;     // e.g. 3
    uint8_t  arc_lost_to_drop_cnt;  // e.g. 3

    uint8_t  latch_estop;          // 1이면 ESTOP 래치
} decide_params_t;

// ===== internal state for hysteresis =====
typedef struct {
    uint8_t wp_ok_cnt;
    uint8_t wp_lost_cnt;
    uint8_t arc_ok_cnt;
    uint8_t arc_lost_cnt;

    uint8_t estop_latched;
    uint8_t initialized;
} decide_state_t;

static decide_state_t g_decide_st = { 0 };

// clamp helpers
static inline uint8_t u8_inc_sat(uint8_t v, uint8_t maxv) { return (v < maxv) ? (uint8_t)(v + 1) : maxv; }
static inline uint8_t u8_dec_sat(uint8_t v) { return (v > 0) ? (uint8_t)(v - 1) : 0; }

// signal freshness 판단
static inline uint8_t is_fresh(uint8_t valid, uint16_t age_ms, uint16_t timeout_ms)
{
    return (valid != 0u) && (age_ms <= timeout_ms);
}

// 현재 모드에서 “필수 신호”가 사라졌을 때 다운그레이드 가드 (Vision / Waypoint 방식 결정)
static ctrl_mode_t guard_mode(ctrl_mode_t mode, uint8_t wp_fresh, uint8_t arc_fresh)
{
    if (mode == MODE_FOLLOW_WAYPOINT && !wp_fresh) {
        return arc_fresh ? MODE_FOLLOW_VISION : MODE_STOP_AND_HOLD;
    }
    if (mode == MODE_FOLLOW_VISION && !arc_fresh) {
        return wp_fresh ? MODE_FOLLOW_WAYPOINT : MODE_STOP_AND_HOLD;
    }
    return mode;
}

/**
 * @brief decide_mode() : msg의 mode / reason을 결정한다.
 * @param msg[in,out]  to_vcp_msg_t. 입력(valid/age/leader_state 등)은 채워져 있어야 함.
 * @param now_ms[in]   CPU monotonic ms (현재 시각). msg->cpu_time_ms와 별개로 “지금”이 필요하면 사용.
 */
uint8_t decide_mode_step(to_vcp_msg_t* msg/*, uint32_t now_ms*/)
{
    // ---- 튜닝 파라미터(일단 고정값으로 박아두고 나중에 config로 빼도 됨) ----
    static const decide_params_t P = {
      .wp_timeout_ms = WP_TIMEOUT_MS_DEFAULT,
      .aruco_timeout_ms = ARUCO_TIMEOUT_MS_DEFAULT,
      .cpu_msg_stale_ms = CPU_MSG_STALE_MS_DEFAULT, // (옵션) CPU->MCU 메시지 스트림 stale 판단에 쓰려면 VCP쪽에서 사용

      .wp_ok_to_use_cnt = WP_OK_TO_USE_CNT_DEFAULT,
      .wp_lost_to_drop_cnt = WP_LOST_TO_DROP_CNT_DEFAULT,
      .arc_ok_to_use_cnt = ARC_OK_TO_USE_CNT_DEFAULT,
      .arc_lost_to_drop_cnt = ARC_LOST_TO_DROP_CNT_DEFAULT,

      .latch_estop = LATCH_ESTOP_DEFAULT
    };

    if (msg == 0) return REASON_NULL;

    // ---- init ----
    if (!g_decide_st.initialized) {
        g_decide_st.initialized = 1;
        msg->mode = MODE_STOP_AND_HOLD;
        msg->reason = (uint8_t)REASON_INIT;
    }
    else {
        msg->reason = (uint8_t)REASON_NONE;
    }

    // ---- 1) leader safety 먼저 (최우선) ----
    if (msg->leader_state == LEADER_ESTOP) {
        if (P.latch_estop) g_decide_st.estop_latched = 1;
        msg->mode = MODE_ESTOP;
        msg->reason = (uint8_t)REASON_LEADER_ESTOP;
        return msg->reason;
    }

    if (msg->leader_state == LEADER_FAULT) {
        // fault는 정책에 따라 ESTOP/STOP 선택 가능. 일단 STOP으로.
        msg->mode = MODE_STOP_AND_HOLD;
        msg->reason = (uint8_t)REASON_LEADER_FAULT;
        return msg->reason;
    }

    if (g_decide_st.estop_latched) {
        msg->mode = MODE_ESTOP;
        msg->reason = (uint8_t)REASON_LEADER_ESTOP;
        return msg->reason;
    }

    // ---- 2) freshness 판단 ----
    const uint8_t wp_fresh = is_fresh(msg->wp_valid, msg->wp_age_ms, P.wp_timeout_ms);
    const uint8_t arc_fresh = is_fresh(msg->aruco_valid, msg->aruco_age_ms, P.aruco_timeout_ms);

    // ---- 3) hysteresis 카운터 업데이트 ----
    if (wp_fresh) {
        g_decide_st.wp_ok_cnt = u8_inc_sat(g_decide_st.wp_ok_cnt, P.wp_ok_to_use_cnt);
        g_decide_st.wp_lost_cnt = u8_dec_sat(g_decide_st.wp_lost_cnt);
    }
    else {
        g_decide_st.wp_lost_cnt = u8_inc_sat(g_decide_st.wp_lost_cnt, P.wp_lost_to_drop_cnt);
        g_decide_st.wp_ok_cnt = u8_dec_sat(g_decide_st.wp_ok_cnt);
    }

    if (arc_fresh) {
        g_decide_st.arc_ok_cnt = u8_inc_sat(g_decide_st.arc_ok_cnt, P.arc_ok_to_use_cnt);
        g_decide_st.arc_lost_cnt = u8_dec_sat(g_decide_st.arc_lost_cnt);
    }
    else {
        g_decide_st.arc_lost_cnt = u8_inc_sat(g_decide_st.arc_lost_cnt, P.arc_lost_to_drop_cnt);
        g_decide_st.arc_ok_cnt = u8_dec_sat(g_decide_st.arc_ok_cnt);
    }
    const uint8_t wp_good = (g_decide_st.wp_ok_cnt >= P.wp_ok_to_use_cnt);
    const uint8_t wp_bad = (g_decide_st.wp_lost_cnt >= P.wp_lost_to_drop_cnt);
    const uint8_t arc_good = (g_decide_st.arc_ok_cnt >= P.arc_ok_to_use_cnt);
    const uint8_t arc_bad = (g_decide_st.arc_lost_cnt >= P.arc_lost_to_drop_cnt);

    // ---- 4) 모드 전이 (핵심) ----
    // 여기서 결정지은 mode에 대해서 신뢰성 검사 수행 (정보 Fresh한지 여부 등)
    ctrl_mode_t next = msg->mode;
    ctrl_mode_t prev = msg->mode;

    switch (msg->mode) {
    case MODE_FOLLOW_WAYPOINT:
        if (wp_bad) {
            if (arc_good) {
                next = MODE_FOLLOW_VISION;
                msg->reason = (uint8_t)REASON_WP_TIMEOUT;
            }
            else {
                next = MODE_STOP_AND_HOLD;
                msg->reason = (uint8_t)REASON_BOTH_LOST;
            }
        }
        break;

    case MODE_FOLLOW_VISION:
        if (wp_good) {
            next = MODE_FOLLOW_WAYPOINT;
            msg->reason = (uint8_t)REASON_WP_RECOVERED;
        }
        else if (arc_bad) {
            next = MODE_STOP_AND_HOLD;
            msg->reason = (uint8_t)REASON_ARUCO_TIMEOUT;
        }
        break;

    case MODE_STOP_AND_HOLD:
    default:
        // 복귀 우선순위: waypoint > vision
        if (wp_good) {
            next = MODE_FOLLOW_WAYPOINT;
            msg->reason = (uint8_t)REASON_WP_RECOVERED;
        }
        else if (arc_good) {
            next = MODE_FOLLOW_VISION;
            msg->reason = (uint8_t)REASON_ARUCO_RECOVERED;
        }
        else {
            next = MODE_STOP_AND_HOLD;
            // reason은 필요하면 BOTH_LOST로
            if (wp_bad && arc_bad) msg->reason = (uint8_t)REASON_BOTH_LOST;
        }
        break;

    case MODE_ESTOP:
        // 여기 오면 사실상 래치/리셋 정책에 따라… 지금은 유지.
        next = MODE_ESTOP;
        break;
    }

    // ---- 5) 가드(모드가 신호와 안 맞으면 강등) ----
    next = guard_mode(next, wp_fresh, arc_fresh);

    // 가드 모드에서 강등당했을 시 디버깅 로그 출력
    if (next != prev && msg->reason == REASON_NONE) {
        msg->reason = (uint8_t)REASON_TRANSMITION;
    }
    // 정보 파싱
    msg->mode = next;

    //(void)now_ms; // 지금은 미사용. 필요 시 cpu_time_ms stale 로직 확장 가능

    return msg->reason;
}
