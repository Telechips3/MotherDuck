// pp.c
#include "pp.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define WHEELBASE      (0.2f)
#define LOOKAHEAD      (0.6f)
#define STREER_LIMIT   (0.6f)
#define MIN_LD_M       (0.2f)

static float clampf(float v, float lo, float hi){
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

static int isfinitef_safe(float v){
    return isfinite(v) ? 1 : 0;
}

void pp_init(PP_Handle* h, const PP_Config* cfg){
    if(!h) return;
    PP_Config d = {
        .wheelbase_m     = WHEELBASE,
        .lookahead_m     = LOOKAHEAD,
        .steer_limit_rad = STREER_LIMIT,
        .min_ld_m        = MIN_LD_M
    };
    h->cfg = cfg ? *cfg : d;

    h->last_target_idx = -1;    
    h->last_target_x_v = 0.0f;
    h->last_target_y_v = 0.0f;
    h->last_ld2        = 0.0f;
}

void pp_set_config(PP_Handle* h, const PP_Config* cfg){
    if(!h || !cfg) return;
    h->cfg = *cfg;
}

int pp_compute_steer(
    PP_Handle* h,
    const PP_Pose* pose,
    const PP_Waypoint* wps, size_t n,
    float* out_steer_rad
){
    if(!h || !pose || !wps || n == 0 || !out_steer_rad) return -1;

    // basic sanity
    if(!isfinitef_safe(pose->x) || !isfinitef_safe(pose->y) || !isfinitef_safe(pose->yaw)) return -2;
    if(h->cfg.wheelbase_m <= 0.0f) return -3;

    float Ld = h->cfg.lookahead_m;
    if(Ld < h->cfg.min_ld_m) Ld = h->cfg.min_ld_m;

    const float cy = cosf(pose->yaw);
    const float sy = sinf(pose->yaw);

    // 1) choose target waypoint:
    //    - candidate must be in front of vehicle (x_v > 0)
    //    - pick first with distance >= Ld
    //    - if none, pick farthest among front candidates
    int best_LD = -1;

    for(size_t i = 0; i < n; i++){
        // 벡터 거리 계산
        const float dx = wps[i].x - pose->x;
        const float dy = wps[i].y - pose->y;

        // global -> vehicle frame
        const float x_v =  cy*dx + sy*dy;       
        const float y_v = -sy*dx + cy*dy;
        
        // 전방에 있는 점인지 확인 (아니면 탈락)
        if(x_v <= 0.0f) continue;               

        // 실제 거리 구하고 Ld 이상인지 확인
        const float d = sqrtf(x_v*x_v + y_v*y_v);
        // LD 이상인 점을 처음 발견하면 그 점을 LD로 잡고 루프 탈출
        // Waypoint가 가까운 순서대로 정렬되어 있다고 가정
        if(d >= Ld){
            best_LD = (int)i;
            break;
        }
    }

    if(best_LD < 0){
        float best_LD_d2 = -1.0f;
        for(size_t i = 0; i < n; i++){
            const float dx = wps[i].x - pose->x;
            const float dy = wps[i].y - pose->y;

            const float x_v =  cy*dx + sy*dy;
            const float y_v = -sy*dx + cy*dy;

            if(x_v <= 0.0f) continue; // 현재보다 뒤에 있는 점은 탈락

            const float d2 = x_v*x_v + y_v*y_v;
            if(d2 > best_LD_d2){
                best_LD_d2 = d2;
                best_LD = (int)i;
            }
        }
        if(best_LD < 0){
            // all points are behind -> fallback: last point
            best_LD = (int)(n - 1);
        }
    }

    // 2) compute target in vehicle frame
    const float dx = wps[best_LD].x - pose->x;
    const float dy = wps[best_LD].y - pose->y;

    const float x_t =  cy*dx + sy*dy;
    const float y_t = -sy*dx + cy*dy;

    const float ld2 = x_t*x_t + y_t*y_t;
    if(ld2 < 1e-6f){
        *out_steer_rad = 0.0f;
        return -4;
    }

    // 3) pure pursuit steering
    // delta = atan2(2*L*y_t, ld2)
    float delta = atan2f(2.0f * h->cfg.wheelbase_m * y_t, ld2);

    // 4) clamp
    delta = clampf(delta, -h->cfg.steer_limit_rad, h->cfg.steer_limit_rad);

    // update debug
    h->last_target_idx = best_LD;
    h->last_target_x_v = x_t;
    h->last_target_y_v = y_t;
    h->last_ld2        = ld2;

    *out_steer_rad = delta;             //넘겨줄 steering 각도
    return 0;
}
