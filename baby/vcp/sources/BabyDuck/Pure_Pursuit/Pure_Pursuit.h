// Pure_Pursuit.h
#pragma once
#include <stddef.h>


typedef struct {
    float x;    // meters (global frame)
    float y;    // meters (global frame)
    float yaw;  // radians (global frame, heading)
} PP_Pose;

typedef struct {
    float x;    // meters
    float y;    // meters
} PP_Waypoint;

typedef struct {
    float wheelbase_m;     // L (m)
    float lookahead_m;     // Ld (m)
    float steer_limit_rad; // clamp (rad), e.g. 0.6 rad
    float min_ld_m;        // safety: minimum Ld to avoid blow-up
} PP_Config;

typedef struct {
    PP_Config cfg;

    // last compute debug info
    int   last_target_idx; // waypoint index
    float last_target_x_v; // vehicle-frame (vector value)
    float last_target_y_v; // vehicle-frame (vector value)
    float last_ld2;
} PP_Handle;

// init / config
void  pp_init(PP_Handle* h, const PP_Config* cfg);
void  pp_set_config(PP_Handle* h, const PP_Config* cfg);

// compute steering angle (rad)
// returns 0 on success, nonzero on error (e.g., invalid inputs)
int   pp_compute_steer(
          PP_Handle* h,
          const PP_Pose* pose,
          const PP_Waypoint* wps, size_t n,
          float* out_steer_rad  // output
      );




