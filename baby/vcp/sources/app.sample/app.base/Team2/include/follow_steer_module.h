// follow_steer_module.h

#pragma once
#include <sal_api.h>
#include "../team2_header.h"

#define STREER_LIMIT   (0.6f)

SALRetCode_t follow_steer_TaskCreate(void);
int update_follower_steer(to_vcp_msg_t* msg);
int follow_steer_Get_steer_rad(float *out_steer_rad);