// follow_steer_module.h

#pragma once
#include <sal_api.h>

SALRetCode_t follow_steer_TaskCreate(void);

int follow_steer_Get_steer_rad(float *out_steer_rad);