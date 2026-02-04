#include "imu.h"
#include "update_pose_test.h"
#include "Pure_Pursuit.h"

#include <math.h>
#include <stdint.h>
#include <debug.h>
#include <app_cfg.h>

static PP_Pose test_pose = {
    0.0f,
    0.0f,
    0.0f
};

static void update_pose_test(){
    test_pose.x += 0.1f; // 0.1m 전진
    test_pose.y += 0.1f; 
    test_pose.yaw += 0.01f; // 0.01 rad 회전
}

int update_pose_test_Get(PP_pose* out_pose)
{
    if (!out_pose)
        return 1;

    *out_pose = test_pose;

    return 0;
}