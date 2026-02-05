// pose_task.c
#include "pose.h"
#include "pose_task.h"
#include <debug.h>

static void PoseTask(void *pArg)
{
    (void)pArg;

    Pose p;

    while (1)
    {
        Pose_Update();
        Pose_Get(&p);

        // mcu_printf doesn't support float format on this platform
        int x_cm = (int)(p.x * 100.0f);
        int y_cm = (int)(p.y * 100.0f);
        int yaw_cdeg = (int)(p.yaw * (18000.0f / 3.1415926535f)); // centi-deg
        int yaw_deg = yaw_cdeg / 100;
        int yaw_frac = yaw_cdeg < 0 ? (-yaw_cdeg % 100) : (yaw_cdeg % 100);

        // mcu_printf("[POSE] x=%dcm y=%dcm yaw=%d.%02d deg\n",
        //            x_cm, y_cm, yaw_deg, yaw_frac);

        SAL_TaskSleep(500);
    }
}

SALRetCode_t PoseTaskCreate(void)
{
    static uint32 poseTaskID;
    static uint32 poseTaskStk[ACFG_TASK_NORMAL_STK_SIZE];

    return SAL_TaskCreate(&poseTaskID,
                          (const uint8 *)"Pose Task",
                          PoseTask,
                          poseTaskStk,
                          ACFG_TASK_NORMAL_STK_SIZE,
                          SAL_PRIO_APP_CFG,
                          NULL);
}
