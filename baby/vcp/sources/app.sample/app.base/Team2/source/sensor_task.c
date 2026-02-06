#include "sensor_task.h"
#include "imu.h"
#include "encoder.h"
#include "pose.h"
#include <app_cfg.h>
#include <debug.h>
#include <FreeRTOS.h>
#include <task.h>

#define SENSOR_TASK_PERIOD_MS     20
#define ENC_CALC_PERIOD_MS        100

static Pose s_sensor_pose;

static inline uint32 get_tick_ms(void)
{
    uint32 tick = 0;
    (void)SAL_GetTickCount(&tick);
    return tick * portTICK_PERIOD_MS;
}

static void SensorTask(void *pArg)
{
    (void)pArg;

    IMU_ModuleInit();
    Encoder_Init();
    Pose_Init(0.0f, 0.0f, 0.0f);

    uint32 lastEncCalcMs = get_tick_ms();

    while (1)
    {
        IMU_ModuleUpdate();

        uint32 nowMs = get_tick_ms();
        if ((nowMs - lastEncCalcMs) >= ENC_CALC_PERIOD_MS)
        {
            Encoder_CalcSpeed();
            lastEncCalcMs = nowMs;
        }

        Pose_Update();
        (void)Pose_Get(&s_sensor_pose);

        SAL_TaskSleep(SENSOR_TASK_PERIOD_MS);
    }
}

SALRetCode_t SensorTaskCreate(void)
{
    static uint32 sensorTaskID;
    static uint32 sensorTaskStk[ACFG_TASK_NORMAL_STK_SIZE];

    return SAL_TaskCreate(&sensorTaskID,
                          (const uint8 *)"Sensor Task",
                          SensorTask,
                          sensorTaskStk,
                          ACFG_TASK_NORMAL_STK_SIZE,
                          SAL_PRIO_APP_CFG,
                          NULL);
}

int SensorTask_GetPose(Pose *out)
{
    if (!out)
        return 1;

    *out = s_sensor_pose;
    return 0;
}
