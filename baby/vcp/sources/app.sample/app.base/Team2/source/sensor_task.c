#include "sensor_task.h"
#include "imu.h"
#include "encoder.h"
#include "pose.h"
#include "../team2_header.h"
#include <app_cfg.h>
#include <debug.h>
#include <FreeRTOS.h>
#include <task.h>


#define PI_F (3.14159265358979323846f)

#define DEG2RAD(deg) ((deg) * (PI_F / 180.0f))
#define RAD2DEG(rad) ((rad) * (180.0f / PI_F))

#define ENC_CALC_PERIOD_MS        500

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
    mcu_printf("[SensorTask] task entered \n");    
    
    Encoder_Init();
    IMU_ModuleInit();
    Pose_Init(0.0f, 0.0f, 0.0f);

    uint32 lastEncCalcMs = get_tick_ms();

    while (1)
    {
        mcu_printf("[SensorTask] loop entered\n");
        uint8_t ret = 0;
        IMU_Data_t imu;
        // if (IMU_GetData(&imu) == 0) {
        //     mcu_printf("[IMU] yaw=%d deg\n", (int)(100*imu.yaw));
        // }
        IMU_ModuleUpdate();
        uint32 nowMs = get_tick_ms();
        if ((nowMs - lastEncCalcMs) >= ENC_CALC_PERIOD_MS)
        {
            Encoder_CalcSpeed();
            lastEncCalcMs = nowMs;
        }

        Pose_Update();
        ret = Pose_Get(&s_sensor_pose);
        if(ret != 0){
            mcu_printf("[SensorTask] Getting Pose Failed \n");
        }
        mcu_printf("[sensorTask] Current POSE : (%d, %d, %d)\n"
            ,(int)(100*s_sensor_pose.x), (int)(100*s_sensor_pose.y),(int)(100*s_sensor_pose.yaw));
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
