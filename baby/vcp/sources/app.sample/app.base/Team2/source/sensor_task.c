#include "sensor_task.h"
#include "imu.h"
#include "encoder.h"
#include "pose.h"
#include "ultrasonic.h"
#include "../team2_header.h"
#include <app_cfg.h>
#include <debug.h>
#include <FreeRTOS.h>
#include <task.h>
#include "sal_internal.h"
#include "sal_com.h"

#define PI_F (3.14159265358979323846f)

#define DEG2RAD(deg) ((deg) * (PI_F / 180.0f))
#define RAD2DEG(rad) ((rad) * (180.0f / PI_F))

#define ENC_CALC_PERIOD_MS        10

uint32 sem_ultra = 0; // 여기서 진짜 변수 생성! (static 아님)

static Pose s_sensor_pose;

void sem_ultra_Init(void){
    if (SAL_SemaphoreCreate(&sem_ultra, (const uint8 *)"MY_LOCK", 1UL, SAL_OPT_BLOCKING) != SAL_RET_SUCCESS) {
        mcu_printf("[SEMA ERROR]\n");
    }
}

static inline uint32 get_tick_ms(void)
{
    uint32 tick = 0;
    (void)SAL_GetTickCount(&tick);
    return tick * portTICK_PERIOD_MS;
}

static void SensorTask(void *pArg)
{
    (void)pArg;
    mcu_printf("[SensorTask] task entered at %d ms\n", get_tick_ms()); 
    
    Encoder_Init();
    IMU_ModuleInit();
    Pose_Init(0.0f, 0.0f, 0.0f);
    //Ultrasonic_Init();
    uint32 lastEncCalcMs = get_tick_ms();

    while (1)
    {
        mcu_printf("[SensorTask] loop entered\n");
        uint8_t ret = 0;
        // IMU_Data_t imu;
        // if (IMU_GetData(&imu) == 0) {
        //     mcu_printf("[IMU] yaw=%d deg\n", (int)(100*imu.yaw));
        // }
        // uint32_t emer_dist = Ultrasonic_GetDistance_cm();
        // mcu_printf("[SensorTask] TEST SONIC Dist : %d\n", emer_dist);
        // if(ULTRA_ECHO_TIMEOUT == emer_dist)
        // {
        //     mcu_printf("[SensorTask] SONIC timeout\n");
        // }
        // else{
        //     if(emer_dist <= 5){
        //         sem_ultra_Init();
        //         mcu_printf("[SensorTask] Emergency Dist : %d\n", emer_dist);
        //         continue;
        //         SAL_TaskSleep(SENSOR_TASK_PERIOD_MS);
        //     }
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

       
        

        // mcu_printf("[sensorTask] POSE(mm,deg_x100)=(%d,%d,%d) ENC(cnt,dist_mm,spd_mmps)=(%d,%d,%d)\n",
        //            (int)(1000.0f * s_sensor_pose.x),
        //            (int)(1000.0f * s_sensor_pose.y),
        //            (int)(s_sensor_pose.yaw * (18000.0f / 3.1415926535f)),
        //            (int)Encoder_GetCount(),
        //            (int)(Encoder_GetDistanceCm() * 10.0f),
        //            (int)(Encoder_GetSpeedCms() * 10.0f));
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
