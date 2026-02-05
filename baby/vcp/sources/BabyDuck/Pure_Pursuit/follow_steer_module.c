// follow_steer_module.c

#include "Pure_Pursuit.h"
#include "follow_steer_module.h"
#include "imu.h"

#include <math.h>
#include <stdint.h>
#include <debug.h>
#include <app_cfg.h>

#define DEG2RAD(x) ((x) * 3.1415926535f / 180.0f)

#define NUM_WAYPOINTS       (10)
#define SAL_TASK_SLEEP_MS   (50) 

// IMU 데이터 저장용 전역 변수
static IMU_Data_t imu_data;

// Pure_Pursuit structures
static PP_Pose pose;
static PP_Waypoint wps[NUM_WAYPOINTS];
static PP_Config cfg;
static PP_Handle pp_handler;

// 정보 받아올 구조체 포인터 및 핸들러 선언
static float steer_angle_rad;



static SALRetCode_t follow_steer_Task(void* pArg)
{   
    (void)pArg;
    // cfg를 제대로 세팅하거나, 그냥 기본값 쓰기
    pp_init(&pp_handler, NULL);  // <-- 이게 제일 안전
    // pose 초기화
    pose.x = 0.0f;
    pose.y = 0.0f;
    pose.yaw = 0.0f;

    while(1)
    {
        uint8_t ret;

        // 1) IMU
        ret = (uint8_t)IMU_GetData(&imu_data);
        if(ret != 0){
            mcu_printf("[follow_steer] IMU_GetData error: %u\n", ret);
        }

        // yaw 단위 확인 (deg라면 변환)
        // pose.yaw = imu_data.yaw * (M_PI / 180.0f);
        pose.yaw = DEG2RAD(imu_data.yaw); // <-- imu_data.yaw가 deg일 때만 OK

        // 2) pose 업데이트 (이 함수가 pose.x/pose.y를 갱신하도록 만들거나 직접 반영)
        ret = (uint8_t)update_current_pose(&pose);
        if(ret != 0){
            mcu_printf("[follow_steer] update_current_pose error: %u\n", ret);
        }

        // TODO: 여기서 pose.x, pose.y가 실제로 업데이트 됐는지 보장해야 함
        // TODO: wps[] 채우기 (안 채우면 PP가 정상 동작 불가)

        // 3) PP
        ret = (uint8_t)pp_compute_steer(&pp_handler, &pose, wps, NUM_WAYPOINTS, &steer_angle_rad);
        if(ret != 0){
            mcu_printf("[follow_steer] pp_compute_steer error: %u\n", ret);
        }

        // float 출력 (float printf 안되면 스케일링해서 int로 찍어)
        mcu_printf("[follow_steer] yaw=%d steer=%d\n",
                   (int)(pose.yaw * 1000), (int)(steer_angle_rad * 1000));

        SAL_TaskSleep(SAL_TASK_SLEEP_MS);
    }

    // 도달 안 하지만 형식상
    return SAL_RET_SUCCESS;
}

SALRetCode_t follow_steer_TaskCreate(void)
{
    static uint32 FollowsteerTaskID;
    static uint32 FollowsteerTaskStk[ACFG_TASK_NORMAL_STK_SIZE];

    SALRetCode_t err = SAL_TaskCreate(&FollowsteerTaskID,
                                      (const uint8 *)"follow Task",
                                      follow_steer_Task,
                                      FollowsteerTaskStk,
                                      ACFG_TASK_NORMAL_STK_SIZE,
                                      SAL_PRIO_APP_CFG,
                                      NULL);

    mcu_printf("Follow steer task create: %d\n", (int)err);
    return err;
}

int follow_steer_Get_steer_rad(float *out_steer_rad)
{
    if (!out_steer_rad)
        return 1;

    *out_steer_rad = steer_angle_rad;

    return 0;
}

