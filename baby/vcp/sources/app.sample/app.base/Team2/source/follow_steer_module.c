// follow_steer_module.c

#include "Pure_Pursuit.h"
#include "follow_steer_module.h"
#include "pose.h"

#include <math.h>
#include <stdint.h>
#include <debug.h>
#include <app_cfg.h>

#define RAD2DEG(x) ((x) * 180.0f / 3.1415926535f)

#define NUM_WAYPOINTS            (10)
#define SAL_TASK_SLEEP_MS        (50) 
#define SAL_TASK_EXCEPTION_SLEEP (5)
#define TEST                      0

// IMU 데이터 저장용 전역 변수
//static IMU_Data_t imu_data;

// Pure_Pursuit structures
static PP_Pose pose;
static PP_Waypoint wps[NUM_WAYPOINTS];
static PP_Handle pp_handler;

#if (TEST == 1)
static float dx_base = 0.12f; // 10개면 마지막이 1.2m;   // 시작 간격(5cm)
static float dx_pose_base = 0.00f;
static float dx_step = 0.02f;   // 루프마다 간격 변화(2cm)
static float dx_min  = 0.03f;   // 3cm
static float dx_max  = 0.40f;   // 40cm
#endif
// 정보 받아올 구조체 포인터 및 핸들러 선언
static float steer_angle_rad;

static void follow_steer_Task(void* pArg)
{
    (void)pArg;

    pp_init(&pp_handler, NULL);

    Pose_Init(0.0f, 0.0f, 0.0f);  // ★ pose 모듈 초기화
    
    while(1)
    {
        // mode 읽고 추종에 해당하는 mode 아니면 conitnue로 넘김
        // if(mode == MODE_ESTOP = 0, MODE_STOP_AND_HOLD = 1) continue; SAL_TaskSleep(SAL_TASK_EXCEPTION_SLEEP);

        uint8_t ret;

        // 2) pose 업데이트 (IMU+Encoder는 pose 모듈이 알아서 함)
        Pose_Update();

        Pose p;
        ret = (uint8_t)Pose_Get(&p);
        if(ret != 0){
            mcu_printf("[follow_steer] Get_pose error: %d\n", ret);
        }
        pose.x   = p.x;
        pose.y   = p.y;
        pose.yaw = p.yaw;

        #if (TEST == 1)
        pose.x = dx_pose_base;
        dx_pose_base += (dx_step + 0.005);
        #endif

        mcu_printf("[follow_steer] calculated pose (%d, %d, %d)\n ",(int)(1000*pose.x), (int)(1000*pose.y), (int)(1000*pose.yaw));
        
       
        // TODO: wps 채우기 (없으면 PP 불가)
        // wps는 패킷 넘겼을 때 이미 채워져 있고, 여기서는 wps큐를 넘기기만 하면 됨
        // for(int i=0;i<NUM_WAYPOINTS;i++){ ... }
       
        #if (TEST == 1)
      
        if (dx_base > dx_max || dx_base < dx_min) dx_step = -dx_step; // 왕복 스윕
        for(int i=0;i<NUM_WAYPOINTS;i++){
            float dx = dx_base * (float)(i+1);
            wps[i].x = dx;
            wps[i].y = pose.y + 0.08f; // y는 고정 오프셋(직관적)
            
        }
        #endif
        ret = (uint8_t)pp_compute_steer(&pp_handler, &pose, wps, NUM_WAYPOINTS, &steer_angle_rad);
        if(ret != 0){
            mcu_printf("[follow_steer] pp_compute_steer error: %d\n", ret);
        }

        mcu_printf("[follow_steer] yaw=%d steer=%d\n",
                   (int)(pose.yaw * 1000), (int)((RAD2DEG(steer_angle_rad))*1000));
        mcu_printf("[follow_steer] selected waypoint (%d, %d)", 
            (int)(pp_handler.last_target_x*1000),(int)(pp_handler.last_target_y*1000) );
        mcu_printf("[floow_steer] last_target_idx = %d , last_target_x_v = %d, last_target_y_v = %d, last_ld2 = %d\n",
                    pp_handler.last_target_idx, (int)(pp_handler.last_target_x_d*1000), (int)(pp_handler.last_target_y_d*1000), (int)(pp_handler.last_ld2*1000));

        SAL_TaskSleep(SAL_TASK_SLEEP_MS);
    }
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

//내부 static 변수에 접근해서 값 복사해가는 코드
int follow_steer_Get_steer_rad(float *out_steer_rad)
{
    if (!out_steer_rad)
        return 1;

    *out_steer_rad = steer_angle_rad;

    return 0;
}


