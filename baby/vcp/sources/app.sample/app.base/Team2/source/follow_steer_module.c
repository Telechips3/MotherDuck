// follow_steer_module.c

#include "../team2_header.h"
#include "Pure_Pursuit.h"
#include "follow_steer_module.h"
#include "pose.h"
#include "common.h"
#include "Vision_steer.h"

#include <math.h>
#include <stdint.h>
#include <debug.h>
#include <app_cfg.h>

#define RAD2DEG(x) ((x) * 180.0f / 3.1415926535f)

#define NUM_WAYPOINTS            (10)
#define TEST                      0

// IMU 데이터 저장용 전역 변수
//static IMU_Data_t imu_data;

// Pure_Pursuit structures
static Pose pose;
static PP_Waypoint wps[NUM_WAYPOINTS];
static PP_Handle pp_handler;


static uint32 FollowsteerTaskID;
static uint32 FollowsteerTaskStk[FOLLOW_STEER_TASK_STACK_SIZE];



#if (TEST == 1)
static float dx_base = 0.12f; // 10개면 마지막이 1.2m;   // 시작 간격(5cm)
static float dx_pose_base = 0.00f;
static float dx_step = 0.02f;   // 루프마다 간격 변화(2cm)
static float dx_min  = 0.03f;   // 3cm
static float dx_max  = 0.40f;   // 40cm
static rx_msg_t lc_rx_t = {0};
#endif
// 정보 받아올 구조체 포인터 및 핸들러 선언
static float steer_angle_rad;

int update_follower_steer(to_vcp_msg_t* msg){
    pp_init(&pp_handler, NULL);
    Pose_Init(0.0f, 0.0f, 0.0f);  // ★ pose 모듈 초기화
    mcu_printf("[follow_steer] update init\n");
    uint8_t ret;    
    Pose_Update();
    
    // pose에 update된 pose 받아옴
    ret = Pose_Get(&pose);
    if(ret != 0){
        mcu_printf("[follow_steer] pose value get error = %d\n", ret);
        return 1;
    }

    mcu_printf("[follow_steer] calculated pose (%d, %d, %d)\n ",(int)(1000*pose.x), (int)(1000*pose.y), (int)(1000*pose.yaw));

    if(msg->mode == MODE_FOLLOW_WAYPOINT){
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
            return 2;
        }
    }   
    
    else if(msg->mode == MODE_FOLLOW_VISION){
        ret = steer_from_aruco_q15(msg->aruco_x_norm_q15, msg->aruco_dist_mm, &steer_angle_rad);

    }


    mcu_printf("[follow_steer] yaw=%d steer=%d\n",
               (int)(pose.yaw * 1000), (int)((RAD2DEG(steer_angle_rad))*1000));
    mcu_printf("[follow_steer] selected waypoint (%d, %d)", 
        (int)(pp_handler.last_target_x*1000),(int)(pp_handler.last_target_y*1000) );
    mcu_printf("[floow_steer] last_target_idx = %d , last_target_x_v = %d, last_target_y_v = %d, last_ld2 = %d\n",
                pp_handler.last_target_idx, (int)(pp_handler.last_target_x_d*1000), (int)(pp_handler.last_target_y_d*1000), (int)(pp_handler.last_ld2*1000));
    
    return 0;
}

static void follow_steer_Task(void* pArg)
{
    (void)pArg;

    rx_msg_t lc_rx = {0};
    pp_init(&pp_handler, NULL);

    Pose_Init(0.0f, 0.0f, 0.0f);  // ★ pose 모듈 초기화
    
    while(1)
    {
        // mode 읽고 추종에 해당하는 mode 아니면 conitnue로 넘김
        // if(mode == MODE_ESTOP = 0, MODE_STOP_AND_HOLD = 1) continue; SAL_TaskSleep(SAL_TASK_EXCEPTION_SLEEP);
        
        uint8_t ret;


        #if (TEST == 1) //modechange test
        ++lc_rx.mode;
        if(lc_rx.mode > 3){lc_rx.mode = 0;}
        mcu_printf("[follow_steer] MODE : %d", lc_rx.mode);
        #endif
        // 2) pose 업데이트 (IMU+Encoder는 pose 모듈이 알아서 함)
        Pose_Update();

        Pose p;
        ret = (uint8_t)Pose_Get(&p);
        if(ret != 0){
            mcu_printf("[follow_steer] Get_pose error: %d\n", ret);
            SAL_TaskSleep(FOLLOW_STEER_TASK_SLEEP_MS);
            continue;
        }
        // 현재 pose(PP_pose type)과 p(pose type) 은 동일 포멧 구조체인데 불필요하게 한번 더 복사해서 사용하고 있음
        // Pose type으로 p라는 구조체에 받아와서 그대로 사용하는 구조로 수정 필요함

        pose.x   = p.x;
        pose.y   = p.y;
        pose.yaw = p.yaw;

        #if (TEST == 1) // test
        pose.x = dx_pose_base;
        dx_pose_base += (dx_step + 0.005);
        #endif

        mcu_printf("[follow_steer] calculated pose (%d, %d, %d)\n ",(int)(1000*pose.x), (int)(1000*pose.y), (int)(1000*pose.yaw));
        
       
        // TODO: wps 채우기 (없으면 PP 불가)
        // wps는 패킷 넘겼을 때 이미 채워져 있고, 여기서는 wps큐를 넘기기만 하면 됨
        // for(int i=0;i<NUM_WAYPOINTS;i++){ ... }
        
        if(lc_rx.mode == MODE_FOLLOW_WAYPOINT){
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
        }   

        else if(lc_rx.mode == MODE_FOLLOW_VISION){
            ret = steer_from_aruco_q15(lc_rx.aruco_x_norm_q15, lc_rx.aruco_dist_mm, &steer_angle_rad);

        }

    
        mcu_printf("[follow_steer] yaw=%d steer=%d\n",
                   (int)(pose.yaw * 1000), (int)((RAD2DEG(steer_angle_rad))*1000));
        mcu_printf("[follow_steer] selected waypoint (%d, %d)", 
            (int)(pp_handler.last_target_x*1000),(int)(pp_handler.last_target_y*1000) );
        mcu_printf("[floow_steer] last_target_idx = %d , last_target_x_v = %d, last_target_y_v = %d, last_ld2 = %d\n",
                    pp_handler.last_target_idx, (int)(pp_handler.last_target_x_d*1000), (int)(pp_handler.last_target_y_d*1000), (int)(pp_handler.last_ld2*1000));

        SAL_TaskSleep(FOLLOW_STEER_TASK_SLEEP_MS);
    }
}
SALRetCode_t follow_steer_TaskCreate(void)
{
    SALRetCode_t err = SAL_TaskCreate(&FollowsteerTaskID,
                                      (const uint8 *)"follow Task",
                                      follow_steer_Task,
                                      FollowsteerTaskStk,
                                      FOLLOW_STEER_TASK_STACK_SIZE,
                                      FOLLOW_STEER_TASK_PRIORITY,
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


