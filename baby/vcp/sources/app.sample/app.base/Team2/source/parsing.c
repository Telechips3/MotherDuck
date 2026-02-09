#include "speed.h"
#include "steer.h"
#include <math.h>
#include <stdlib.h>
#include "follow_steer_module.h"
#include "../team2_header.h"
#include "parsing.h"


static float steering_rad;
// 조향 강제 스윕 테스트 (1: 활성, 0: 비활성)
#define STEER_SWEEP_TEST 1
// 제어 명령 처리 함수
// parse에서는 패킷 까서 mode만 확인하고 던져줌. 의사결정 X
// parse에서 해줘야 할 건 exception에 대한 처리 (ESTOP, STOP_AND_HOLD)
void parse_and_excute_control(to_vcp_spi_msg_t* pkt)
{
    to_vcp_msg_t* msg = &(pkt->vcp_msg);
    int ret = 0;

    mcu_printf("\n[SPI RX] ════════════════════════════════════════\n");
    mcu_printf("  Magic : 0x%02X | Seq: %d | Time: %d ms\n", 
               pkt->magic, msg->seq, msg->cpu_time_ms);
    mcu_printf("  Mode: %d \n", msg->mode);
    

    switch(msg->mode)
    {
        case MODE_ESTOP:
            mcu_printf("   [PARSING] MODE_ESTOP - Emergency Stop!\n");
            //아예 exception 처리에서 처리하면 좋을듯
            //정책 -> 긴급정지(pwm 출력 0)

            control_motor_drive(0); // 비상정지
            Control_Steering_Custom(0); // 중앙 정렬
            break;

        case MODE_STOP_AND_HOLD:
            mcu_printf("   [PARSING] MODE_STOP_AND_HOLD - Holding Position\n");
            //아예 exception 처리에서 처리하면 좋을듯
            //정책 -> 천천히 정지
            control_motor_drive(0); // 정지 및 유지
            Control_Steering_Custom(0); // 중앙 정렬
            break;
            
        case MODE_FOLLOW_WAYPOINT:
            // 웨이포인트 추종 로직
            
            /*============= legecy code ===============
            if(msg->wp_valid)
            {
                float tx = (float)msg->leader_x_mm / 10.0f;  // mm → cm
                float ty = (float)msg->leader_y_mm / 10.0f;

                float target_distance = sqrtf(tx * tx + ty * ty);
                float target_angle_rad = atan2f(ty, tx);
                float target_angle_deg = target_angle_rad * 180.0f / 3.14159f;

                // float → int 변환 (소수점 1자리 유지)
                int tx_int = (int)(tx * 10);        // 80.0 → 800
                int ty_int = (int)(ty * 10);        // 42.4 → 424
                int angle_int = (int)target_angle_deg;

                if (target_angle_deg < -30.0f) target_angle_deg = -30.0f;
                if (target_angle_deg > +30.0f) target_angle_deg = +30.0f;
        
                int16_t x_norm = (int16_t)(target_angle_deg * 32768.0f / 30.0f);

                mcu_printf("   [PARSING] MODE_FOLLOW_WAYPOINT\n");
                mcu_printf("   └─ Leader Pos: X=%d.%dcm, Y=%d.%dcm | Angle=%d°\n", 
                           tx_int/10, tx_int%10,      // 800 → 80.0
                           ty_int/10, ty_int%10,      // 424 → 42.4
                           angle_int);
                
                process_acc_with_encoder(target_distance);  // ← 엔코더 기반 ACC
                control_steering_absolute(x_norm); // 조향 제어
            }*/
            
        case MODE_FOLLOW_VISION:
            // 모드가 wp냐 vision이냐는 안중요 -> follow_steer 모듈에서 알아서 모드 읽어서 추종해줌
            mcu_printf("[Parsing] VIsion Mode -> Steering\n");
            ret = update_follower_steer(msg);
            if (ret != 0){
                mcu_printf("[PARSING] update_follower_steer failed\n");
            }
            ret = follow_steer_Get_steer_rad(&steering_rad);
            if (ret != 0){
                mcu_printf("[PARSING] getting steering rad failed\n");
            }
#if (STEER_SWEEP_TEST == 1)
            {
                static int sweep_dir = 1;
                static int sweep_cnt = 0;
                const float sweep_rad = 1.0f; // 약 60도

                steering_rad = (sweep_dir > 0) ? sweep_rad : -sweep_rad;
                if (++sweep_cnt >= 10) { // 10번마다 방향 전환
                    sweep_dir = -sweep_dir;
                    sweep_cnt = 0;
                }
                mcu_printf("[STEER_TEST] steering_rad_mrad=%d\n", (int)(steering_rad * 1000));
            }
#endif
            Control_Steering_Custom(steering_rad);
            // {
            //     float norm = steering_rad / MAX_STEER_RAD;
            //     if (norm > 1.0f) norm = 1.0f;
            //     if (norm < -1.0f) norm = -1.0f;
            //     int16_t x_norm = (int16_t)(norm * 32767.0f);
            //     control_steering_absolute(x_norm);
            // }
            
            /* ============= legacy code ==================*/
            // if(msg->aruco_valid)
            // {
            //     float dist_cm = (float)msg->aruco_dist_mm / 10.0f;
            //     float steer_norm = (float)msg->aruco_x_norm_q15 / 32768.0f;  // -1.0 ~ +1.0
            //     float steer_rad = steer_norm * 0.5236F; // ±30도
            //     float steer_deg = steer_rad * 180.0f / 3.14159f;

            //     // float → int 변환
            //     int dist_int = (int)(dist_cm * 10);       // 30.5 → 305
            //     int norm_int = (int)(steer_norm * 100);   // 0.15 → 15
            //     int steer_int = (int)steer_deg;

            //     mcu_printf("   [PARSING] MODE_FOLLOW_VISION\n");
            //     mcu_printf("   └─ ArUco: Dist=%d.%dcm | X_norm=%d.%02d (Steer=%d°)\n",
            //                dist_int/10, dist_int%10,        // 305 → 30.5
            //                norm_int/100, abs(norm_int%100), // 15 → 0.15
            //                steer_int);
                
            //     process_acc_with_encoder(dist_cm);  // ← 엔코더 기반 ACC
            //     control_steering_absolute(msg->aruco_x_norm_q15); // 조향 제어
            // }
            // else {
            //     mcu_printf("   [PARSING] ARUCO INVALID - STOP!\n");
            //     control_motor_drive(0);
            //     control_steering_absolute(0); // 중앙 정렬
            // }
            break;
            
        default:
            mcu_printf("[PARSING] UNKNOWN MODE (%d) - STOP!\n", msg->mode);
            control_motor_drive(0);
            Control_Steering_Custom(0); // 중앙 정렬
            break;
    }
}
