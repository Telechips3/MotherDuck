#include "speed.h"
#include "steer.h"
#include <math.h>
#include "../team2_header.h"
#include "parsing.h"

// 제어 명령 처리 함수
void parse_and_excute_control(to_vcp_spi_msg_t* pkt)
{
    to_vcp_msg_t* msg = &(pkt->vcp_msg);

    mcu_printf("\n[SPI RX] ════════════════════════════════════════\n");
    mcu_printf("  Seq: %u | Time: %u ms\n", 
               msg->seq, msg->cpu_time_ms);
    mcu_printf("  Mode: %u ", msg->mode);

    switch(msg->mode)
    {
        case MODE_ESTOP:
            mcu_printf("   [PARSING] MODE_ESTOP - Emergency Stop!\n");
            control_motor_drive(0); // 비상정지
            break;
            
        case MODE_STOP_AND_HOLD:
            mcu_printf("   [PARSING] MODE_STOP_AND_HOLD - Holding Position\n");
            control_motor_drive(0); // 정지 및 유지
            break;
            
        case MODE_FOLLOW_WAYPOINT:
            // 웨이포인트 추종 로직
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

                mcu_printf("   [PARSING] MODE_FOLLOW_WAYPOINT\n");
                mcu_printf("   └─ Leader Pos: X=%d.%dcm, Y=%d.%dcm | Angle=%d°\n", 
                           tx_int/10, tx_int%10,      // 800 → 80.0
                           ty_int/10, ty_int%10,      // 424 → 42.4
                           angle_int);
                
                process_acc_with_encoder(target_distance);  // ← 엔코더 기반 ACC
            }
            else {
                mcu_printf("   [PARSING] WAYPOINT INVALID - STOP!\n");
                control_motor_drive(0xFF);
            }
            break;
            
        case MODE_FOLLOW_VISION:
        {
            // 비전 기반 추종 로직 (aruco_valid/age 사용 안 함)
            float dist_cm = (float)msg->aruco_dist_mm / 10.0f;
            float steer_norm = (float)msg->aruco_x_norm_q15 / 32768.0f;  // -1.0 ~ +1.0
            float steer_rad = steer_norm * 0.5236F; // ±30도
            float steer_deg = steer_rad * 180.0f / 3.14159f;

            // float → int 변환
            int dist_int = (int)(dist_cm * 10);       // 30.5 → 305
            int norm_int = (int)(steer_norm * 100);   // 0.15 → 15
            int steer_int = (int)steer_deg;

            mcu_printf("   [PARSING] MODE_FOLLOW_VISION\n");
            mcu_printf("   └─ ArUco: Dist=%d.%dcm | X_norm=%d.%02d (Steer=%d°)\n",
                       dist_int/10, dist_int%10,        // 305 → 30.5
                       norm_int/100, abs(norm_int%100), // 15 → 0.15
                       steer_int);

            process_acc_with_encoder(dist_cm);  // ← 엔코더 기반 ACC
            control_steering_absolute(msg->aruco_x_norm_q15);
            break;
        }
            
        default:
            mcu_printf("   [PARSING] UNKNOWN MODE (%d) - STOP!\n", msg->mode);
            control_motor_drive(0);
            break;
    }
}
