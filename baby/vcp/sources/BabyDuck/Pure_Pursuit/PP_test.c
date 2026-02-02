#include <stdio.h>
#include <math.h>
#include <string.h>
#include "Pure_Pursuit.h"

// 도(degree)를 라디안(radian)으로
#define DEG2RAD(x) ((x) * 3.1415926535f / 180.0f)
// 라디안을 도(degree)로 (결과 확인용)
#define RAD2DEG(x) ((x) * 180.0f / 3.1415926535f)

// 테스트 결과를 이쁘게 출력하는 도우미 함수
void check_result(const char* test_name, float steer_rad, float expected_deg) {
    float steer_deg = RAD2DEG(steer_rad);
    printf("[%s]\n", test_name);
    printf("  -> Calculated: %.2f deg (%.4f rad)\n", steer_deg, steer_rad);
    
    // 예상치랑 비슷한지 확인 (대충 눈대중)
    if (fabs(steer_deg - expected_deg) < 5.0f) { // 오차 5도 이내면 통과로 침
        printf("  -> Status: PASS ✅\n");
    } else {
        printf("  -> Status: CHECK (Expected approx %.2f) ⚠️\n", expected_deg);
    }
    printf("----------------------------------------\n");
}

int main() {
    PP_Handle pp;
    //PP_Config cfg;
    
    // 1. 초기화 테스트
    // 기본값: WB=0.2, LD=0.6, Limit=0.6(약 34도)
    pp_init(&pp, NULL); 
    printf("=== Pure Pursuit Test Start ===\n\n");

    // ==========================================
    // CASE 1: 완전 직진 (차랑 웨이포인트랑 일직선)
    // ==========================================
    {
        PP_Pose pose = {0, 0, 0}; // 원점, 0도(동쪽)
        PP_Waypoint wps[] = {
            {0.5f, 0.0f}, // LD(0.6)보다 가까움 (무시될듯)
            {1.0f, 0.0f}, // LD보다 멂 (타겟)
            {2.0f, 0.0f}
        };
        float steer = 0;
        
        pp_compute_steer(&pp, &pose, wps, 3, &steer);
        check_result("CASE 1: Straight Line", steer, 0.0f);
    }

    // ==========================================
    // CASE 2: 좌회전 (타겟이 왼쪽에 있음)
    // ==========================================
    {
        PP_Pose pose = {0, 0, 0};
        // 차는 (0,0)인데 웨이포인트가 (1, 1)에 있음 -> 왼쪽으로 꺾어야 함
        PP_Waypoint wps[] = {{1.0f, 1.0f}}; 
        float steer = 0;

        // 예상치 계산:
        // x=1, y=1, Ld^2 = 2. WB=0.2
        // steer = atan(2 * 0.2 * 1 / 2) = atan(0.2)
        // atan(0.2)는 약 11.3도
        pp_compute_steer(&pp, &pose, wps, 1, &steer);
        check_result("CASE 2: Left Turn", steer, 11.3f);
    }

    // ==========================================
    // CASE 3: 우회전 (타겟이 오른쪽에 있음)
    // ==========================================
    {
        PP_Pose pose = {0, 0, 0};
        PP_Waypoint wps[] = {{1.0f, -1.0f}}; // y가 음수
        float steer = 0;

        // 대칭이니까 -11.3도 나와야 함
        pp_compute_steer(&pp, &pose, wps, 1, &steer);
        check_result("CASE 3: Right Turn", steer, -11.3f);
    }

    // ==========================================
    // CASE 4: 좌표 변환 테스트 (차가 90도 틀어져 있을 때)
    // ==========================================
    {
        // 차가 (0,0)에서 90도(북쪽, Y축)를 보고 있음
        PP_Pose pose = {0, 0, DEG2RAD(90)}; 
        
        // 웨이포인트가 (-1, 1)에 있음.
        // 북쪽 보는 차 입장에서 (-1, 1)은 "왼쪽 앞"임. -> 좌회전(+) 나와야 함.
        PP_Waypoint wps[] = {{-1.0f, 1.0f}}; 
        float steer = 0;

        pp_compute_steer(&pp, &pose, wps, 1, &steer);
        // 로컬 좌표계로 변환하면 x_v=1, y_v=1이 됨 (CASE 2랑 똑같은 상황)
        check_result("CASE 4: Yaw Rotation (90deg)", steer, 11.3f);
    }

    // ==========================================
    // CASE 5: 후진 경로 (웨이포인트가 다 뒤에 있음)
    // ==========================================
    {
        PP_Pose pose = {0, 0, 0};
        // 웨이포인트가 다 x 음수 (뒤쪽)
        PP_Waypoint wps[] = {{-1.0f, 0.0f}, {-2.0f, 0.5f}};
        float steer = 0;

        // 로직상 다 뒤에 있으면 마지막 점(-2, 0.5)을 타겟으로 잡음
        // 근데 x_v가 음수면 로직에 따라 steering 계산이 좀 튈 수 있음
        // 니 코드 로직: x_v < 0 이어도 강제로 계산함.
        int ret = pp_compute_steer(&pp, &pose, wps, 2, &steer);
        
        if (ret == 0) {
            printf("[CASE 5: All Behind]\n  -> Computed steer: %.2f deg (Fallback Logic Works)\n", RAD2DEG(steer));
            printf("  -> Status: PASS ✅\n");
        } else {
            printf("[CASE 5: All Behind]\n  -> Error Code: %d\n", ret);
        }
        printf("----------------------------------------\n");
    }

    return 0;
}