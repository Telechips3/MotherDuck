#include <stdio.h>
#include <math.h>
#include "Pure_Pursuit.h"
#include "pose.h"
#include "pp_pose_test.h"

// 도(degree)를 라디안(radian)으로
#define DEG2RAD(x) ((x) * 3.1415926535f / 180.0f)
// 라디안을 도(degree)로 (결과 확인용)
#define RAD2DEG(x) ((x) * 180.0f / 3.1415926535f)

// =============================
// Pose 테스트용 스텁
// =============================
static float s_stub_yaw_rad = 0.0f;
static float s_stub_delta_m = 0.0f;

float IMU_GetYawRad(void)
{
    return s_stub_yaw_rad;
}

float Encoder_GetDeltaDistanceM(void)
{
    float d = s_stub_delta_m;
    s_stub_delta_m = 0.0f;
    return d;
}

static void check_result(const char* test_name, float steer_rad, float expected_deg)
{
    float steer_deg = RAD2DEG(steer_rad);
    printf("[%s]\n", test_name);
    printf("  -> Calculated: %.2f deg (%.4f rad)\n", steer_deg, steer_rad);
    if (fabs(steer_deg - expected_deg) < 5.0f) {
        printf("  -> Status: PASS\n");
    } else {
        printf("  -> Status: CHECK (Expected approx %.2f)\n", expected_deg);
    }
    printf("----------------------------------------\n");
}

static void pose_basic_test(void)
{
    Pose p;

    printf("=== Pose Test Start ===\n\n");

    // 초기화
    Pose_Init(0.0f, 0.0f, 0.0f);

    // 1) 전진 1m, yaw=0
    s_stub_yaw_rad = 0.0f;
    s_stub_delta_m = 1.0f;
    Pose_Update();
    Pose_Get(&p);
    printf("[POSE 1] x=%.3f y=%.3f yaw=%.2f deg\n", p.x, p.y, RAD2DEG(p.yaw));

    // 2) 90도 회전 후 전진 1m
    s_stub_yaw_rad = DEG2RAD(90.0f);
    s_stub_delta_m = 1.0f;
    Pose_Update();
    Pose_Get(&p);
    printf("[POSE 2] x=%.3f y=%.3f yaw=%.2f deg\n", p.x, p.y, RAD2DEG(p.yaw));

    // 3) -90도 회전 후 전진 2m
    s_stub_yaw_rad = DEG2RAD(-90.0f);
    s_stub_delta_m = 2.0f;
    Pose_Update();
    Pose_Get(&p);
    printf("[POSE 3] x=%.3f y=%.3f yaw=%.2f deg\n", p.x, p.y, RAD2DEG(p.yaw));

    printf("----------------------------------------\n\n");
}

int pp_pose_test_run(void)
{
    PP_Handle pp;

    pp_init(&pp, NULL);

    printf("=== Pure Pursuit Test Start ===\n\n");

    // CASE 1: 직진
    {
        PP_Pose pose = {0, 0, 0};
        PP_Waypoint wps[] = {
            {0.5f, 0.0f},
            {1.0f, 0.0f},
            {2.0f, 0.0f}
        };
        float steer = 0;

        pp_compute_steer(&pp, &pose, wps, 3, &steer);
        check_result("CASE 1: Straight Line", steer, 0.0f);
    }

    // CASE 2: 좌회전
    {
        PP_Pose pose = {0, 0, 0};
        PP_Waypoint wps[] = {{1.0f, 1.0f}};
        float steer = 0;

        pp_compute_steer(&pp, &pose, wps, 1, &steer);
        check_result("CASE 2: Left Turn", steer, 11.3f);
    }

    // CASE 3: 우회전
    {
        PP_Pose pose = {0, 0, 0};
        PP_Waypoint wps[] = {{1.0f, -1.0f}};
        float steer = 0;

        pp_compute_steer(&pp, &pose, wps, 1, &steer);
        check_result("CASE 3: Right Turn", steer, -11.3f);
    }

    // CASE 4: Yaw 회전 테스트
    {
        PP_Pose pose = {0, 0, DEG2RAD(90)};
        PP_Waypoint wps[] = {{-1.0f, 1.0f}};
        float steer = 0;

        pp_compute_steer(&pp, &pose, wps, 1, &steer);
        check_result("CASE 4: Yaw Rotation (90deg)", steer, 11.3f);
    }

    // CASE 5: 뒤쪽 웨이포인트
    {
        PP_Pose pose = {0, 0, 0};
        PP_Waypoint wps[] = {{-1.0f, 0.0f}, {-2.0f, 0.5f}};
        float steer = 0;

        int ret = pp_compute_steer(&pp, &pose, wps, 2, &steer);
        if (ret == 0) {
            printf("[CASE 5: All Behind]\n  -> Computed steer: %.2f deg (Fallback Logic Works)\n", RAD2DEG(steer));
            printf("  -> Status: PASS\n");
        } else {
            printf("[CASE 5: All Behind]\n  -> Error Code: %d\n", ret);
        }
        printf("----------------------------------------\n");
    }

    printf("\n");
    pose_basic_test();

    return 0;
}

int main(void)
{
    return pp_pose_test_run();
}
