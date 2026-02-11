#include "../team2_header.h"
#include "../include/ipc.h"
#include "speed.h" // control_motor_drive 함수 호출용
#include "steer.h" // control_steering_step 함수 호출용
#include "parsing.h" // parse_and_excute_control 함수 호출용
/* TEST */
#define IPC_VISION_DUMMY_TEST 1

/* 전역 변수 */
uint32 IPC_queue_id = 0;
static uint32 IPC_task_id = 0;

/* 태스크용 스택 배열 (SAL_TaskCreate는 정적 스택을 요구함) */

static uint32 IPC_task_stack[IPC_TASK_STACK_SIZE];

/* [소비자 태스크] 큐를 감시하다가 데이터가 없으면 멈춤 */
static void IPC_Task(void *pParam)
{
    // uint32 received_cmd = 0xFF;
    // uint32 copied_size = 0;
    // SALRetCode_t ret;

    // while (1)
    // {
    //     /* * SAL_QueueGet 특징: 
    //      * - 4번째 인자(iTimeout): 200ms 대기
    //      * - 데이터가 오면 SAL_RET_SUCCESS
    //      * - 200ms 동안 데이터가 안 오면 SAL_ERR_FAIL_GET_DATA (타임아웃) 반환
    //      */
    //     //ret = SAL_QueueGet(g_motor_queue_id, &received_cmd, &copied_size, 200, SAL_OPT_BLOCKING);

    //     if (ret == SAL_RET_SUCCESS)
    //     {
    //         // 명령 수신 성공: 전진/후진 등 실행
    //         if(received_cmd == 0 || received_cmd == 2)
    //         {
    //             control_motor_drive(received_cmd);
    //         }
    //         else{
    //             control_steering_step(received_cmd);
    //         }
    //     }
    //     else
    //     {
    //         // 타임아웃 발생: 리눅스에서 'w'를 뗐다고 판단
    //         // control_motor_drive 내부의 else(정지) 로직을 타게 함
    //         control_motor_drive(0xFF); 
    //     }
    //     SAL_TaskSleep(1);
    // }
    to_vcp_spi_msg_t received_pkt; // 구조체 패킷 수신
    uint32 copied_size = 0; // 수신 크기
    SALRetCode_t ret;
    mcu_printf("[ipc] task started\n");
    while(1){
#if (IPC_VISION_DUMMY_TEST == 1)
        static int16_t x_norm = 32000; // 좌우 스윙 시작점
        static int16_t dist_mm = 1000;  // 100~1000mm 스윙 시작점
        static int16_t step = 4096;
        static int16_t dist_step = -100;
        to_vcp_spi_msg_t pkt;
        static uint32_t packet_seq = 0;  // static으로 선언해서 값 유지

        pkt.magic = 0xA5;
        pkt.vcp_msg.seq = ++packet_seq; // 1, 2, 3... 계속 증가
        pkt.vcp_msg.cpu_time_ms = 100;
        pkt.vcp_msg.mode = MODE_FOLLOW_VISION;

        pkt.vcp_msg.aruco_valid = 1;
        pkt.vcp_msg.aruco_age_ms = 0;
        pkt.vcp_msg.aruco_dist_mm = dist_mm;
        pkt.vcp_msg.aruco_x_norm_q15 = x_norm;

        pkt.vcp_msg.wp_valid = 0;
        pkt.vcp_msg.wp_age_ms = 0;
        pkt.vcp_msg.leader_x_mm = 0;
        pkt.vcp_msg.leader_y_mm = 0;
        pkt.vcp_msg.reason = 0;

        parse_and_excute_control((void*)&pkt);

        int32_t temp_val = (int32_t)x_norm + step; // 32비트로 계산해서 오버플로우 방지

        if (temp_val >= 32000) {
            x_norm = 32000;      // 끝값 고정
            step = -step;        // 방향 반대로 (감소)
        } 
        else if (temp_val <= -32000) {
            x_norm = -32000;     // 끝값 고정
            step = -step;        // 방향 반대로 (증가)
        } 
        else {
            x_norm = (int16_t)temp_val; // 범위 안이면 대입
        }

        int32_t dist_temp = (int32_t)dist_mm + dist_step;
        if (dist_temp >= 1000) {
            dist_mm = 1000;
            dist_step = -dist_step;
        }
        else if (dist_temp <= 100) {
            dist_mm = 100;
            dist_step = -dist_step;
        }
        else {
            dist_mm = (int16_t)dist_temp;
        }

        SAL_TaskSleep(500);
    #else
        mcu_printf("[ipc] loop entered\n");
        ret = SAL_QueueGet(IPC_queue_id, &received_pkt, &copied_size, 200, SAL_OPT_BLOCKING);
        uint32_t tick = 0;
        SAL_GetTickCount(&tick);
        mcu_printf("[ipc] PRE PARSE TICK: %d", tick);
        if(ret == SAL_RET_SUCCESS)
        {   
            if(received_pkt.magic == 0xA5)
            {
                // 유효한 패킷 수신 시 제어 함수 호출
                mcu_printf("[ipc] PKT RX Successed\n");
                parse_and_excute_control((void*)&received_pkt);
                SAL_GetTickCount(&tick);
                mcu_printf("[ipc] POST PARSE TICK: %d", tick);
            }
        }
            else
            {
            // 타임아웃 발생 시 정지 명령 실행
            mcu_printf("[ipc] unknown magic number\n");
            control_motor_drive(0xFF);
            }
        
        SAL_TaskSleep(IPC_TASK_SLEEP_MS);
        #endif
    }
}

/* 테스트 코드 ==================================================================
typedef struct {
    const char* scenario_name;      // 시나리오 이름
    ctrl_mode_t mode;               // 제어 모드
    uint8_t aruco_valid;            // ArUco 유효성
    int16_t aruco_dist_mm;          // ArUco 거리
    int16_t aruco_x_norm;           // ArUco X 위치
    uint8_t wp_valid;               // Waypoint 유효성
    int32_t leader_x_mm;            // 리더 X 위치
    int32_t leader_y_mm;            // 리더 Y 위치
    uint16_t duration_ms;           // 시나리오 지속 시간
} test_scenario_t;

📋 테스트 시나리오 배열 (실제 주행 상황 시뮬레이션)
static const test_scenario_t test_scenarios[] = {
    // === 시나리오 1: ESTOP (비상정지) ===
    {
        .scenario_name = "EMERGENCY STOP",
        .mode = MODE_ESTOP,
        .aruco_valid = 0,
        .aruco_dist_mm = 0,
        .aruco_x_norm = 0,
        .wp_valid = 0,
        .leader_x_mm = 0,
        .leader_y_mm = 0,
        .duration_ms = 3000  // 3초간 정지
    },
    
    // === 시나리오 2: Vision 모드 - 너무 가까움 (후진 필요) ===
    {
        .scenario_name = "VISION: Too Close (30cm)",
        .mode = MODE_FOLLOW_VISION,
        .aruco_valid = 1,
        .aruco_dist_mm = 300,   // 30cm (목표 50cm보다 가까움)
        .aruco_x_norm = 0,      // 정중앙
        .wp_valid = 0,
        .leader_x_mm = 0,
        .leader_y_mm = 0,
        .duration_ms = 4000
    },
    
    // === 시나리오 3: Vision 모드 - 적정 거리 (유지) ===
    {
        .scenario_name = "VISION: Safe Distance (50cm)",
        .mode = MODE_FOLLOW_VISION,
        .aruco_valid = 1,
        .aruco_dist_mm = 500,   // 50cm (목표 거리)
        .aruco_x_norm = 2000,   // 약간 오른쪽
        .wp_valid = 0,
        .leader_x_mm = 0,
        .leader_y_mm = 0,
        .duration_ms = 3000
    },
    
    // === 시나리오 4: Vision 모드 - 멀리 떨어짐 (전진 필요) ===
    {
        .scenario_name = "VISION: Too Far (100cm)",
        .mode = MODE_FOLLOW_VISION,
        .aruco_valid = 1,
        .aruco_dist_mm = 1000,  // 100cm (목표보다 멀음)
        .aruco_x_norm = -5000,  // 왼쪽으로 치우침
        .wp_valid = 0,
        .leader_x_mm = 0,
        .leader_y_mm = 0,
        .duration_ms = 4000
    },
    
    // === 시나리오 5: Vision 모드 - 매우 멀리 (빠른 전진) ===
    {
        .scenario_name = "VISION: Very Far (150cm)",
        .mode = MODE_FOLLOW_VISION,
        .aruco_valid = 1,
        .aruco_dist_mm = 1500,  // 150cm
        .aruco_x_norm = 0,
        .wp_valid = 0,
        .leader_x_mm = 0,
        .leader_y_mm = 0,
        .duration_ms = 4000
    },
    
    // === 시나리오 6: Waypoint 모드 - 정면 전방 ===
    {
        .scenario_name = "WAYPOINT: Straight Ahead (80cm)",
        .mode = MODE_FOLLOW_WAYPOINT,
        .aruco_valid = 0,
        .aruco_dist_mm = 0,
        .aruco_x_norm = 0,
        .wp_valid = 1,
        .leader_x_mm = 800,     // X: 80cm 전방
        .leader_y_mm = 0,       // Y: 0cm (정중앙)
        .duration_ms = 4000
    },
    
    // === 시나리오 7: Waypoint 모드 - 대각선 위치 ===
    {
        .scenario_name = "WAYPOINT: Diagonal (60cm, 45deg)",
        .mode = MODE_FOLLOW_WAYPOINT,
        .aruco_valid = 0,
        .aruco_dist_mm = 0,
        .aruco_x_norm = 0,
        .wp_valid = 1,
        .leader_x_mm = 424,     // X: 42.4cm (cos45° * 60cm)
        .leader_y_mm = 424,     // Y: 42.4cm (sin45° * 60cm)
        .duration_ms = 4000
    },
    
    // === 시나리오 8: Waypoint 모드 - 매우 가까움 ===
    {
        .scenario_name = "WAYPOINT: Very Close (35cm)",
        .mode = MODE_FOLLOW_WAYPOINT,
        .aruco_valid = 0,
        .aruco_dist_mm = 0,
        .aruco_x_norm = 0,
        .wp_valid = 1,
        .leader_x_mm = 300,     // X: 30cm
        .leader_y_mm = 170,     // Y: 17cm (약간 비스듬히)
        .duration_ms = 4000
    },
    
    // === 시나리오 9: Vision 신호 상실 (STOP) ===
    {
        .scenario_name = "VISION LOST -> STOP",
        .mode = MODE_FOLLOW_VISION,
        .aruco_valid = 0,       // 신호 상실!
        .aruco_dist_mm = 0,
        .aruco_x_norm = 0,
        .wp_valid = 0,
        .leader_x_mm = 0,
        .leader_y_mm = 0,
        .duration_ms = 3000
    },
    
    // === 시나리오 10: Waypoint 신호 상실 (STOP) ===
    {
        .scenario_name = "WAYPOINT LOST -> STOP",
        .mode = MODE_FOLLOW_WAYPOINT,
        .aruco_valid = 0,
        .aruco_dist_mm = 0,
        .aruco_x_norm = 0,
        .wp_valid = 0,          // 신호 상실!
        .leader_x_mm = 0,
        .leader_y_mm = 0,
        .duration_ms = 3000
    },
    
    // === 시나리오 11: 모드 전환 테스트 (Vision → Waypoint) ===
    {
        .scenario_name = "MODE SWITCH: Vision to WP",
        .mode = MODE_FOLLOW_VISION,
        .aruco_valid = 1,
        .aruco_dist_mm = 600,
        .aruco_x_norm = 0,
        .wp_valid = 1,          // 두 신호 모두 있음
        .leader_x_mm = 700,
        .leader_y_mm = 0,
        .duration_ms = 3000
    },
    
    // === 시나리오 12: 초근접 위험 상황 ===
    {
        .scenario_name = "CRITICAL: Ultra Close (5cm)",
        .mode = MODE_FOLLOW_VISION,
        .aruco_valid = 1,
        .aruco_dist_mm = 50,    // 5cm! (비상!)
        .aruco_x_norm = 0,
        .wp_valid = 0,
        .leader_x_mm = 0,
        .leader_y_mm = 0,
        .duration_ms = 5000     // 긴 시간 관찰
    }
};
===========================================================================

#define NUM_SCENARIOS (sizeof(test_scenarios) / sizeof(test_scenario_t))

static void vDummyProducerTask(void *pParam)
{
    to_vcp_spi_msg_t fake_pkt;
    uint32_t scenario_idx = 0;
    uint32_t packet_seq = 0;
    
    mcu_printf("\n");
    mcu_printf("╔════════════════════════════════════════════════════════════╗\n");
    mcu_printf("║     ACC System Comprehensive Test Started                 ║\n");
    mcu_printf("║     Total Scenarios: %d                                   ║\n", NUM_SCENARIOS);
    mcu_printf("╚════════════════════════════════════════════════════════════╝\n");
    mcu_printf("\n");

    while(1)
    {
        const test_scenario_t* scenario = &test_scenarios[scenario_idx];
        
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // 시나리오 시작 알림
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        mcu_printf("\n");
        mcu_printf("┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓\n");
        mcu_printf("┃ [Scenario %d/%d] %s\n", 
                   scenario_idx + 1, NUM_SCENARIOS, scenario->scenario_name);
        mcu_printf("┃ Duration: %d ms\n", scenario->duration_ms);
        mcu_printf("┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");
        
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // 패킷 데이터 구성
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        fake_pkt.magic = 0xA5;
        fake_pkt.vcp_msg.seq = packet_seq++;
        fake_pkt.vcp_msg.cpu_time_ms = packet_seq * 100;  // 간단한 타임스탬프
        fake_pkt.vcp_msg.mode = scenario->mode;
        fake_pkt.vcp_msg.leader_state = 0;  // LEADER_OK
        
        // ArUco 데이터
        fake_pkt.vcp_msg.aruco_valid = scenario->aruco_valid;
        fake_pkt.vcp_msg.aruco_age_ms = 50;  // 신선한 데이터
        fake_pkt.vcp_msg.aruco_dist_mm = scenario->aruco_dist_mm;
        fake_pkt.vcp_msg.aruco_x_norm_q15 = scenario->aruco_x_norm;
        
        // Waypoint 데이터
        fake_pkt.vcp_msg.wp_valid = scenario->wp_valid;
        fake_pkt.vcp_msg.wp_age_ms = 80;
        fake_pkt.vcp_msg.leader_x_mm = scenario->leader_x_mm;
        fake_pkt.vcp_msg.leader_y_mm = scenario->leader_y_mm;
        
        fake_pkt.vcp_msg.reason = 0;
        fake_pkt.crc16 = 0xDEAD;  // 더미 CRC
        
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // 패킷 정보 출력
        // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        mcu_printf("│ Mode: ");
        switch(scenario->mode) {
            case MODE_ESTOP: mcu_printf("ESTOP"); break;
            case MODE_STOP_AND_HOLD: mcu_printf("STOP_AND_HOLD"); break;
            case MODE_FOLLOW_WAYPOINT: mcu_printf("FOLLOW_WAYPOINT"); break;
            case MODE_FOLLOW_VISION: mcu_printf("FOLLOW_VISION"); break;
        }
        mcu_printf("\n");
        
        if (scenario->aruco_valid) {
            mcu_printf("│ ArUco: Valid | Dist=%d mm (%.1f cm) | X_norm=%d\n",
                       scenario->aruco_dist_mm, 
                       scenario->aruco_dist_mm / 10.0f,
                       scenario->aruco_x_norm);
        } else {
            mcu_printf("│ ArUco: INVALID (signal lost)\n");
        }
        
        if (scenario->wp_valid) {
            float wp_x_cm = scenario->leader_x_mm / 10.0f;
            float wp_y_cm = scenario->leader_y_mm / 10.0f;
            float wp_dist = sqrtf(wp_x_cm*wp_x_cm + wp_y_cm*wp_y_cm);
            mcu_printf("│ Waypoint: Valid | X=%d mm, Y=%d mm | Dist=%.1f cm\n",
                       scenario->leader_x_mm, scenario->leader_y_mm, wp_dist);
        } else {
            mcu_printf("│ Waypoint: INVALID (signal lost)\n");
        }
        mcu_printf("└────────────────────────────────────────────────────────────\n");

        uint32_t elapsed_ms = 0;
        const uint32_t interval_ms = 100;
        
        while(elapsed_ms < scenario->duration_ms)
        {
            // 패킷 큐에 삽입
            (void)SAL_QueuePut(g_motor_queue_id, (void *)&fake_pkt, 
                              sizeof(to_vcp_spi_msg_t), 0, SAL_OPT_NON_BLOCKING);
            
            // 100ms 대기
            SAL_TaskSleep(interval_ms);
            elapsed_ms += interval_ms;
            
            // 시퀀스 번호 증가
            fake_pkt.vcp_msg.seq = ++packet_seq;
            fake_pkt.vcp_msg.cpu_time_ms = packet_seq * 100;
        }
        
        // 다음 시나리오로 이동
        scenario_idx = (scenario_idx + 1) % NUM_SCENARIOS;
        
        // 한 사이클 완료 시 구분선
        if (scenario_idx == 0) {
            mcu_printf("\n");
            mcu_printf("╔════════════════════════════════════════════════════════════╗\n");
            mcu_printf("║     Test Cycle Complete - Restarting...                   ║\n");
            mcu_printf("╚════════════════════════════════════════════════════════════╝\n");
            mcu_printf("\n");
            SAL_TaskSleep(3000);  // 사이클 간 3초 대기
        }
    }
}
=========================================================================================*/
void ipc_init(void)
{
    //(void)SAL_QueueCreate(&g_motor_queue_id, (const uint8 *)"MotorQ", QUEUE_LENGTH, sizeof(uint32));

    (void)SAL_QueueCreate(&IPC_queue_id, (const uint8 *)"IPCTask", QUEUE_LENGTH, sizeof(to_vcp_spi_msg_t));
    /* 2. 태스크 생성 */
    /* SAL_TaskCreate(ID, 이름, 함수, 스택, 스택크기, 우선순위, 파라미터) */
    (void)SAL_TaskCreate(&IPC_task_id, 
                         (const uint8 *)"IPCTask", 
                         (SALTaskFunc)IPC_Task, 
                         IPC_task_stack, 
                         IPC_TASK_STACK_SIZE, 
                         IPC_TASK_PRIORITY, // 우선순위 (0~15 중 적절히 선택)
                         NULL_PTR);

    /* 3. 테스트용 더미 생산자 태스크 생성 */
    // static uint32 dummy_task_id = 0;
    // static uint32 dummy_task_stack[512];
    // (void)SAL_TaskCreate(&dummy_task_id,
    //                      (const uint8 *)"DummyProducer",
    //                      (SALTaskFunc)vDummyProducerTask,
    //                      dummy_task_stack,
    //                      512,
    //                      4, // 우선순위 (MotorTask보다 낮게 설정)
    //                      NULL_PTR);
}
