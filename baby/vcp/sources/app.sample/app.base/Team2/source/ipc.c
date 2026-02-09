#include "ipc.h"
#include "speed.h" // control_motor_drive 함수 호출용
#include "steer.h" // control_steering_step 함수 호출용
#include "parsing.h" // parse_and_excute_control 함수 호출용
#include "team2_header.h"

// Vision 더미 패킷 테스트
#define IPC_VISION_DUMMY_TEST 1

/* 전역 변수 */
uint32 g_motor_queue_id = 0;
static uint32 g_motor_task_id = 0;

/* 태스크용 스택 배열 (SAL_TaskCreate는 정적 스택을 요구함) */
#define MOTOR_TASK_STACK_SIZE 512
static uint32 g_motor_task_stack[MOTOR_TASK_STACK_SIZE];

/* [소비자 태스크] 큐를 감시하다가 데이터가 없으면 멈춤 */
static void vMotorControlTask(void *pParam)
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
    while(1)
    {
#if (IPC_VISION_DUMMY_TEST == 1)
        static int16_t x_norm = -16000;
        static int16_t step = 2000;
        static int16_t dist_mm = 400;   // 40cm
        static int16_t dist_step = 50;  // 5cm
        to_vcp_spi_msg_t pkt;

        pkt.magic = 0xA5;
        pkt.vcp_msg.seq = 1;
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

        x_norm = (int16_t)(x_norm + step);
        if (x_norm >= 16000 || x_norm <= -16000)
        {
            step = (int16_t)-step;
        }
        dist_mm = (int16_t)(dist_mm + dist_step);
        if (dist_mm >= 1200 || dist_mm <= 200)
        {
            dist_step = (int16_t)-dist_step;
        }

        SAL_TaskSleep(50);
#else
        to_vcp_spi_msg_t received_pkt; // 구조체 패킷 수신
        uint32 copied_size = 0; // 수신 크기
        SALRetCode_t ret;

        ret = SAL_QueueGet(g_motor_queue_id, &received_pkt, &copied_size, 200, SAL_OPT_BLOCKING);

        if(ret == SAL_RET_SUCCESS)
        {
            if(received_pkt.magic == 0xA5)
            {
                // 유효한 패킷 수신 시 제어 함수 호출
                parse_and_excute_control((void*)&received_pkt);
            }
        }
        else{
            // 타임아웃 발생 시 정지 명령 실행
            control_motor_drive(0xFF);
        }
        SAL_TaskSleep(1);
#endif
    }
}
//         .mode = MODE_FOLLOW_VISION,
//         .aruco_valid = 1,
//         .aruco_dist_mm = 600,
//         .aruco_x_norm = 0,
//         .wp_valid = 1,          // 두 신호 모두 있음
//         .leader_x_mm = 700,
//         .leader_y_mm = 0,
//         .duration_ms = 3000
//     },
    
//     // === 시나리오 12: 초근접 위험 상황 ===
//     {
//         .scenario_name = "CRITICAL: Ultra Close (5cm)",
//         .mode = MODE_FOLLOW_VISION,
//         .aruco_valid = 1,
//         .aruco_dist_mm = 50,    // 5cm! (비상!)
//         .aruco_x_norm = 0,
//         .wp_valid = 0,
//         .leader_x_mm = 0,
//         .leader_y_mm = 0,
//         .duration_ms = 5000     // 긴 시간 관찰
//     }
// };

void ipc_init(void)
{
    //(void)SAL_QueueCreate(&g_motor_queue_id, (const uint8 *)"MotorQ", QUEUE_LENGTH, sizeof(uint32));

    (void)SAL_QueueCreate(&g_motor_queue_id, (const uint8 *)"MotorQ", 10, sizeof(to_vcp_spi_msg_t));
    /* 2. 태스크 생성 */
    /* SAL_TaskCreate(ID, 이름, 함수, 스택, 스택크기, 우선순위, 파라미터) */
    (void)SAL_TaskCreate(&g_motor_task_id, 
                         (const uint8 *)"MotorTask", 
                         (SALTaskFunc)vMotorControlTask, 
                         g_motor_task_stack, 
                         MOTOR_TASK_STACK_SIZE, 
                         5, // 우선순위 (0~15 중 적절히 선택)
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
