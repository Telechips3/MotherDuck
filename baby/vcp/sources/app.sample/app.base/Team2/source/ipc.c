#include "ipc.h"
#include "speed.h" // control_motor_drive 함수 호출용
#include "steer.h" // control_steering_step 함수 호출용

/* 전역 변수 */
uint32 g_motor_queue_id = 0;
static uint32 g_motor_task_id = 0;

/* 태스크용 스택 배열 (SAL_TaskCreate는 정적 스택을 요구함) */
#define MOTOR_TASK_STACK_SIZE 512
static uint32 g_motor_task_stack[MOTOR_TASK_STACK_SIZE];

/* [소비자 태스크] 큐를 감시하다가 데이터가 없으면 멈춤 */
static void vMotorControlTask(void *pParam)
{
    uint32 received_cmd = 0xFF;
    uint32 copied_size = 0;
    SALRetCode_t ret;

    while (1)
    {
        /* * SAL_QueueGet 특징: 
         * - 4번째 인자(iTimeout): 200ms 대기
         * - 데이터가 오면 SAL_RET_SUCCESS
         * - 200ms 동안 데이터가 안 오면 SAL_ERR_FAIL_GET_DATA (타임아웃) 반환
         */
        ret = SAL_QueueGet(g_motor_queue_id, &received_cmd, &copied_size, 200, SAL_OPT_BLOCKING);

        if (ret == SAL_RET_SUCCESS)
        {
            // 명령 수신 성공: 전진/후진 등 실행
            if(received_cmd == 0 || received_cmd == 2)
            {
                control_motor_drive(received_cmd);
            }
            else{
                control_steering_step(received_cmd);
            }
        }
        else
        {
            // 타임아웃 발생: 리눅스에서 'w'를 뗐다고 판단
            // control_motor_drive 내부의 else(정지) 로직을 타게 함
            control_motor_drive(0xFF); 
        }
        SAL_TaskSleep(1);
    }
}

void ipc_init(void)
{
    /* 1. 큐 생성 (깊이 10, 데이터 크기 uint32) */
    (void)SAL_QueueCreate(&g_motor_queue_id, (const uint8 *)"MotorQ", QUEUE_LENGTH, sizeof(uint32));

    /* 2. 태스크 생성 */
    /* SAL_TaskCreate(ID, 이름, 함수, 스택, 스택크기, 우선순위, 파라미터) */
    (void)SAL_TaskCreate(&g_motor_task_id, 
                         (const uint8 *)"MotorTask", 
                         (SALTaskFunc)vMotorControlTask, 
                         g_motor_task_stack, 
                         MOTOR_TASK_STACK_SIZE, 
                         5, // 우선순위 (0~15 중 적절히 선택)
                         NULL_PTR);
}