#include "ipc.h"
#include "speed.h"
#include "steer.h"

uint32 g_motor_queue_id = 0;
static uint32 g_motor_task_id = 0;

#define MOTOR_TASK_STACK_SIZE 512
static uint32 g_motor_task_stack[MOTOR_TASK_STACK_SIZE];

static void vMotorControlTask(void *pParam)
{
    uint32 received_cmd = 0xFF;
    uint32 copied_size = 0;
    SALRetCode_t ret;

    while (1)
    {
        ret = SAL_QueueGet(g_motor_queue_id, &received_cmd, &copied_size, 
                           100, SAL_OPT_BLOCKING);

        if (ret == SAL_RET_SUCCESS)
        {
            // 속도 명령
            if (received_cmd >= CMD_SPEED_BASE && 
                received_cmd < (CMD_SPEED_BASE + 9))
            {
                uint32 speed_level = received_cmd - CMD_SPEED_BASE + 1;
                control_motor_speed(speed_level);
            }
            // 모터 제어 (전진/후진/조합)
            else if (received_cmd == CMD_FORWARD || 
                     received_cmd == CMD_BACKWARD ||
                     received_cmd >= CMD_FORWARD_LEFT)
            {
                control_motor_drive(received_cmd);
                
                // 조합 명령이면 조향도 함께 처리
                if (received_cmd >= CMD_FORWARD_LEFT && 
                    received_cmd <= CMD_BACKWARD_RIGHT)
                {
                    control_steering_step(received_cmd);
                }
            }
            // 조향만 (A, D 단독)
            else if (received_cmd == CMD_LEFT || received_cmd == CMD_RIGHT)
            {
                control_steering_step(received_cmd);
            }
        }
        else
        {
            control_motor_drive(0xFF);
        }
        
        SAL_TaskSleep(1);
    }
}

void ipc_init(void)
{
    (void)SAL_QueueCreate(&g_motor_queue_id, (const uint8 *)"MotorQ", 
                          20, sizeof(uint32));  // 큐 깊이 증가

    (void)SAL_TaskCreate(&g_motor_task_id, 
                         (const uint8 *)"MotorTask", 
                         (SALTaskFunc)vMotorControlTask, 
                         g_motor_task_stack, 
                         MOTOR_TASK_STACK_SIZE, 
                         5,
                         NULL_PTR);
}