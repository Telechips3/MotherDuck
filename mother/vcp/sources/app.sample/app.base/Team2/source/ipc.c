#include <math.h>
#include "../team2_header.h"
#include "../include/ipc.h"
#include "speed.h"
#include "steer.h"

uint32 g_motor_queue_id = 0;
static uint32 g_motor_task_id = 0;

#define MOTOR_TASK_STACK_SIZE 512
static uint32 g_motor_task_stack[MOTOR_TASK_STACK_SIZE];

static void Dump_Vcp_Hex(void* m, int32 len)
{
    uint8_t* p = (uint8_t*)m;
    int size = len;

    mcu_printf("[SPI HEX DUMP] Size: %d\n", size);

    for (int i = 0; i < size; i++)
    {
        // %02x: 2자리 확보하고 빈 곳은 0으로 채움 (0xA5 -> a5, 0x07 -> 07)
        // 사용자님의 DBG_Printfi 내부 'fill'과 'fminString' 로직을 활용합니다.
        mcu_printf("%02x ", (int)p[i]);

        // 8바이트마다 줄바꿈해서 보기 편하게 출력
        if ((i + 1) % 8 == 0) {
            mcu_printf("\n");
        }
    }
    mcu_printf("\n----------------------\n");
}

static void vMotorControlTask(void *pParam)
{
    spi_motor_packet_t received_cmd = {0};
    //uint8 received_cmd = 0xFF;
    uint32 copied_size = 0;
    SALRetCode_t ret;

    //control_motor_drive(0);

    while (1)
    {
        ret = SAL_QueueGet(g_motor_queue_id, &received_cmd, &copied_size, 
                           100, SAL_OPT_BLOCKING);
        
        Dump_Vcp_Hex(&received_cmd, 8);
        
        if (ret == SAL_RET_SUCCESS)
        {
            // 속도 명령
            if (( fabsf(received_cmd.data.speed - 1.0f)) < EPSILON)
            {
                mcu_printf("w\n");
                control_motor_drive(0);
            }
            else if( (fabsf(received_cmd.data.speed + 1.0f)) < EPSILON)
            {
                mcu_printf("s\n");
                control_motor_drive(2);
            }
            else{
                control_motor_drive(0xFF);
            }

            if(( fabsf(received_cmd.data.steer - 0.291f)) < EPSILON)
            {
                mcu_printf("a\n");
                control_steering_step(1);
            }
            else if((fabsf(received_cmd.data.steer + 0.291f)) < EPSILON)
            {
                mcu_printf("d\n");
                control_steering_step(3);
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
                          100, 8);  // 큐 깊이 증가

    (void)SAL_TaskCreate(&g_motor_task_id, 
                         (const uint8 *)"MotorTask", 
                         (SALTaskFunc)vMotorControlTask, 
                         g_motor_task_stack, 
                         MOTOR_TASK_STACK_SIZE, 
                         5,
                         NULL_PTR);
}