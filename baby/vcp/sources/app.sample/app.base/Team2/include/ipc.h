#ifndef _IPC_H_
#define _IPC_H_

#include "sal_api.h"

#define QUEUE_LENGTH    1

/* 전역 큐 ID (SPI 등 다른 파일에서 참조) */
extern uint32 g_motor_queue_id;

/* 초기화 함수 */
void ipc_init(void);

#endif