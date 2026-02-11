#ifndef _TEAM2_COMMON_H_  // 중복 포함 방지 시작
#define _TEAM2_COMMON_H_

#include <sal_api.h> // BSP 기본 API 포함 (필요 시)
#include <gpio.h>
#include <pdm.h>
#include <gpsb.h>
#include <stdint.h>
#include "gic.h"

#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>

#include <FreeRTOS.h>
#include <task.h>

//울트라.h
#define ULTRA_TRIG_GPIO       GPIO_GPA(23)
#define ULTRA_ECHO_GPIO       GPIO_GPA(24)
#define ULTRA_ECHO_TIMEOUT    0xFFFFFFFF

//speed.h
#define MOTOR_IN1           GPIO_GPA(4)      // L298N IN1
#define MOTOR_IN2           GPIO_GPA(8)      // L298N IN2
#define MOTOR_ENA_CH        0                // L298N ENA (PDM CH0 - GPIO A10)

// GPIO
/* ===== Encoder GPIO ===== */
#define ENC_A_GPIO          GPIO_GPA(22)
#define ENC_B_GPIO          GPIO_GPA(21)
#define EIT_ENCODER_A        (GIC_EXT4)
#define EIT_ENCODER_B        (GIC_EXT5)

/* ===== Motor GPIO ===== */

#define BUZZER_GPIO         GPIO_GPA(19)

//spi.h
#define SPI_CS_GPIO     GPIO_GPB(5)
#define SPI_SCLK_GPIO   GPIO_GPB(4)
#define SPI_MOSI_GPIO   GPIO_GPB(6)
#define SPI_MISO_GPIO   GPIO_GPB(7)

//Interrupt Example
#define EIT (GIC_EXT4)
#define MY_GPIO (GPIO_GPB(2))

//task delay
#define IPC_TASK_SLEEP_MS           60
#define IMU_TASK_SLEEP_MS           20
#define POSE_TASK_SLEEP_MS          500
#define ENCODER_TASK_SLEEP_MS       1000
#define ULTRASONIC_TASK_SLEEP_MS    50
#define FOLLOW_STEER_TASK_SLEEP_MS  50
#define SENSOR_TASK_PERIOD_MS       20

//task priorities 클수록 좋아요
#define IPC_TASK_PRIORITY           SAL_PRIO_APP_CFG
#define IMU_TASK_PRIORITY           SAL_PRIO_APP_CFG
#define POSE_TASK_PRIORITY          SAL_PRIO_APP_CFG
#define ENCODER_TASK_PRIORITY       SAL_PRIO_APP_CFG
#define ULTRASONIC_TASK_PRIORITY    SAL_PRIO_APP_CFG
#define FOLLOW_STEER_TASK_PRIORITY  SAL_PRIO_APP_CFG
#define SENSOR_TASK_TASK_PRIORITY   SAL_PRIO_APP_CFG

//task stack sizes
#define IPC_TASK_STACK_SIZE           512
#define IMU_TASK_STACK_SIZE           512
#define POSE_TASK_STACK_SIZE          512
#define ENCODER_TASK_STACK_SIZE       512
#define ULTRASONIC_TASK_STACK_SIZE    512
#define FOLLOW_STEER_TASK_STACK_SIZE  512
#define SENSOR_TASK_STACK_SIZE        512

// 제어 모드 정의
typedef enum {
    MODE_ESTOP = 0,
    MODE_STOP_AND_HOLD = 1,
    MODE_FOLLOW_WAYPOINT = 2,
    MODE_FOLLOW_VISION = 3,
} ctrl_mode_t;

#pragma pack(push, 1)

// 핵심 데이터 구조체 (29 byte)
// valid -> D3에서 해당 데이터가 신뢰성 없다고 판단될 때 0으로 내려서 보내줌
typedef struct {
    uint32_t seq;
    uint32_t cpu_time_ms;

    uint8_t mode;
    uint8_t leader_state;

    // ArUco 데이터
    uint8_t  aruco_valid;
    uint16_t aruco_age_ms;
    int16_t  aruco_dist_mm;
    int16_t  aruco_x_norm_q15;

    // Waypoint 데이터
    uint8_t  wp_valid;
    uint16_t wp_age_ms;
    int32_t  leader_x_mm;
    int32_t  leader_y_mm;

    uint8_t  reason;
} to_vcp_msg_t;

extern uint32 sem_ultra;
// 전체 SPI 패킷 구조체
typedef struct {
    uint8_t magic;           // 0xA5
    to_vcp_msg_t vcp_msg;    // 실제 메시지
    uint16_t crc16;          // 체크섬
} to_vcp_spi_msg_t;

#pragma pack(pop)

static inline uint16_t crc16_ccitt_false(const uint8_t* data, size_t len)
{
    // CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, xorout 0x0000, refin/refout false
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else             crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}


#endif // _TEAM2_COMMON_H_ 끝
