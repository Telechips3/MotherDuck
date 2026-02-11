#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <arpa/inet.h>
#include "../common/Proto/proto_spi.h"
#include "../proto.h"

#include "decide_mode.h"
#include "ArUco_RX.h"
#include "Waypoint_RX.h"

#define SPI_DEVICE "/dev/spidev0.0"

// --- [리더님 요청 1: 전역 배열 선언] ---
// 1바이트만 쓰더라도 배열로 관리하면 추후 프로토콜 확장이 유리합니다.

static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 1000000;

to_vcp_msg_t msg = {0};
to_vcp_spi_msg_t tx_msg = {0};
static uint32_t rx_buf[32] = {0x00};

static void print_vcp_msg(const to_vcp_msg_t *msg) {
    if (msg == NULL) return;

    printf("\n================ [ VCP Message Data ] ================\n");
    
    // 요청하신 순서 그대로 출력
    printf("1.  Sequence        : %u\n", msg->seq);
    printf("2.  CPU Time (ms)   : %u\n", msg->cpu_time_ms);
    printf("3.  Mode            : %u\n", msg->mode);
    printf("4.  Leader State    : %u\n", msg->leader_state);
    
    printf("5.  ArUco Valid     : %u\n", msg->aruco_valid);
    printf("6.  ArUco Age (ms)  : %u\n", msg->aruco_age_ms);
    printf("7.  ArUco Dist (mm) : %d\n", msg->aruco_dist_mm);      // int16_t이므로 %d
    printf("8.  ArUco X (Q15)   : %d\n", msg->aruco_x_norm_q15);  // int16_t이므로 %d
    
    printf("9.  WP Valid        : %u\n", msg->wp_valid);
    printf("10. WP Age (ms)     : %u\n", msg->wp_age_ms);
    printf("11. Leader X (mm)   : %d\n", msg->leader_x_mm);      // int32_t이므로 %d
    printf("12. Leader Y (mm)   : %d\n", msg->leader_y_mm);      // int32_t이므로 %d
    
    printf("13. Reason          : %u\n", msg->reason);

    printf("======================================================\n");
}

static void operator_equal_msg(to_vcp_msg_t *dest, to_vcp_msg_t *src)
{
    dest->seq = src->seq;
    dest->cpu_time_ms = src->cpu_time_ms;
    dest->mode = src->mode;
    dest->leader_state = src->leader_state;
    dest->aruco_valid = src->aruco_valid;
    dest->aruco_age_ms = src->aruco_age_ms;
    dest->aruco_dist_mm = src->aruco_dist_mm;
    dest->aruco_x_norm_q15 = src->aruco_x_norm_q15;
    dest->wp_valid = src->wp_valid;
    dest->wp_age_ms = src->wp_age_ms;
    dest->leader_x_mm = src->leader_x_mm;
    dest->leader_y_mm = src->leader_y_mm;
    dest->reason = src->reason;
}

static int spi_init(const char *device)
{
    int fd = open(device, O_RDWR);
    if (fd < 0)
    {
        perror("SPI 장치 열기 실패");
        return -1;
    }
    // 기본 설정 유지 (8비트 모드)
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    return fd;
}

void spi_send_byte(int fd)
{
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)&tx_msg,
        .rx_buf = (unsigned long)rx_buf,
        .len = 32,
        .delay_usecs = 0,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
    {
        perror("[Linux Master] SPI Transfer Failed");
    }
}
// ===== Payload msg ==============

void init_decide_mode_msg(void)
{
    memset(&msg, 0, sizeof(msg));
}

uint32_t GetTickCount(void)
{
    struct timespec ts;

    // CLOCK_MONOTONIC: 시간 수정돼도 영향 안 받는 절대 시간
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        // 에러 나면 0 리턴하거나 로그 찍으셈
        perror("clock_gettime failed");
        return 0;
    }

    // 초(s) * 1000 + 나노초(ns) / 1,000,000 = 밀리초(ms)
    uint64_t ms = (uint64_t)ts.tv_sec * 1000ull + (uint64_t)(ts.tv_nsec / 1000000ull);
    return (uint32_t)ms; // 여기서만 32비트로 잘라서 roll-over 자연스럽게
}

int main(int argc, char **argv)
{
    int spi_fd;
    uint32_t seq = 0;
    init_decide_mode_msg();

    aruco_rx_init(); // ArUco 소켓 통신 초기화 호출
    const char *wp_path = (argc >= 2) ? argv[1] : NULL;
    waypoint_rx_init(wp_path); // Waypoint UDS 수신 초기화

    spi_fd = spi_init(SPI_DEVICE);
    if (spi_fd < 0)
        exit(EXIT_FAILURE);

    tx_msg.magic = 0xA5;
    while (1)
    {
        uint8_t ret = 0; // Handler
        uint32_t time_now = 0;

        time_now = GetTickCount();

        msg.cpu_time_ms = time_now;
        msg.seq = seq++;

        // aruco marker input
        uint8_t aruco_ret = RX_aruco_marker(&msg);
        if (aruco_ret == 0)
        {
            // 새로운 데이터를 성공적으로 받은 경우
        }
        else
        {
            // 데이터를 못 받았거나 연결에 문제가 있는 경우
        }

        // waypoint input
        ret = RX_waypoint(&msg);
        if (ret)
        {
            fprintf(stderr, "[seq : %u| time : %u] Failed to receive waypoint data\n", msg.seq, time_now);
        }

        ret = decide_mode_step(&msg);

        time_now = GetTickCount();
        printf("[seq : %u| time : %u] mode=%u reason=%u leader_state : %u\n",
               msg.seq, time_now, msg.mode, msg.reason, msg.leader_state);
        printf("[ArUco status] aruco_valid : %u, aruco_age_ms : %u, aruco_dist_mm : %d, aruco_x_norm_q15 : %d\n",
               msg.aruco_valid, msg.aruco_age_ms, msg.aruco_dist_mm, msg.aruco_x_norm_q15);
        printf("[Waypoint status] wp_valid: %u, wp_age_ms : %u, leader_x_mm : %d, leader_y_mm : %d\n",
               msg.wp_valid, msg.wp_age_ms, msg.leader_x_mm, msg.leader_y_mm);

        // 실제 통신 인터페이스 모듈(SPI)
        operator_equal_msg(&tx_msg.vcp_msg, &msg);

        size_t payload_len = sizeof(to_vcp_spi_msg_t) - sizeof(uint16_t);
        tx_msg.crc16 = crc16_ccitt_false((uint8_t *)&tx_msg, payload_len);

        print_vcp_msg(&tx_msg.vcp_msg);

        // SPI 전송 (32바이트 통째로)
        spi_send_byte(spi_fd);

        usleep(10000); // 50ms delay
    }

    return 0;
}