#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <arpa/inet.h>
#include <stdint.h>
#include "../common/Proto/proto_spi.h"
#include "../proto.h"

#define PORT 8080
#define SPI_DEVICE "/dev/spidev0.0"

to_vcp_spi_msg_t tx_buf;
static uint32_t rx_buf[32] = {0x00};

static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 500000;

void operator_equal_msg(to_vcp_msg_t *dest, to_vcp_msg_t *src);
void print_vcp_msg(const to_vcp_msg_t *msg);

int spi_init(const char *device)
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

void operator_equal(to_vcp_spi_msg_t *dest, to_vcp_spi_msg_t *src)
{
    dest->magic = src->magic;
    operator_equal_msg(&dest->vcp_msg, &src->vcp_msg);
    dest->crc16 = src->crc16;
}

void operator_equal_msg(to_vcp_msg_t *dest, to_vcp_msg_t *src)
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

void spi_send_byte(int fd, to_vcp_spi_msg_t data)
{
    operator_equal(&tx_buf, &data);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)&tx_buf,
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

    printf("[SPI 수신 완료] x = 0x%08X (%u)\n", rx_buf[0], rx_buf[0]);
}

int main()
{
    int spi_fd;
    FILE *fp;

    spi_fd = spi_init(SPI_DEVICE);
    if (spi_fd < 0)
        exit(EXIT_FAILURE);

    fp = fopen("testcase.txt", "r");
    if (fp == NULL)
    {
        perror("파일 열기 실패");
        close(spi_fd);
        exit(EXIT_FAILURE);
    }

    printf("Topst D3-G: testcase.txt 상세 시나리오 시작\n");

    while (1)
    {
        to_vcp_spi_msg_t send_data = {0};
        to_vcp_msg_t *m = &send_data.vcp_msg;

        // 구조체 순서대로 13개 데이터 파싱
        // %u(32bit), %hhu(8bit), %hu(16bit), %hd(16bit signed), %d(32bit signed)
        int res = fscanf(fp, "%u %u %hhu %hhu %hhu %hu %hd %hd %hhu %hu %d %d %hhu",
                         &m->seq,              // uint32_t
                         &m->cpu_time_ms,      // uint32_t
                         &m->mode,             // uint8_t
                         &m->leader_state,     // uint8_t
                         &m->aruco_valid,      // uint8_t
                         &m->aruco_age_ms,     // uint16_t
                         &m->aruco_dist_mm,    // int16_t
                         &m->aruco_x_norm_q15, // int16_t
                         &m->wp_valid,         // uint8_t
                         &m->wp_age_ms,        // uint16_t
                         &m->leader_x_mm,      // int32_t
                         &m->leader_y_mm,      // int32_t
                         &m->reason            // uint8_t
        );

        if (res == EOF)
            break;
        if (res < 13)
        {
            // 한 줄에 데이터가 모자라면 나머지 건너뜀
            fscanf(fp, "%*[^\n]");
            continue;
        }

        // 공용 헤더 설정
        send_data.magic = 0xA5;

        size_t payload_len = sizeof(to_vcp_spi_msg_t) - sizeof(uint16_t);
        send_data.crc16 = crc16_ccitt_false((uint8_t *)&send_data, payload_len);

        print_vcp_msg(m);

        // SPI 전송 (32바이트 통째로)
        spi_send_byte(spi_fd, send_data);

        // 전송 로그 (주요 데이터 확인)
        printf("[SPI] Seq: %u, Mode: %u, Dist: %d mm\n",
               m->seq, m->mode, m->aruco_dist_mm);

        usleep(1000000); // 50ms 간격
    }

    printf("테스트 완료\n");
    fclose(fp);
    close(spi_fd);
    return 0;
}

void print_vcp_msg(const to_vcp_msg_t *msg) {
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