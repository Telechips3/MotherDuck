#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <arpa/inet.h>
#include <stdint.h>

#define PORT 8080
#define SPI_DEVICE "/dev/spidev0.0"

// --- [리더님 요청 1: 전역 배열 선언] ---
// 1바이트만 쓰더라도 배열로 관리하면 추후 프로토콜 확장이 유리합니다.
static uint8_t tx_buf[1] = {0x00};

static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 500000;

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

// --- [리더님 요청 2: write()를 이용한 1바이트 전송] ---
void spi_send_byte(int fd, uint8_t val)
{
    tx_buf[0] = val; // 전역 배열의 첫 번째 칸에 값 대입
    printf("[SPI 송신] tx_buf[0] = %u\n", tx_buf[0]);
    
    // 배열의 시작 주소를 넘기고, 길이를 1바이트로 지정합니다.
    if (write(fd, tx_buf, 1) != 1)
    {
        perror("SPI write 실패");
    }
}

int main()
{
    int sockfd, spi_fd;
    char udp_buffer[1];
    struct sockaddr_in servaddr, cliaddr;

    spi_fd = spi_init(SPI_DEVICE);
    if (spi_fd < 0)
        exit(EXIT_FAILURE);

    // UDP 소켓 설정
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        exit(EXIT_FAILURE);
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);
    bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr));

    printf("Topst D3-G: 1바이트 전역배열 송신 대기 중...\n");

    while (1)
    {
        socklen_t len = sizeof(cliaddr);
        // UDP로 키 입력 한 글자 수신
        int n = recvfrom(sockfd, udp_buffer, 1, MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
        if (n <= 0)
            continue;

        uint8_t cmd = 0xFF;
        switch (udp_buffer[0])
        {
        case 'w':
            cmd = 0;
            break;
        case 'a':
            cmd = 1;
            break;
        case 's':
            cmd = 2;
            break;
        case 'd':
            cmd = 3;
            break;
        case 'q':
            close(sockfd);
            close(spi_fd);
            return 0;
        default:
            continue;
        }

        if (cmd != 0xFF)
        {
            spi_send_byte(spi_fd, cmd);
        }
    }
    return 0;
}