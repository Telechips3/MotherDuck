#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <arpa/inet.h>

#define PORT 8080
#define BUF_SIZE 1024
#define SPI_DEVICE "/dev/spidev0.0" // 사용 중인 SPI 장치 경로 확인 필요

// SPI 설정 값
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 500000; // 500kHz

// SPI 초기화 함수
int spi_init(const char *device) {
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("SPI 장치 열기 실패");
        return -1;
    }
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    return fd;
}

// SPI 데이터 전송 함수
void spi_send(int fd, char data) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)&data,
        .rx_buf = 0,
        .len = 1,
        .delay_usecs = 0,
        .speed_hz = speed,
        .bits_per_word = bits,
    };
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("SPI 전송 실패");
    }
}

int main() {
    int sockfd, spi_fd;
    char buffer[BUF_SIZE];
    struct sockaddr_in servaddr, cliaddr;

    // 1. SPI 초기화
    spi_fd = spi_init(SPI_DEVICE);
    if (spi_fd < 0) exit(EXIT_FAILURE);

    // 2. UDP 소켓 생성
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("소켓 생성 실패");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("바인딩 실패");
        exit(EXIT_FAILURE);
    }

    printf("Topst D3-G: UDP 수신 및 SPI 전달 대기 중...\n");

    while (1) {
        socklen_t len = sizeof(cliaddr);
        int n = recvfrom(sockfd, buffer, BUF_SIZE, MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
        buffer[n] = '\0';

        char key = buffer[0];
        
        // SPI로 데이터 즉시 전송
        if (key == 'w' || key == 'a' || key == 's' || key == 'd') {
            spi_send(spi_fd, key);
            printf("[SPI 송신] 명령: %c\n", key);
        }

        switch(key) {
            case 'w': printf("[제어] 전진 (Forward)\n"); break;
            case 'a': printf("[제어] 좌회전 (Left)\n"); break;
            case 's': printf("[제어] 후진 (Backward)\n"); break;
            case 'd': printf("[제어] 우회전 (Right)\n"); break;
            case 'q': 
                printf("종료합니다.\n");
                close(sockfd);
                close(spi_fd);
                return 0;
        }
    }
    return 0;
}