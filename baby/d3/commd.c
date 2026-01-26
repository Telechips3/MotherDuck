// commd.c
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>

#include "proto.h"

static volatile sig_atomic_t g_run = 1;
static void on_sig(int s){ (void)s; g_run = 0; }

static int set_nonblock(int fd){
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0) return -1;
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

int main(int argc, char** argv)
{
    int udp_port = (argc >= 2) ? atoi(argv[1]) : 5005;
    const char* uds_path = (argc >= 3) ? argv[2] : "/tmp/waypoint.sock";

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    // UDP RX
    int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) { perror("udp socket"); return 1; }

    struct sockaddr_in in = {0};
    in.sin_family = AF_INET;
    in.sin_addr.s_addr = htonl(INADDR_ANY);
    in.sin_port = htons((uint16_t)udp_port);
    if (bind(udp_fd, (struct sockaddr*)&in, sizeof(in)) < 0) {
        perror("udp bind"); return 1;
    }
    set_nonblock(udp_fd);

    // UDS TX (logicd가 bind해둔 주소로 sendto)
    int uds_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (uds_fd < 0) { perror("uds socket"); return 1; }

    struct sockaddr_un dst = {0};
    dst.sun_family = AF_UNIX;
    strncpy(dst.sun_path, uds_path, sizeof(dst.sun_path)-1);

    fprintf(stderr, "[commd] UDP :%d -> UDS %s\n", udp_port, uds_path);

    uint8_t rx[2048];

    while (g_run) {
        ssize_t n = recvfrom(udp_fd, rx, sizeof(rx), 0, NULL, NULL);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) { usleep(1000); continue; }
            perror("[commd] recvfrom"); break;
        }

        // 길이 체크
        if (n != (ssize_t)sizeof(WaypointPktV1)) {
            fprintf(stderr, "[commd] drop: size=%zd (expect %zu)\n", n, sizeof(WaypointPktV1));
            continue;
        }

        WaypointPktV1 pkt;
        memcpy(&pkt, rx, sizeof(pkt));

        // magic/type 체크
        if (pkt.magic != WP_MAGIC) {
            fprintf(stderr, "[commd] drop: bad magic 0x%02X\n", pkt.magic);
            continue;
        }

        // CRC 체크 (리틀엔디안 가정: 그냥 값 비교)
        uint16_t crc = pkt_calc_crc(&pkt);
        if (crc != pkt.crc16) {
            fprintf(stderr, "[commd] drop: bad crc calc=0x%04X pkt=0x%04X\n", crc, pkt.crc16);
            continue;
        }

        // 검증 통과 → logicd로 전달
        ssize_t s = sendto(uds_fd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&dst, sizeof(dst));
        if (s < 0) {
            // logicd가 아직 안 떠서 소켓 파일이 없으면 ENOENT 가능
            if (errno != ENOENT) perror("[commd] sendto uds");
        }
    }

    close(udp_fd);
    close(uds_fd);
    fprintf(stderr, "[commd] exit\n");
    return 0;
}
