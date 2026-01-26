// logic_latest.c
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>

#include "proto.h"

static int set_nonblock(int fd){
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0) return -1;
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

static uint64_t mono_ns(void){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}

static void sleep_ms(int ms){
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

int main(int argc, char** argv)
{
    const char* path = (argc >= 2) ? argv[1] : "/tmp/waypoint.sock";

    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return 1; }

    unlink(path);
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }
    set_nonblock(fd);

    printf("[logic] bind ok: %s\n", path);

    WaypointPktV1 latest = {0};
    uint64_t latest_rx_ns = 0;
    int have = 0;

    while (1) {
        // 1) drain: 큐에 쌓인거 다 읽고 마지막만 남김
        int drained = 0;
        for (;;) {
            WaypointPktV1 pkt;
            ssize_t n = recvfrom(fd, &pkt, sizeof(pkt), 0, NULL, NULL);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                perror("recvfrom");
                break;
            }
            if (n != (ssize_t)sizeof(pkt)) continue;
            if (pkt.magic != WP_MAGIC) continue;
            if (pkt_calc_crc(&pkt) != pkt.crc16) continue;

            latest = pkt;
            latest_rx_ns = mono_ns();
            have = 1;
            drained++;
        }

        // 2) “소비”는 지금은 100ms마다 한 번만 출력(의사결정 대신)
        uint64_t now = mono_ns();
        if (have) {
            double age_ms = (double)(now - latest_rx_ns) / 1e6;
            printf("[logic] drained=%d latest: type=%u exc=0x%02X x=%d y=%d age=%.1fms\n",
                   drained, (unsigned)latest.type, (unsigned)latest.exception,
                   (int)latest.x_mm, (int)latest.y_mm, age_ms);
        } else {
            printf("[logic] drained=%d (no pkt yet)\n", drained);
        }

        sleep_ms(100); // 나중에 여기 20ms로 바꾸면 50Hz decision 루프가 됨
    }

    return 0;
}
