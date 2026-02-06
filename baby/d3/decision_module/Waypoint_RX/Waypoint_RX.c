#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <time.h>

#include "Waypoint_RX.h"
#include "../../proto.h"

#define WAYPOINT_SOCKET_DEFAULT "/tmp/waypoint.sock"

static int wp_sock = -1;
static char wp_path[sizeof(((struct sockaddr_un*)0)->sun_path)] = WAYPOINT_SOCKET_DEFAULT;
static WaypointPktV1 wp_last_pkt;
static uint32_t wp_last_rx_ms = 0;
static int wp_have_last = 0;

static int set_nonblock(int fd)
{
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0) return -1;
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

static uint32_t mono_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ull + (uint64_t)(ts.tv_nsec / 1000000ull));
}

void waypoint_rx_init(const char* path)
{
    if (path && path[0] != '\0') {
        strncpy(wp_path, path, sizeof(wp_path) - 1);
        wp_path[sizeof(wp_path) - 1] = '\0';
    }

    if (wp_sock != -1) {
        close(wp_sock);
        wp_sock = -1;
    }

    wp_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (wp_sock < 0) {
        perror("[waypoint_rx] socket");
        return;
    }

    unlink(wp_path);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, wp_path, sizeof(addr.sun_path) - 1);

    if (bind(wp_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[waypoint_rx] bind");
        close(wp_sock);
        wp_sock = -1;
        return;
    }

    if (set_nonblock(wp_sock) < 0) {
        perror("[waypoint_rx] nonblock");
    }

    fprintf(stderr, "[waypoint_rx] bind ok: %s\n", wp_path);
}

uint8_t RX_waypoint(to_vcp_msg_t* msg)
{
    if (msg == NULL) return 1;

    if (wp_sock == -1) {
        waypoint_rx_init(NULL);
        return 1;
    }

    WaypointPktV1 latest;
    int have = 0;
    int invalid = 0;

    for (;;) {
        WaypointPktV1 pkt;
        ssize_t n = recvfrom(wp_sock, &pkt, sizeof(pkt), 0, NULL, NULL);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            perror("[waypoint_rx] recvfrom");
            break;
        }
        if (n != (ssize_t)sizeof(pkt)) { invalid = 1; continue; }
        if (pkt.magic != WP_MAGIC) { invalid = 1; continue; }
        if (pkt_calc_crc(&pkt) != pkt.crc16) { invalid = 1; continue; }
        if (pkt.type != WP_TYPE_WAYPOINT) { invalid = 1; continue; }

        latest = pkt;
        have = 1;
    }

    if (invalid) {
        msg->wp_valid = 0;
    }

    const uint32_t now_ms = mono_ms();

    if (have) {
        wp_last_pkt = latest;
        wp_last_rx_ms = now_ms;
        wp_have_last = 1;
        msg->wp_valid = 1;
    }

    if (!wp_have_last) return 1;

    msg->leader_x_mm = wp_last_pkt.x_mm;
    msg->leader_y_mm = wp_last_pkt.y_mm;
    msg->leader_state = wp_last_pkt.exception;
    msg->wp_age_ms = (uint16_t)(now_ms - wp_last_rx_ms);

    return have ? 0 : 1;
}
