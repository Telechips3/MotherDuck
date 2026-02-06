#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include "ArUco_RX.h"

#define SOCKET_PATH "/tmp/aruco_socket"
#define IMG_HALF_WIDTH 320.0f // Python의 640px 기준 절반

// ArUco 마커의 원시 패킷 구조체 (Python 쪽과 동일하게 맞춤)
struct ArucoRawPacket {
    int32_t id;
    float dist_cm;
    float error_px;
    char steering;
} __attribute__((packed));

static int aruco_sock = -1;
static struct ArucoRawPacket aruco_last_pkt;
static uint32_t aruco_last_rx_ms = 0;
static int aruco_have_last = 0;

static uint32_t mono_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ull + (uint64_t)(ts.tv_nsec / 1000000ull));
}

static void fill_msg_from_pkt(to_vcp_msg_t* msg, const struct ArucoRawPacket* pkt, uint32_t age_ms)
{
    // 1. 거리 변환: Python(cm) -> C(mm)
    msg->aruco_dist_mm = (int16_t)(pkt->dist_cm * 10.0f);

    // 2. 오차 변환: Pixel(-320~320) -> Q15(-32768~32767)
    float norm = pkt->error_px / IMG_HALF_WIDTH;
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;
    msg->aruco_x_norm_q15 = (int16_t)(norm * 32767.0f);

    msg->aruco_age_ms = (uint16_t)age_ms;
}

// 소켓 초기화 및 비차단 설정
void aruco_rx_init(void) {
    if (aruco_sock != -1) close(aruco_sock);

    aruco_sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (aruco_sock < 0) return;

    // 비차단 모드 설정 (데이터 없어도 read에서 무한 대기 안 함)
    int flags = fcntl(aruco_sock, F_GETFL, 0);
    fcntl(aruco_sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

    // 비차단 연결 시도
    connect(aruco_sock, (struct sockaddr *)&addr, sizeof(addr));
}

uint8_t RX_aruco_marker(to_vcp_msg_t *msg) {
    if (aruco_sock == -1) {
        aruco_rx_init();
        return 1; // 연결 시도 중이므로 데이터 없음 리턴
    }

    struct ArucoRawPacket packet;
    ssize_t len = read(aruco_sock, &packet, sizeof(packet));

    if (len == sizeof(packet)) {
        aruco_last_pkt = packet;
        aruco_last_rx_ms = mono_ms();
        aruco_have_last = 1;
        msg->aruco_valid = 1;
        fill_msg_from_pkt(msg, &aruco_last_pkt, 0);
        return 0; // 신규 데이터 수신
    }

    // 소켓 에러 또는 서버 종료 시 소켓 닫기
    if (len == 0 || (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
        close(aruco_sock);
        aruco_sock = -1;
    }

    // 잘못된 패킷(길이 불일치)만 invalid 처리
    if (len > 0 && len != (ssize_t)sizeof(packet)) {
        msg->aruco_valid = 0;
    }

    if (!aruco_have_last) return 1;

    fill_msg_from_pkt(msg, &aruco_last_pkt, mono_ms() - aruco_last_rx_ms);

    return 1; // 신규 데이터 없음
}
