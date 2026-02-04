// proto.h
#pragma once
#include <stdint.h>
#include <stddef.h>

#define WP_MAGIC 0xA5

// type 예시 (원하는대로 확장)
enum {
    WP_TYPE_WAYPOINT = 1,
    WP_TYPE_HEARTBEAT = 2,
    WP_TYPE_CAM = 3,
};

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic;       // 0xA5
    uint16_t type;        // little-endian
    uint8_t  exception;   // bit flags 추천
    int32_t  x_mm;        // little-endian
    int32_t  y_mm;        // little-endian
    uint16_t crc16;       // little-endian (CRC field 제외하고 계산)
} WaypointPktV1;
#pragma pack(pop)

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(WaypointPktV1) == (1+2+1+4+4+2), "WaypointPktV1 size mismatch");
#endif

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

static inline uint16_t pkt_calc_crc(const WaypointPktV1* p)
{
    // crc16 필드 바로 앞까지
    return crc16_ccitt_false((const uint8_t*)p, offsetof(WaypointPktV1, crc16));
}