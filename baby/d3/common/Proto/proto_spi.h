#include <stdint.h>
#include <decide_mode.h>
// proto_spi.h
// 32-bit 정렬된 SPI용 메시지 정의

#pragma pack(push, 1)
typedef struct {
  uint8_t magic;         // 0xA5
  
  to_vcp_msg_t vcp_msg;  // 내부에 기존 메시지 포함, 29 byte

  uint16_t crc16;         // little-endian (CRC field 제외하고 계산)
} to_vcp_spi_msg_t;
#pragma pack(pop)

