#ifndef _SPI_H_  // 중복 포함 방지 시작
#define _SPI_H_
#include "../team2_header.h"

#define SPI_CHANNEL     0
#define SPI_GPIO_FUNC   1
#define SPI_BYTE        32
#define SPI_DMA_BYTE    32
void Dump_Vcp_Hex(to_vcp_spi_msg_t* m);
void SPI_Init(void);

#endif 