#ifndef _SPI_H_  // 중복 포함 방지 시작
#define _SPI_H_

#include "team2_header.h"

#define SPI_CHANNEL     0
#define SPI_CS_GPIO     GPIO_GPB(5)
#define SPI_SCLK_GPIO   GPIO_GPB(4)
#define SPI_MOSI_GPIO   GPIO_GPB(6)
#define SPI_MISO_GPIO   GPIO_GPB(7)
#define SPI_GPIO_FUNC   1

void SPI_Init(void);

#endif 