#include "../include/spi.h"
#include "../team2_header.h"
#include "speed.h"
#include "ipc.h"
#include "encoder.h"

static uint32_t spi_rx_buf[SPI_BYTE] = {0};
static uint32_t spi_tx_buf[SPI_BYTE] = {0};

static uint32_t spi_dma_rx_buf[SPI_DMA_BYTE] = {0};
static uint32_t spi_dma_tx_buf[SPI_DMA_BYTE] = {0};

void Dump_Vcp_Hex(to_vcp_spi_msg_t* m)
{
    uint8_t* p = (uint8_t*)m;
    int size = (int)sizeof(to_vcp_spi_msg_t);

    mcu_printf("[SPI HEX DUMP] Size: %d\n", size);

    for (int i = 0; i < size; i++)
    {
        // %02x: 2자리 확보하고 빈 곳은 0으로 채움 (0xA5 -> a5, 0x07 -> 07)
        // 사용자님의 DBG_Printfi 내부 'fill'과 'fminString' 로직을 활용합니다.
        mcu_printf("%02x ", (int)p[i]);

        // 8바이트마다 줄바꿈해서 보기 편하게 출력
        if ((i + 1) % 8 == 0) {
            mcu_printf("\n");
        }
    }
    mcu_printf("\n----------------------\n");
}

static void spi_receive(uint32 uiCh, uint32 iEvent, void *pArg)
{
    mcu_printf("[SPI] Interrupt received: Channel=%d Event=0x%08X\n", uiCh, iEvent);
    to_vcp_spi_msg_t *m = (to_vcp_spi_msg_t *)spi_rx_buf;

    Dump_Vcp_Hex(m);

    size_t payload_len = sizeof(to_vcp_spi_msg_t) - sizeof(uint16_t);
    uint16_t calculated_crc = crc16_ccitt_false((uint8_t *)m, payload_len);

    if (m->magic == 165 && m->crc16 == calculated_crc) // 0xA5
    {
        SAL_QueuePut(g_motor_queue_id, (void *)m, sizeof(to_vcp_spi_msg_t), 0, SAL_OPT_NON_BLOCKING);
    }
    else
    {
        mcu_printf("[SPI] Invalid packet: Magic=0x%X (expected 0xA5), CRC=0x%X (expected 0x%X)\n",
                   m->magic, m->crc16, calculated_crc);
    }

    SAL_CoreCriticalEnter();
    spi_tx_buf[0] = s_encCnt;
    SAL_CoreCriticalExit();
    abcd(SPI_CHANNEL);
    GPSB_SetSlaveDMAMode(SPI_CHANNEL, (const void *)spi_tx_buf, (void *)spi_rx_buf, SPI_BYTE);
}

void SPI_Init(void)
{
    GPIO_Config(SPI_CS_GPIO, GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    GPIO_Set(SPI_CS_GPIO, 1);

    GPSBOpenParam_t param = {
        .uiSdo = SPI_MOSI_GPIO,
        .uiSdi = SPI_MISO_GPIO,
        .uiSclk = SPI_SCLK_GPIO,
        .uiIsSlave = GPSB_SLAVE_MODE,
        .uiDmaBufSize = SPI_DMA_BYTE,
        .pDmaAddrTx = spi_dma_tx_buf,
        .pDmaAddrRx = spi_dma_rx_buf,
        .fbCallback = (GPSBCallback)(spi_receive),
        .pArg = NULL};

    if (GPSB_Open(SPI_CHANNEL, param) != SAL_RET_SUCCESS)
    {
        mcu_printf("[SPI] GPSB open failed\n");
        return;
    }

    GPSB_Init();
    GPSB_SetBpw(SPI_CHANNEL, 8);

    GPSB_SetSlaveDMAMode(SPI_CHANNEL, (const void *)spi_tx_buf, (void *)spi_rx_buf, SPI_BYTE);
    GPIO_Config(SPI_CS_GPIO, GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    GPIO_Set(SPI_CS_GPIO, 1);
}
