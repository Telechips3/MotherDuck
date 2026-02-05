#include "../include/spi.h"
#include "speed.h"
#include "ipc.h"
#include "encoder.h"
#include "../team2_header.h"

static uint32 spi_rx_buf[SPI_BYTE] = {0};
static uint32 spi_tx_buf[SPI_BYTE] = {100};

static uint32 spi_dma_rx_buf[SPI_DMA_BYTE] = {0};
static uint32 spi_dma_tx_buf[SPI_DMA_BYTE] = {0};

static void spi_receive(uint32 uiCh, uint32 iEvent, void *pArg)
{
    mcu_printf("[SPI] PIO Received: %d\n", spi_rx_buf[0]);

    (void)SAL_QueuePut(g_motor_queue_id, (void *)&spi_rx_buf[0], sizeof(uint32), 0, SAL_OPT_NON_BLOCKING);

    int x = 123456;
    SAL_CoreCriticalEnter();
    //x = 123456;
    //spi_tx_buf[0] = spi_rx_buf[0];
    SAL_CoreCriticalExit();
    abcd(SPI_CHANNEL);
    spi_tx_buf[0] = x;
    GPSB_SetSlaveDMAMode(SPI_CHANNEL, (const void *)spi_tx_buf, (void *)spi_rx_buf, SPI_BYTE);
}

void SPI_Init(void)
{
    GPSBOpenParam_t param = {
        .uiSdo = SPI_MOSI_GPIO,
        .uiSdi = SPI_MISO_GPIO,
        .uiSclk = SPI_SCLK_GPIO,
        .uiIsSlave = GPSB_SLAVE_MODE,
        .uiDmaBufSize = SPI_DMA_BYTE * sizeof(uint32),
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