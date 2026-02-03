#include "spi.h"
#include "speed.h"
#include "ipc.h"
#include "encoder.h"

static volatile uint32_t spi_rx_buf[1] = {0};
static uint32_t spi_tx_buf[1] = {0};

static void spi_receive(uint32 uiCh, uint32 iEvent, void *pArg)
{
    mcu_printf("[SPI] PIO Received: %d\n", spi_rx_buf[0]);
    
    (void)SAL_QueuePut(g_motor_queue_id, (void *)&spi_rx_buf[0], sizeof(uint32), 0, SAL_OPT_NON_BLOCKING);
    
    SAL_CoreCriticalEnter();
    spi_tx_buf[0] = g_enc_count;
    SAL_CoreCriticalExit();
    
    GPSB_AsyncXfer(SPI_CHANNEL, (uint32 *)spi_tx_buf, (uint32 *)spi_rx_buf, 1,
                   GPSB_XFER_MODE_WITH_INTERRUPT | GPSB_XFER_MODE_WITHOUT_CTF);
}

uint32_t spi_send(void* arg)
{
    uint32_t ret = -1;

    return ret;
}

void SPI_Init(void)
{
    GPIO_Config(SPI_CS_GPIO, GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    GPIO_Set(SPI_CS_GPIO, 1);

    // GPIO_Config(SPI_SCLK_GPIO, GPIO_FUNC(SPI_GPIO_FUNC) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    // GPIO_Config(SPI_MOSI_GPIO, GPIO_FUNC(SPI_GPIO_FUNC) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    // GPIO_Config(SPI_MISO_GPIO, GPIO_FUNC(SPI_GPIO_FUNC) | GPIO_OUTPUT);

    GPSBOpenParam_t param = {
        .uiSdo = SPI_MOSI_GPIO,
        .uiSdi = SPI_MISO_GPIO,
        .uiSclk = SPI_SCLK_GPIO,
        .uiIsSlave = GPSB_SLAVE_MODE,
        .uiDmaBufSize = 0,
        .pDmaAddrTx = NULL,
        .pDmaAddrRx = NULL,
        .fbCallback = (GPSBCallback)(spi_receive),
        .pArg = NULL};

    if (GPSB_Open(SPI_CHANNEL, param) != SAL_RET_SUCCESS)
    {
        mcu_printf("[SPI] GPSB open failed\n");
        return;
    }

    GPSB_Init();
    GPSB_SetBpw(SPI_CHANNEL, 8);
    GPSB_AsyncXfer(SPI_CHANNEL, (uint32 *)spi_tx_buf, (uint32 *)spi_rx_buf, 1,
                   GPSB_XFER_MODE_WITH_INTERRUPT | GPSB_XFER_MODE_WITHOUT_CTF);
}