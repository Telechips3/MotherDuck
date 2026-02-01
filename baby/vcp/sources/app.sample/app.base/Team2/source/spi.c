#include "spi.h"

uint8 tx[4] = { -1};
uint8 rx[4] = { -1};

void spi_receive(uint32 uiCh, uint32 iEvent, void *pArg)
{
    mcu_printf("[SPI] GPSB Receive %d %d\n", rx[0],GPSB_GetStatus(SPI_CHANNEL));
    GPSB_SetSlaveDMAMode(SPI_CHANNEL, tx, rx, 1);
}

void SPI_Init(void)
{
    GPIO_Config(SPI_CS_GPIO, GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    GPIO_Set(SPI_CS_GPIO, 1);
    
    GPIO_Config(SPI_SCLK_GPIO, GPIO_FUNC(SPI_GPIO_FUNC)| GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    GPIO_Config(SPI_MOSI_GPIO, GPIO_FUNC(SPI_GPIO_FUNC)| GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    GPIO_Config(SPI_MISO_GPIO, GPIO_FUNC(SPI_GPIO_FUNC)| GPIO_OUTPUT );
    
    GPSBOpenParam_t param = {
        .uiSdo = SPI_MOSI_GPIO,
        .uiSdi = SPI_MISO_GPIO,
        .uiSclk = SPI_SCLK_GPIO,
        .uiIsSlave = GPSB_SLAVE_MODE,
        .uiDmaBufSize = 1,
        .pDmaAddrTx = tx,
        .pDmaAddrRx = rx,
        .fbCallback = (GPSBCallback)(spi_receive),
        .pArg = NULL
    };

    if (GPSB_Open(SPI_CHANNEL, param) != SAL_RET_SUCCESS) {
        mcu_printf("[SPI] GPSB open failed\n");
        return;
    }

    GPSB_SetBpw(SPI_CHANNEL, 8);
    //GPSB_Init();
    GPSB_SetSlaveDMAMode(SPI_CHANNEL, tx, rx, 1);
}
