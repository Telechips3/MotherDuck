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
    mcu_printf("[SPI] Interrupt received: Channel=%u Event=0x%08X\n", uiCh, iEvent);
     to_vcp_spi_msg_t* pkt = (to_vcp_spi_msg_t*)spi_rx_buf;
    
    if (pkt->magic == 0xA5) {
        mcu_printf("[SPI] Packet received: Magic=0x%02X Seq=%u\n", 
                   pkt->magic, pkt->vcp_msg.seq);
        
        // 전체 패킷을 큐에 삽입
        SALRetCode_t ret = SAL_QueuePut(g_motor_queue_id, (void *)pkt, 
                                         sizeof(to_vcp_spi_msg_t), 0, SAL_OPT_NON_BLOCKING);
        if (ret != SAL_RET_SUCCESS) {
            mcu_printf("[SPI] Queue full! Packet dropped.\n");
        }
    } else {
        mcu_printf("[SPI] Invalid magic: 0x%02X (expected 0xA5)\n", pkt->magic);
    }

    SAL_CoreCriticalEnter();
    spi_tx_buf[0] = s_encCnt;
    SAL_CoreCriticalExit();
    abcd(SPI_CHANNEL);
    GPSB_SetSlaveDMAMode(SPI_CHANNEL, (const void *)spi_tx_buf, (void *)spi_rx_buf, SPI_BYTE);
}

// #define SPI_PKT_SIZE_WORDS 8  // 32바이트 = 8 x uint32_t
// static volatile uint32_t spi_rx_buf[SPI_PKT_SIZE_WORDS] = {0};
// static uint32_t spi_tx_buf[SPI_PKT_SIZE_WORDS] = {0};

// static void spi_receive(uint32 uiCh, uint32 iEvent, void *pArg)
// {
//     // 패킷 시작 검증 (magic 바이트 확인)
//     to_vcp_spi_msg_t* pkt = (to_vcp_spi_msg_t*)spi_rx_buf;
    
//     if (pkt->magic == 0xA5) {
//         mcu_printf("[SPI] Packet received: Magic=0x%02X Seq=%u\n", 
//                    pkt->magic, pkt->vcp_msg.seq);
        
//         // 전체 패킷을 큐에 삽입
//         SALRetCode_t ret = SAL_QueuePut(g_motor_queue_id, (void *)pkt, 
//                                          sizeof(to_vcp_spi_msg_t), 0, SAL_OPT_NON_BLOCKING);
//         if (ret != SAL_RET_SUCCESS) {
//             mcu_printf("[SPI] Queue full! Packet dropped.\n");
//         }
//     } else {
//         mcu_printf("[SPI] Invalid magic: 0x%02X (expected 0xA5)\n", pkt->magic);
//     }
    
//     // 다음 수신 준비 (전체 패킷 크기로 수신)
//     GPSB_AsyncXfer(SPI_CHANNEL, (uint32 *)spi_tx_buf, (uint32 *)spi_rx_buf, SPI_PKT_SIZE_WORDS,
//                    GPSB_XFER_MODE_WITH_INTERRUPT | GPSB_XFER_MODE_WITHOUT_CTF);
// }

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
