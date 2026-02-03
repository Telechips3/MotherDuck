#include "spi.h"
#include "speed.h"

static volatile uint32_t spi_rx_buf[1] = {0};
static uint32_t spi_tx_buf[1] = {0};
static uint32_t last_received_val = 0xFFFFFFFF;  // 이전 값 저장 (초기값 무효)

static void spi_receive(uint32 uiCh, uint32 iEvent, void *pArg)
{
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // STEP 1: 함수 진입 확인
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    mcu_printf("\n[STEP 1] === spi_receive ENTER ===\n");
    mcu_printf("         Channel: %d, Event: 0x%08X\n", uiCh, iEvent);
    
    // 목적: 인터럽트가 정상 호출되는지 확인
    // 예상: uiCh=0, iEvent는 완료 이벤트 플래그
    
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // STEP 2: 하드웨어 버퍼 읽기 전 상태 확인
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    mcu_printf("[STEP 2] Before read:\n");
    mcu_printf("         spi_rx_buf[0] = 0x%08X\n", spi_rx_buf[0]);
    mcu_printf("         &spi_rx_buf   = 0x%08X\n", (uint32_t)&spi_rx_buf[0]);
    
    // 목적: 하드웨어가 쓴 값 확인
    // 예상 정상: 0x00000000~0x00000003 (w,a,s,d)
    // 예상 비정상: 0xFFFFFFFF, 쓰레기 값, 또는 계속 0
    
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // STEP 3: 로컬 변수에 복사
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    uint32 received_val = spi_rx_buf[0];
    
    mcu_printf("[STEP 3] After read:\n");
    mcu_printf("         received_val  = 0x%08X (%d)\n", 
               received_val, received_val);
    mcu_printf("         &received_val = 0x%08X (stack)\n", 
               (uint32_t)&received_val);
    
    // 목적: 복사가 제대로 되었는지 확인
    // 예상: received_val == spi_rx_buf[0]
    
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // STEP 4: Re-arm 전 마지막 값 확인
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    mcu_printf("[STEP 4] Before GPSB_AsyncXfer:\n");
    mcu_printf("         last_received_val = 0x%08X (%d)\n", 
               last_received_val, last_received_val);
    
    // 목적: 이전 값 확인
    // 예상: 초기 0xFFFFFFFF, 이후 0~3
    
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // STEP 5: GPSB_AsyncXfer 호출
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    mcu_printf("[STEP 5] Calling GPSB_AsyncXfer...\n");
    
    (void)GPSB_AsyncXfer(SPI_CHANNEL, 
                         (uint32 *)spi_tx_buf, 
                         (uint32 *)spi_rx_buf, 
                         1, 
                         GPSB_XFER_MODE_WITH_INTERRUPT | 
                         GPSB_XFER_MODE_WITHOUT_CTF);
    
    mcu_printf("[STEP 6] GPSB_AsyncXfer completed\n");
    mcu_printf("         spi_rx_buf[0] = 0x%08X (after re-arm)\n", 
               spi_rx_buf[0]);
    
    // 목적: Re-arm 후 버퍼 상태 확인
    // 예상 정상: spi_rx_buf[0]는 그대로 또는 이미 다음 값
    // 예상 비정상: 0으로 클리어되거나 쓰레기 값
    
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // STEP 7: 유효성 검사
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    mcu_printf("[STEP 7] Condition check:\n");
    mcu_printf("         received_val < 4? %s (%d < 4)\n", 
               (received_val < 4) ? "TRUE" : "FALSE", received_val);
    mcu_printf("         received_val != last? %s (%d != %d)\n", 
               (received_val != last_received_val) ? "TRUE" : "FALSE",
               received_val, last_received_val);
    
    // 목적: 조건 분기 확인
    // 예상 정상: 둘 다 TRUE일 때만 처리
    // 예상 비정상: 
    //   - received_val >= 4 (노이즈)
    //   - received_val == last_received_val (중복)
    
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // STEP 8: 데이터 처리
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    if (received_val < 4 && received_val != last_received_val) {
        mcu_printf("[STEP 8] Processing data...\n");
        mcu_printf("         Calling control_motor_drive(%d)\n", received_val);
        
        control_motor_drive(received_val);
        
        mcu_printf("[STEP 9] Motor control completed\n");
        mcu_printf("         Updating last_received_val: %d -> %d\n", 
                   last_received_val, received_val);
        
        last_received_val = received_val;
        
        mcu_printf("[STEP 10] last_received_val updated = %d\n", 
                   last_received_val);
        
        // 목적: 처리 과정 추적
        // 예상 정상: control_motor_drive 호출, last_received_val 업데이트
        
    } else {
        mcu_printf("[STEP 8] Data IGNORED\n");
        
        if (received_val >= 4) {
            mcu_printf("         Reason: Invalid value (%d >= 4)\n", 
                       received_val);
        } else {
            mcu_printf("         Reason: Duplicate value (%d == %d)\n", 
                       received_val, last_received_val);
        }
        
        // 목적: 왜 무시되었는지 확인
        // 예상 정상: 중복 또는 노이즈
        // 예상 비정상: 유효한 값인데 무시됨
    }
    
    
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // STEP 11: 함수 종료
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    mcu_printf("[STEP 11] === spi_receive EXIT ===\n\n");
    
    // 목적: 함수 완료 확인
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
    
    // 첫 수신 대기는 초기화 시에만 한 번
    GPSB_AsyncXfer(SPI_CHANNEL, (uint32 *)spi_tx_buf, (uint32 *)spi_rx_buf, 1,
                   GPSB_XFER_MODE_WITH_INTERRUPT | GPSB_XFER_MODE_WITHOUT_CTF);
}
