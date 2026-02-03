// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : main.c
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/

#define MCU_BSP_SUPPORT_APP_BASE 1

#if (MCU_BSP_SUPPORT_APP_BASE == 1)

#include <main.h>
#include "team2_header.h"
#include "speed.h"
#include "interrupt_example.h"
#include "encoder.h"
#include "spi.h"
#include "ultrasonic.h"
#include "buzzer.h"
#include "ipc.h"
#include "steer.h"

volatile uint32 g_ultraDistanceCm = 0;

#define I2C_CH_0 0
#define I2C_PORT_0 0
#define I2C_SPEED 100

#if (ENABLE_IMU_TEMP_TEST == 1)
#include "mpu_driver.h"
static MPU6050_Filter_t gMpuTest = {0};
#endif

#if (APLT_LINUX_SUPPORT_SPI_DEMO == 1)
#include <spi_eccp.h>
#endif
#if (APLT_LINUX_SUPPORT_POWER_CTRL == 1)
#include <power_app.h>
#endif
#if (MCU_BSP_SUPPORT_APP_KEY == 1)
#include <key.h>
#endif // ( MCU_BSP_SUPPORT_APP_KEY == 1 )

#if (MCU_BSP_SUPPORT_APP_CONSOLE == 1)
#include <console.h>
#endif // ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )

#if (MCU_BSP_SUPPORT_CAN_DEMO == 1)
#include <can_demo.h>
#endif // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#if (MCU_BSP_SUPPORT_APP_IDLE == 1)
#include <idle.h>
#endif // ( MCU_BSP_SUPPORT_APP_IDLE == 1 )

#if (MCU_BSP_SUPPORT_APP_SPI_LED == 1)
#include <spi_led.h>
#endif // ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )

#if (MCU_BSP_SUPPORT_APP_FW_UPDATE == 1)
#include "fwupdate.h"
#elif (MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP == 1)
#include "fwupdate.h"
#endif

/*
***************************************************************************************************
*                                         GLOBAL VARIABLES
***************************************************************************************************
*/
uint32 gALiveMsgOnOff;
static uint32 gALiveCount;

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void Main_StartTask(
    void *pArg);

static void AppTaskCreate(
    void);

static void DisplayAliveLog(
    void);

static void DisplayOTPInfo(
    void);

/*
***************************************************************************************************
*                                         FUNCTIONS
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                          cmain
*
* This is the standard entry point for C code.
*
* Notes
*   It is assumed that your code will call main() once you have performed all necessary
*   initialization.
*
***************************************************************************************************
*/
void cmain(void)
{
    static uint32 AppTaskStartID = 0;
    static uint32 AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    SALRetCode_t err;
    SALMcuVersionInfo_t versionInfo = {0, 0, 0, 0};

    (void)SAL_Init();

    BSP_PreInit(); /* Initialize basic BSP functions */

#if (MCU_BSP_SUPPORT_CAN_DEMO == 1)
    (void)CAN_DemoInitialize();
#endif // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

    BSP_Init(); /* Initialize BSP functions */

    (void)SAL_GetVersion(&versionInfo);
    mcu_printf("\n===============================\n");
    mcu_printf("    MCU BSP Version: V%d.%d.%d\n",
               versionInfo.viMajorVersion,
               versionInfo.viMinorVersion,
               versionInfo.viPatchVersion);
    mcu_printf("-------------------------------\n");
    DisplayOTPInfo();
    mcu_printf("===============================\n\n");

    // create the first app task...
    err = (SALRetCode_t)SAL_TaskCreate(&AppTaskStartID,
                                       (const uint8 *)"App Task Start",
                                       (SALTaskFunc)&Main_StartTask,
                                       &AppTaskStartStk[0],
                                       ACFG_TASK_MEDIUM_STK_SIZE,
                                       SAL_PRIO_APP_CFG,
                                       NULL);

    if (err == SAL_RET_SUCCESS)
    {
        // start woring os.... never return from this function
        (void)SAL_OsStart();
    }
}

/*
***************************************************************************************************
*                                          Main_StartTask
*
* This is an example of a startup task.
*
* Notes
*   As mentioned in the book's text, you MUST initialize the ticker only once multitasking has
*   started.
*
*   1) The first line of code is used to prevent a compiler warning because 'pArg' is not used.
*      The compiler should not generate any code for this statement.
*
***************************************************************************************************
*/
static void Main_StartTask(void *pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();

    PDM_Init(); // pwm 초기화 코드. 우리 코드가 아님.

#if (ENABLE_IPC_TEST == 1)
    ipc_init();
    mcu_printf(">>> ipc_init complete\n");
#endif

    /* --- 2. SPI 통신 설정 --- */
#if (ENABLE_SPI_TEST == 1)
    SPI_Init();
    mcu_printf(">>> SPI_Init complete\n");
#endif

#if (ENABLE_IMU_TEMP_TEST == 1)
    /* --- 리더님의 기존 테스트 로직 이식 --- */
    I2C_Init();
    I2C_Open(I2C_CH_0, I2C_PORT_0, I2C_SPEED, NULL, NULL);

    if (MPU6050_Init(I2C_CH_0, MPU6050_ADDR_7BIT) == 0)
    {
        mcu_printf("[SUCCESS] MPU6050 Detected!\n");

        // 3. MPU6050_Calibrate(채널, 주소, 구조체포인터) 형식으로 수정
        MPU6050_Calibrate(I2C_CH_0, MPU6050_ADDR_7BIT, &gMpuTest);

        while (1)
        {
            int16 ax, ay, az, gx, gy, gz;
            uint32 now;
            SAL_GetTickCount(&now);
            float dt = (gMpuTest.last_tick == 0) ? 0.02f : (float)(now - gMpuTest.last_tick) / 1000.0f;
            gMpuTest.last_tick = now;

            // 4. MPU6050_Read_Raw(채널, 주소, 데이터들...) 형식으로 수정
            if (MPU6050_Read_Raw(I2C_CH_0, MPU6050_ADDR_7BIT, &ax, &ay, &az, &gx, &gy, &gz) == 0)
            {
                MPU6050_Update_Filter(&gMpuTest, ax, ay, az, gx, gy, gz, dt);
                mcu_printf("ACC[%6d %6d %6d] | P:%4d R:%4d Y:%4d\r\n",
                           ax, ay, az, (int)gMpuTest.pitch, (int)gMpuTest.roll, (int)gMpuTest.yaw);
            }
            SAL_TaskSleep(20);
        }
    }
    else
    {
        mcu_printf("MPU6050 Detection Failed!\n");
    }
#endif

#if (ENABLE_ENCODER_TASK == 1)
    (void)EncoderTaskCreate();
#endif

#if (ENABLE_ULTRASONIC_TASK == 1)
    (void)UltrasonicTaskCreate();
#endif

#if (ENABLE_BUZZER_TASK == 1)
    (void)BuzzerTaskCreate();
#endif

    /* Main task는 여기서 끝 */
    while (1)
    {
        SAL_TaskSleep(5000);
    }
}

static void AppTaskCreate(void)
{
#if (APLT_LINUX_SUPPORT_SPI_DEMO == 1)
    ECCP_InitSPIManager();
#endif
#if (APLT_LINUX_SUPPORT_POWER_CTRL == 1)
    POWER_APP_StartDemo();
#endif

#if (MCU_BSP_SUPPORT_APP_CONSOLE == 1)
    CreateConsoleTask();
#endif // ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )

#if (MCU_BSP_SUPPORT_APP_KEY == 1)
    KEY_AppCreate();
#endif // ( MCU_BSP_SUPPORT_APP_KEY == 1 )

#if (MCU_BSP_SUPPORT_CAN_DEMO == 1)
    CAN_DemoCreateApp();
#endif // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#if (MCU_BSP_SUPPORT_APP_FW_UPDATE == 1)
    CreateFWUDTask();
#elif (MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP == 1)
    CreateFWUDTask();
#endif

#if (MCU_BSP_SUPPORT_APP_IDLE == 1)
    IDLE_CreateTask();
#endif // ( MCU_BSP_SUPPORT_APP_IDLE == 1 )

#if (MCU_BSP_SUPPORT_APP_SPI_LED == 1)
    SPILED_CreateAppTask();
#endif // ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )
}

static void DisplayAliveLog(void)
{
    if (gALiveMsgOnOff != 0U)
    {
        mcu_printf("\n %d", gALiveCount);

        gALiveCount++;

        if (gALiveCount >= MAIN_UINT_MAX_NUM)
        {
            gALiveCount = 0;
        }
    }
    else
    {
        gALiveCount = 0;
    }
}

#define LDT1_AREA_ADDR 0xA1011800U
#define PMU_REG_ADDR 0xA0F28000U

static void DisplayOTPInfo(void)
{
    volatile uint32 *ldt1Addr;
    volatile uint32 *chipNameAddr;
    volatile uint32 *remapAddr;
    volatile uint32 *hsmStatusAddr;
    uint32 chipName = 0;
    uint32 dualBankVal = 0;
    uint32 dual_bank = 0;
    uint32 expandFlashVal = 0;
    uint32 expand_flash = 0;
    uint32 remap_mode = 0;
    uint32 hsm_ready = 0;

    //----------------------------------------------------------------
    // OTP LDT1 Read
    // [11:0]Dual_Bank_Selection, [59:48]EXPAND_FLASH
    // Dual_Bank_Sel: [0xC0][11: 0] & [0xD0][11: 0] & [0xE0][11: 0] & [0xF0][11: 0]
    // EXPAND_FLASH : [0xC4][27:16] & [0xD4][27:16] & [0xE4][27:16] & [0xF4][27:16]
    // HwMC_PRG_FLS_LDT1: 0xA1011800

    ldt1Addr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x00C0);
    chipNameAddr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x0300);
    remapAddr = (volatile uint32 *)(PMU_REG_ADDR);
    hsmStatusAddr = (volatile uint32 *)(PMU_REG_ADDR + 0x0020);

    chipName = *chipNameAddr;
    chipName &= 0x000FFFFF;

    dualBankVal = ldt1Addr[0];
    expandFlashVal = ldt1Addr[1];

    dualBankVal &= ldt1Addr[4];
    expandFlashVal &= ldt1Addr[5];

    dualBankVal &= ldt1Addr[8];
    expandFlashVal &= ldt1Addr[9];

    dualBankVal &= ldt1Addr[12];
    expandFlashVal &= ldt1Addr[13];

    dualBankVal = (dualBankVal >> 0) & 0x0FFF;
    expandFlashVal = (expandFlashVal >> 16) & 0x0FFF;

    dual_bank = (dualBankVal == 0x0FFF) ? 0 : 1;       // (single_bank : dual_bank)
    expand_flash = (expandFlashVal == 0x0000) ? 0 : 1; // (only_eFlash : use_extSNOR)

    remap_mode = remapAddr[0];

    mcu_printf("    CHIP   NAME  : %x\n", chipName);
    mcu_printf("    DUAL   BANK  : %d\n", dual_bank);
    mcu_printf("    EXPAND FLASH : %d\n", expand_flash);
    mcu_printf("    REMAP  MODE  : %d\n", (remap_mode >> 16));

    hsm_ready = hsmStatusAddr[0];
    hsm_ready = (hsm_ready >> 2) & 0x0001;
#if 0
    if(hsm_ready)
    {
        mcu_printf("    HSM    READY : %d\n",    hsm_ready);
    }
    else
    {
        while(hsm_ready != 1)
        {
            mcu_printf("    HSM    READY : %d\n",    hsm_ready);
            mcu_printf("    wait...\n");
            hsm_ready = (hsm_ready >> 2) & 0x0001;
        }
    }
#else
    mcu_printf("    HSM    READY : %d\n", hsm_ready);
#endif
}

#endif // ( MCU_BSP_SUPPORT_APP_BASE == 1 )
