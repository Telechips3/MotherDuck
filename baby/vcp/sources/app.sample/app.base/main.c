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

#include "main.h"
#include "team2_header.h"
#include "speed.h"
#include "interrupt_example.h"
#include "encoder.h"
#include "spi.h"
#include "ultrasonic.h"
#include "ipc.h"
#include "steer.h"
#include "imu.h"
#include "pose.h"
#include "pose_task.h"
#include "sensor_task.h"
#include "follow_steer_module.h"

#define ENABLE_IPC_TEST             1
#define ENABLE_SPI_TEST             1          
#define ENABLE_IMU_TASK             0       
#define ENABLE_ENCODER_TASK         0
#define ENABLE_ULTRASONIC_TASK      0   
#define ENABLE_POSE_TASK            0
#define ENABLE_SENSOR_TASK          1
#define ENABLE_FOLLOW_STEER_TASK    0

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

static void SemaTask(void *pArg);

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
    
    PDM_Init(); // pwm 초기화 코드. 우리 코드가 아님.
    void Buzzer_Init(void);
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
            mcu_printf(">>> PDM_Init complete\n");
    // Debug: force motor forward (remove after test)

#if (ENABLE_IPC_TEST == 1)
    ipc_init();
    mcu_printf(">>> ipc_init complete\n");
#else
    mcu_printf(">>>ipc not enabled\n");
#endif

    /* --- 2. SPI 통신 설정 --- */
#if (ENABLE_SPI_TEST == 1)
    SPI_Init(); //이 친구는 task가 아닙니다. main task에 기생해서 사는 친구입니다. 밑에 while문을 살려주시죠.
    mcu_printf(">>> SPI_Init complete\n");
#endif
    mcu_printf(">>>spi\n");
#if (ENABLE_IMU_TASK == 1)
    (void)IMUTaskCreate();
    mcu_printf(">>> IMU_task_init complete\n");
#endif
#if (ENABLE_POSE_TASK == 1)
    (void)PoseTaskCreate();

    // Debug: fixed yaw override (30 deg) for pose
    Pose_DebugSetYawOverride(30.0f * 3.1415926535f / 180.0f, 1);
    
    mcu_printf(">>>pose enabled\n");
#endif
#if (ENABLE_ENCODER_TASK == 1)
    (void)EncoderTaskCreate();
    mcu_printf(">>>encoder enabled\n");
#endif
#if (ENABLE_SENSOR_TASK == 1)
    (void)SensorTaskCreate();
    mcu_printf(">>>sensor task enabled\n");
#endif
#if (ENABLE_ULTRASONIC_TASK == 1)
    (void)UltrasonicTaskCreate();
#endif

#if (ENABLE_FOLLOW_STEER_TASK == 1)
    (void)follow_steer_TaskCreate();
    mcu_printf(">>>follow_steer enabled\n");
#endif
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
{(void)pArg;
    (void)SAL_OsInitFuncs();
    /* Main task는 여기서 끝 */
    while (1)
    {
        SAL_TaskSleep(1);
    }
}
static void SemaTask(void *pArg) {
    (void)pArg;
    
    mcu_printf("[EmergencyTask] Task Started & Waiting...\n");

    while (1) {
        // 1. 평소에는 여기서 무한 대기 (CPU 안 씀)
        // 누군가 Trigger_Emergency_Stop()을 호출할 때까지 영원히 잠듦
        if (SAL_SemaphoreWait(sem_ultra, 0, SAL_OPT_BLOCKING) == SAL_RET_SUCCESS) {
            
            // 2. 세마포어를 받았다! (비상 상황 발생)
            mcu_printf("\n\n!!! [EMERGENCY] STOP TRIGGERED !!!\n\n");

            // 3. 즉각적인 하드웨어 정지 조치
            // (1) 모터 전원 차단 (PWM 0)
            //control_motor_drive(0);        // 드라이브 모드 정지
            control_motor_manual(0.0f, 0); // 매뉴얼 모드 강제 0
            
            // (2) 부저 울림
            Buzzer_Set(1);

            // (3) 적분 제어기 초기화 (다시 출발할 때 급발진 방지)
            // (speed.c의 전역변수를 직접 건드리기 어려우니 정지 함수가 해줘야 함)
            
            // 4. 후속 처리 (선택 사항)
            // 여기서 바로 다시 루프를 돌면 또 대기 상태로 감.
            // 만약 "리셋 버튼 누를 때까지 정지" 하려면 여기서 또 다른 로직이 필요함.
            
            // 일단은 계속 정지 상태를 유지하기 위해 짧은 딜레이나 반복 정지 명령
            for(int i=0; i<10; i++) {
                 control_motor_drive(0);
                 SAL_TaskSleep(100);
             } // 1초간 확실하게 제압
             Buzzer_Set(0);
             (void)SAL_SemaphoreRelease(sem_ultra);
            
            
        }
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
