#include "steer.h"

static PDMModeConfig_t g_steer_pwm_cfg;
static uint8 g_steer_initialized = 0;

// [핵심] 현재의 PWM 위치를 기억하는 정적 변수 (BSS 섹션에 위치)
static uint32 g_current_steer_ns = STEER_NEUTRAL_NS;

static void steer_init(void)
{
    // GPIO는 PDM이 자동으로 설정하므로 별도 설정 불필요

    g_steer_pwm_cfg.mcPortNumber = GPIO_PERICH_CH0;
    g_steer_pwm_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    g_steer_pwm_cfg.mcInversedSignal = 0;
    g_steer_pwm_cfg.mcOutSignalInIdle = 0;
    g_steer_pwm_cfg.mcLoopCount = 0;
    g_steer_pwm_cfg.mcOutputCtrl = 0;
    g_steer_pwm_cfg.mcPeriodNanoSec1 = STEER_PERIOD_NS;
    g_steer_pwm_cfg.mcPeriodNanoSec2 = 0;
    g_steer_pwm_cfg.mcDutyNanoSec1 = STEER_NEUTRAL_NS;  // 초기 중립 위치

    // [핵심] PDM 채널 초기화 및 활성화
    if (PDM_SetConfig(STEER_PWM_CH, &g_steer_pwm_cfg) == SAL_RET_SUCCESS)
    {
        PDM_Enable(STEER_PWM_CH, PMM_ON);
        mcu_printf("[STEER] Initialized at neutral position\n");
    }
    else
    {
        mcu_printf("[STEER] Init FAILED!\n");
    }

    g_steer_initialized = 1;
}

void control_steering_step(uint32 cmd)
{
    if (!g_steer_initialized) steer_init();

    // --- [증감 로직] ---
    if (cmd == 1) // 예: 'a' 입력 (왼쪽으로 조향 깎기)
    {
        g_current_steer_ns -= STEER_STEP_NS;
    }
    else if (cmd == 3) // 예: 'd' 입력 (오른쪽으로 조향 더하기)
    {
        g_current_steer_ns += STEER_STEP_NS;
    }
    else // 예: 'c' 입력 (강제 정중앙 정렬)
    {
        g_current_steer_ns = STEER_NEUTRAL_NS;
    }

    // --- [안전 범위 제한 (Clamping)] ---
    if (g_current_steer_ns < STEER_MIN_NS) g_current_steer_ns = STEER_MIN_NS;
    if (g_current_steer_ns > STEER_MAX_NS) g_current_steer_ns = STEER_MAX_NS;

    // --- [PWM 업데이트] ---
    uint32 wait_cnt = 0;
    PDM_Disable(STEER_PWM_CH, PMM_ON);

    while (PDM_GetChannelStatus(STEER_PWM_CH))
    {
        for (volatile int i = 0; i < 500; i++); 
        if (++wait_cnt > 1000) break;
    }

    g_steer_pwm_cfg.mcDutyNanoSec1 = g_current_steer_ns;

    if (PDM_SetConfig(STEER_PWM_CH, &g_steer_pwm_cfg) == SAL_RET_SUCCESS)
    {
        PDM_Enable(STEER_PWM_CH, PMM_ON);
    }
    else
    {
        mcu_printf("Steer failed in control_motor_drive\n");
    }
}