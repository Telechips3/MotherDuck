#include "steer.h"
#include "../team2_header.h"

static PDMModeConfig_t g_steer_pwm_cfg;
static uint8 g_steer_initialized = 0;

// [핵심] 현재의 PWM 위치를 기억하는 정적 변수 (BSS 섹션에 위치)
static uint32 g_current_steer_ns = STEER_NEUTRAL_NS;

static void steer_init(void)
{
    //GPIO_Config(MOTOR_STEER_PIN, (GPIO_FUNC(1) | GPIO_OUTPUT)); // 조향 핀 설정 확인 필요

    g_steer_pwm_cfg.mcPortNumber = GPIO_PERICH_CH1;
    g_steer_pwm_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    g_steer_pwm_cfg.mcInversedSignal = 0;
    g_steer_pwm_cfg.mcOutSignalInIdle = 0;
    g_steer_pwm_cfg.mcLoopCount = 0;
    g_steer_pwm_cfg.mcOutputCtrl = 0;
    g_steer_pwm_cfg.mcPeriodNanoSec1 = STEER_PERIOD_NS;
    g_steer_pwm_cfg.mcPeriodNanoSec2 = 0;

    g_steer_initialized = 1;
}
void control_steering_absolute(int16_t x_norm_q15)
{
    if (!g_steer_initialized) steer_init();

    // x_norm_q15: -32768(최대 왼쪽) ~ 0(중앙) ~ +32767(최대 오른쪽)
    // PWM: STEER_MIN_NS(왼쪽) ~ STEER_NEUTRAL_NS(중앙) ~ STEER_MAX_NS(오른쪽)
    
    // 정규화된 값을 PWM 범위로 매핑
    int32_t range = (STEER_MAX_NS - STEER_MIN_NS) / 2;  // 편차 범위
    int32_t offset = (int32_t)x_norm_q15 * range / 32768;
    
    g_current_steer_ns = STEER_NEUTRAL_NS + offset;
    
    // 안전 범위 제한
    if (g_current_steer_ns < STEER_MIN_NS) g_current_steer_ns = STEER_MIN_NS;
    if (g_current_steer_ns > STEER_MAX_NS) g_current_steer_ns = STEER_MAX_NS;
    
    // PWM 업데이트 (기존 코드와 동일)
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
    
    mcu_printf("[STEER] x_norm=%d -> PWM=%d ns\n", x_norm_q15, g_current_steer_ns);
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