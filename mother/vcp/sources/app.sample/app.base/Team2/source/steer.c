#include "steer.h"

static PDMModeConfig_t g_steer_pwm_cfg;
static uint8 g_steer_initialized = 0;
static uint32 g_current_steer_ns = STEER_NEUTRAL_NS;

static void steer_init(void)
{
    g_steer_pwm_cfg.mcPortNumber = GPIO_PERICH_CH0;
    g_steer_pwm_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    g_steer_pwm_cfg.mcInversedSignal = 0;
    g_steer_pwm_cfg.mcOutSignalInIdle = 0;
    g_steer_pwm_cfg.mcLoopCount = 0;
    g_steer_pwm_cfg.mcOutputCtrl = 0;
    g_steer_pwm_cfg.mcPeriodNanoSec1 = STEER_PERIOD_NS;
    g_steer_pwm_cfg.mcPeriodNanoSec2 = 0;
    g_steer_pwm_cfg.mcDutyNanoSec1 = STEER_NEUTRAL_NS;

    if (PDM_SetConfig(STEER_PWM_CH, &g_steer_pwm_cfg) == SAL_RET_SUCCESS)
    {
        PDM_Enable(STEER_PWM_CH, PMM_ON);
        mcu_printf("[STEER] Initialized\n");
    }

    g_steer_initialized = 1;
}

void control_steering_step(uint32 cmd)
{
    if (!g_steer_initialized) steer_init();

    // 조합 명령 처리
    switch(cmd) {
        case CMD_LEFT:  // 'a' 단독
            g_current_steer_ns -= STEER_STEP_NS;
            mcu_printf("[조향] 좌\n");
            break;
            
        case CMD_RIGHT:  // 'd' 단독
            g_current_steer_ns += STEER_STEP_NS;
            mcu_printf("[조향] 우\n");
            break;
            
        case CMD_FORWARD_LEFT:  // W+A
        case CMD_BACKWARD_LEFT:  // S+A
            g_current_steer_ns -= STEER_STEP_NS;
            mcu_printf("[조향] 좌회전 중\n");
            break;
            
        case CMD_FORWARD_RIGHT:  // W+D
        case CMD_BACKWARD_RIGHT:  // S+D
            g_current_steer_ns += STEER_STEP_NS;
            mcu_printf("[조향] 우회전 중\n");
            break;
            
        default:  // 중립 복귀
            g_current_steer_ns = STEER_NEUTRAL_NS;
            break;
    }

    // 범위 제한
    if (g_current_steer_ns < STEER_MIN_NS) g_current_steer_ns = STEER_MIN_NS;
    if (g_current_steer_ns > STEER_MAX_NS) g_current_steer_ns = STEER_MAX_NS;

    // PWM 업데이트
    uint32 wait_cnt = 0;
    PDM_Disable(STEER_PWM_CH, PMM_ON);
    while (PDM_GetChannelStatus(STEER_PWM_CH) && ++wait_cnt < 1000);

    g_steer_pwm_cfg.mcDutyNanoSec1 = g_current_steer_ns;
    if (PDM_SetConfig(STEER_PWM_CH, &g_steer_pwm_cfg) == SAL_RET_SUCCESS)
    {
        PDM_Enable(STEER_PWM_CH, PMM_ON);
    }
}