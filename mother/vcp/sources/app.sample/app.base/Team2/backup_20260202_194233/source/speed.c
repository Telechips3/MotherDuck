#include "speed.h"

static PDMModeConfig_t g_pwm_cfg_r; // RPWM 설정용
static PDMModeConfig_t g_pwm_cfg_l; // LPWM 설정용
static uint8 g_motor_initialized = 0;

static void motor_init()
{
    // 1. Enable 핀 설정 (High로 설정하여 드라이버 활성화)
    GPIO_Config(MOTOR_R_EN, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(MOTOR_L_EN, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Set(MOTOR_R_EN, 1);
    GPIO_Set(MOTOR_L_EN, 1);

    // 2. PWM 기본 설정 (두 채널 공통)
    g_pwm_cfg_r.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    g_pwm_cfg_r.mcPeriodNanoSec1 = PERIOD_NS;
    g_pwm_cfg_r.mcDutyNanoSec1 = 0; // 초기값 0

    g_pwm_cfg_l = g_pwm_cfg_r; // 동일하게 복사
    
    g_pwm_cfg_r.mcPortNumber = GPIO_PERICH_CH0; // RPWM용
    g_pwm_cfg_l.mcPortNumber = GPIO_PERICH_CH1; // LPWM용

    g_motor_initialized = 1;
}

void control_motor_drive(uint32 cmd)
{
    if (!g_motor_initialized) motor_init();

    uint32 target_duty_r = 0;
    uint32 target_duty_l = 0;

    // --- [BTS7960 로직 판단] ---
    if (cmd == 0) {         // 전진
        target_duty_r = DUTY_SLOW_NS;
        target_duty_l = 0;
    } 
    else if (cmd == 2) {    // 후진
        target_duty_r = 0;
        target_duty_l = DUTY_SLOW_NS;
    } 
    else {                  // 정지
        target_duty_r = 0;
        target_duty_l = 0;
    }

    // --- [PWM 업데이트: RPWM] ---
    PDM_Disable(MOTOR_RPWM_CH, PMM_ON);
    g_pwm_cfg_r.mcDutyNanoSec1 = target_duty_r;
    if (PDM_SetConfig(MOTOR_RPWM_CH, &g_pwm_cfg_r) == SAL_RET_SUCCESS) {
        PDM_Enable(MOTOR_RPWM_CH, PMM_ON);
    }

    // --- [PWM 업데이트: LPWM] ---
    PDM_Disable(MOTOR_LPWM_CH, PMM_ON);
    g_pwm_cfg_l.mcDutyNanoSec1 = target_duty_l;
    if (PDM_SetConfig(MOTOR_LPWM_CH, &g_pwm_cfg_l) == SAL_RET_SUCCESS) {
        PDM_Enable(MOTOR_LPWM_CH, PMM_ON);
    }
}