#include "speed.h"


static PDMModeConfig_t g_left_pwm_cfg;
static PDMModeConfig_t g_right_pwm_cfg;
static uint8 g_motor_initialized = 0;



static void motor_init(void)
{
    // 1. GPIO 설정 (왼쪽, 오른쪽 모터)
    GPIO_Config(MOTOR_LEFT_IN1, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(MOTOR_LEFT_IN2, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(MOTOR_RIGHT_IN3, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(MOTOR_RIGHT_IN4, (GPIO_FUNC(0) | GPIO_OUTPUT));

    // 2. PWM 기본 환경 설정 - 왼쪽 모터 (ENA)
    g_left_pwm_cfg.mcPortNumber = GPIO_PERICH_CH0;
    g_left_pwm_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    g_left_pwm_cfg.mcInversedSignal = 0;
    g_left_pwm_cfg.mcOutSignalInIdle = 0;
    g_left_pwm_cfg.mcLoopCount = 0;
    g_left_pwm_cfg.mcOutputCtrl = 0;
    g_left_pwm_cfg.mcPeriodNanoSec1 = PERIOD_NS;
    g_left_pwm_cfg.mcPeriodNanoSec2 = 0;

    // 3. PWM 기본 환경 설정 - 오른쪽 모터 (ENB)
    g_right_pwm_cfg.mcPortNumber = GPIO_PERICH_CH0;
    g_right_pwm_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    g_right_pwm_cfg.mcInversedSignal = 0;
    g_right_pwm_cfg.mcOutSignalInIdle = 0;
    g_right_pwm_cfg.mcLoopCount = 0;
    g_right_pwm_cfg.mcOutputCtrl = 0;
    g_right_pwm_cfg.mcPeriodNanoSec1 = PERIOD_NS;
    g_right_pwm_cfg.mcPeriodNanoSec2 = 0;


    g_motor_initialized = 1;

}


void control_motor_drive(uint32 cmd)
{
    if (!g_motor_initialized) {
        motor_init(); // 기존 초기화 함수 호출
    }

    uint32 target_duty = DUTY_STOP_NS;
    uint32 wait_cnt = 0;

    if(cmd == 0)
    {
        //전진
        GPIO_Set(MOTOR_LEFT_IN1, 1);
        GPIO_Set(MOTOR_LEFT_IN2, 0);
        GPIO_Set(MOTOR_RIGHT_IN3, 1);
        GPIO_Set(MOTOR_RIGHT_IN4, 0);
        target_duty = DUTY_SLOW_NS; // 30~50% 속도
    }
    else if(cmd == 2)
    {
        //후진
        GPIO_Set(MOTOR_LEFT_IN1, 0);
        GPIO_Set(MOTOR_LEFT_IN2, 1);
        GPIO_Set(MOTOR_RIGHT_IN3, 0);
        GPIO_Set(MOTOR_RIGHT_IN4, 1);
        target_duty = DUTY_SLOW_NS;
    }
    else
    {
        //정지 (입력이 없거나 다른 키일 때)
        GPIO_Set(MOTOR_LEFT_IN1, 0);
        GPIO_Set(MOTOR_LEFT_IN2, 0);
        GPIO_Set(MOTOR_RIGHT_IN3, 0);
        GPIO_Set(MOTOR_RIGHT_IN4, 0);
        target_duty = DUTY_STOP_NS;
    }
    

    // --- [왼쪽 모터 PWM] ---
    PDM_Disable(MOTOR_LEFT_ENA_CH, PMM_ON);
    wait_cnt = 0;
    while (PDM_GetChannelStatus(MOTOR_LEFT_ENA_CH) && ++wait_cnt < 1000);
    g_left_pwm_cfg.mcDutyNanoSec1 = target_duty;
    (void)PDM_SetConfig(MOTOR_LEFT_ENA_CH, &g_left_pwm_cfg);
    PDM_Enable(MOTOR_LEFT_ENA_CH, PMM_ON);

    // --- [오른쪽 모터 PWM] ---
    PDM_Disable(MOTOR_RIGHT_ENB_CH, PMM_ON);
    wait_cnt = 0;
    while (PDM_GetChannelStatus(MOTOR_RIGHT_ENB_CH) && ++wait_cnt < 1000);
    g_right_pwm_cfg.mcDutyNanoSec1 = target_duty;
    (void)PDM_SetConfig(MOTOR_RIGHT_ENB_CH, &g_right_pwm_cfg);
    PDM_Enable(MOTOR_RIGHT_ENB_CH, PMM_ON);

    
}

