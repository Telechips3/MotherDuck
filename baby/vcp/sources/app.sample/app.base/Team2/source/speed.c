#include "speed.h"

static PDMModeConfig_t g_pwm_cfg;
static uint8 g_motor_initialized = 0;

static void motor_init()
{
    // 1. GPIO 설정
    GPIO_Config(MOTOR_IN1, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(MOTOR_IN2, (GPIO_FUNC(0) | GPIO_OUTPUT));

    // 2. PWM 기본 환경 설정
    g_pwm_cfg.mcPortNumber = GPIO_PERICH_CH0;
    g_pwm_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    g_pwm_cfg.mcInversedSignal = 0;
    g_pwm_cfg.mcOutSignalInIdle = 0;
    g_pwm_cfg.mcLoopCount = 0;
    g_pwm_cfg.mcOutputCtrl = 0;
    g_pwm_cfg.mcPeriodNanoSec1 = PERIOD_NS;
    g_pwm_cfg.mcPeriodNanoSec2 = 0;

    g_motor_initialized = 1;
}

// 핵심 제어 함수: 외부에서 받은 cmd('w', 's' 등)에 따라 동작
void control_motor_drive(uint32 cmd)
{
    if (!g_motor_initialized)
        motor_init();

    uint32 target_duty = DUTY_STOP_NS;
    uint32 wait_cnt = 0;

    // --- [입력에 따른 로직 판단] ---
    if (cmd == 0)
    { // 전진
        GPIO_Set(MOTOR_IN1, 1);
        GPIO_Set(MOTOR_IN2, 0);
        target_duty = DUTY_SLOW_NS; // 30~50% 속도
    }
    else if (cmd == 2)
    { // 후진
        GPIO_Set(MOTOR_IN1, 0);
        GPIO_Set(MOTOR_IN2, 1);
        target_duty = DUTY_SLOW_NS;
    }
    else
    { // 정지 (입력이 없거나 다른 키일 때)
        GPIO_Set(MOTOR_IN1, 0);
        GPIO_Set(MOTOR_IN2, 0);
        target_duty = DUTY_STOP_NS;
    }
    

    // --- [PWM 업데이트] ---
    // TCC70xx의 PDM은 설정을 바꿀 때 Disable -> Config -> Enable 과정을 거쳐야 안전합니다.
    PDM_Disable(MOTOR_PWM_CH, PMM_ON);

    while (PDM_GetChannelStatus(MOTOR_PWM_CH))
    {
        wait_cnt = 0;
        while (PDM_GetChannelStatus(MOTOR_PWM_CH))
        {
            // SAL_TaskSleep(1); <- 이걸 지우세요!
            for (volatile int i = 0; i < 1000; i++)
                ; // 단순 루프로 아주 짧게 대기

            if (++wait_cnt > 1000)
                break; // 카운트를 좀 더 늘려주세요
        }
    }

    g_pwm_cfg.mcDutyNanoSec1 = target_duty;

    if (PDM_SetConfig(MOTOR_PWM_CH, &g_pwm_cfg) == SAL_RET_SUCCESS)
    {
        PDM_Enable(MOTOR_PWM_CH, PMM_ON);
    }
}