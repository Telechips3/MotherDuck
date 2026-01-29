
#include "speed.h"

void speed_pwm()
{
    /* Service Init*/
    PDMModeConfig_t pwm_cfg;
    uint32 wait_cnt;

    // 1. 초기화 (PDM 및 GPIO)
    PDM_Init();
    
    // 방향 핀(GPB1)을 일반 출력 모드로 설정
    GPIO_Config(MOTOR_IN1, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(MOTOR_IN2, (GPIO_FUNC(0) | GPIO_OUTPUT));

    // 2. PDM(PWM) 기본 환경 설정
    pwm_cfg.mcPortNumber      = GPIO_PERICH_CH0;  // GPIO A10 매핑
    pwm_cfg.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
    pwm_cfg.mcInversedSignal  = 0;
    pwm_cfg.mcOutSignalInIdle = 0;
    pwm_cfg.mcLoopCount       = 0;
    pwm_cfg.mcOutputCtrl      = 0;
    pwm_cfg.mcPeriodNanoSec1  = PERIOD_NS;
    pwm_cfg.mcPeriodNanoSec2  = 0;

    // --- [주행 시퀀스 시작] ---

    // A. 방향 설정 (1: 전진, 0: 후진 - 드라이버에 따라 다름)
    GPIO_Set(MOTOR_IN1, 1);
    GPIO_Set(MOTOR_IN2, 0);

    // B. 속도 설정 (30% 출력으로 시작)
    pwm_cfg.mcDutyNanoSec1 = DUTY_SLOW_NS;
    
    PDM_Disable(MOTOR_PWM_CH, PMM_ON);
    if (PDM_SetConfig(MOTOR_PWM_CH, &pwm_cfg) == SAL_RET_SUCCESS) {
        PDM_Enable(MOTOR_PWM_CH, PMM_ON);
        mcu_printf("Drive Start: 30%% Speed for 3 seconds...\n");
    }

    // C. 3초 대기 (SAL 전용 슬립)
    SAL_TaskSleep(5000);

    // --- [주행 종료] ---

    // D. 모터 정지 (Duty를 0으로 변경)
    GPIO_Set(MOTOR_IN1, 0);
    GPIO_Set(MOTOR_IN2, 0);

    pwm_cfg.mcDutyNanoSec1 = DUTY_STOP_NS;
    
    PDM_Disable(MOTOR_PWM_CH, PMM_ON);
    
    // 채널이 완전히 꺼질 때까지 상태 체크
    wait_cnt = 0;
    while (PDM_GetChannelStatus(MOTOR_PWM_CH)) {
        SAL_TaskSleep(1);
        if (++wait_cnt > 100) break;
    }

    if (PDM_SetConfig(MOTOR_PWM_CH, &pwm_cfg) == SAL_RET_SUCCESS) {
        PDM_Enable(MOTOR_PWM_CH, PMM_ON);
        mcu_printf("Drive Stop: Sequence Finished.\n");
    }

    // 작업 완료 후 무한 대기
    while (1) {
        SAL_TaskSleep(1000);
    }
}