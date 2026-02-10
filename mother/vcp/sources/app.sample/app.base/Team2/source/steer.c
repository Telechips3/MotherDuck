#include "../include/steer.h"

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

// void control_steering_step(uint32 cmd)
// {
//     if (!g_steer_initialized) steer_init();

//     // 조합 명령 처리
//     switch(cmd) {
//         case CMD_LEFT:  // 'a' 단독
//             g_current_steer_ns -= STEER_STEP_NS;
//             mcu_printf("[조향] 좌\n");
//             break;
            
//         case CMD_RIGHT:  // 'd' 단독
//             g_current_steer_ns += STEER_STEP_NS;
//             mcu_printf("[조향] 우\n");
//             break;
            
//         case CMD_FORWARD_LEFT:  // W+A
//         case CMD_BACKWARD_LEFT:  // S+A
//             g_current_steer_ns -= STEER_STEP_NS;
//             mcu_printf("[조향] 좌회전 중\n");
//             break;
            
//         case CMD_FORWARD_RIGHT:  // W+D
//         case CMD_BACKWARD_RIGHT:  // S+D
//             g_current_steer_ns += STEER_STEP_NS;
//             mcu_printf("[조향] 우회전 중\n");
//             break;
            
//         default:  // 중립 복귀
//             g_current_steer_ns = STEER_NEUTRAL_NS;
//             break;
//     }

//     mcu_printf("[조향] 현재 펄스: %d ns\n", g_current_steer_ns);

//     // 범위 제한
//     if (g_current_steer_ns < STEER_MIN_NS) g_current_steer_ns = STEER_MIN_NS;
//     if (g_current_steer_ns > STEER_MAX_NS) g_current_steer_ns = STEER_MAX_NS;

//     // PWM 업데이트
//     uint32 wait_cnt = 0;
//     PDM_Disable(STEER_PWM_CH, PMM_ON);
//     while (PDM_GetChannelStatus(STEER_PWM_CH) && ++wait_cnt < 1000);

//     g_steer_pwm_cfg.mcDutyNanoSec1 = g_current_steer_ns;
//     if (PDM_SetConfig(STEER_PWM_CH, &g_steer_pwm_cfg) == SAL_RET_SUCCESS)
//     {
//         PDM_Enable(STEER_PWM_CH, PMM_ON);
//     }
// }

void control_steer_with_ros(float angular_vel)
{
    if (!g_steer_initialized) steer_init();

    int32 target_pulse = STEER_NEUTRAL_NS;
    
    // 1. 최대 각속도 제한 (Input Clamping)
    if (angular_vel > MAX_ANGULAR_VEL) angular_vel = MAX_ANGULAR_VEL;
    if (angular_vel < -MAX_ANGULAR_VEL) angular_vel = -MAX_ANGULAR_VEL;

    // 2. 매핑 계산
    // 공식: 목표펄스 = 중립 + ( (입력각속도 / 최대각속도) * 가용범위 )
    // float 연산 후 int로 변환
    float pwm_offset = (angular_vel / MAX_ANGULAR_VEL) * (float)STEER_RANGE_NS;
    
    // 3. 방향 설정 (중요!!)
    // ROS 표준: +가 좌회전, -가 우회전
    // 서보 모터가 펄스가 커질 때 왼쪽으로 가는지 오른쪽으로 가는지에 따라 부호 결정
    // Case A: 펄스 증가 -> 좌회전이면 (+)
    // Case B: 펄스 증가 -> 우회전이면 (-)  <-- 테스트 필요
    
    // 여기서는 Case A (Standard Servo) 가정:
    target_pulse = STEER_NEUTRAL_NS + (int32)pwm_offset; 

    // 4. 하드웨어 안전장치 (Output Clamping)
    if (target_pulse < STEER_MIN_NS) target_pulse = STEER_MIN_NS;
    if (target_pulse > STEER_MAX_NS) target_pulse = STEER_MAX_NS;

    // 5. PWM 업데이트
    uint32 wait_cnt = 0;
    PDM_Disable(STEER_PWM_CH, PMM_ON);
    while (PDM_GetChannelStatus(STEER_PWM_CH) && ++wait_cnt < 1000); // Wait

    g_steer_pwm_cfg.mcDutyNanoSec1 = (uint32)target_pulse;
    
    if (PDM_SetConfig(STEER_PWM_CH, &g_steer_pwm_cfg) == SAL_RET_SUCCESS)
    {
        PDM_Enable(STEER_PWM_CH, PMM_ON);
    }

    // mcu_printf("Steer Pulse: %d ns (Ang: %.2f)\n", target_pulse, angular_vel);
}