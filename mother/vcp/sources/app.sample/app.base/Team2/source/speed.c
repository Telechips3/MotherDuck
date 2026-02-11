#include "../team2_header.h"
#include "speed.h"

static PDMModeConfig_t g_left_pwm_cfg;
static PDMModeConfig_t g_right_pwm_cfg;
static uint8 g_motor_initialized = 0;
static uint32 g_current_duty = 400000;
static uint32 g_steer_duty = 70000;

static void motor_init(void)
{
    GPIO_Config(MOTOR_LEFT_IN1, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(MOTOR_LEFT_IN2, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(MOTOR_RIGHT_IN3, (GPIO_FUNC(0) | GPIO_OUTPUT));
    GPIO_Config(MOTOR_RIGHT_IN4, (GPIO_FUNC(0) | GPIO_OUTPUT));

    g_left_pwm_cfg.mcPortNumber = GPIO_PERICH_CH0;
    g_left_pwm_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    g_left_pwm_cfg.mcInversedSignal = 0;
    g_left_pwm_cfg.mcOutSignalInIdle = 0;
    g_left_pwm_cfg.mcLoopCount = 0;
    g_left_pwm_cfg.mcOutputCtrl = 0;
    g_left_pwm_cfg.mcPeriodNanoSec1 = PERIOD_NS;
    g_left_pwm_cfg.mcPeriodNanoSec2 = 0;

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

void control_motor_speed(uint32 speed_level)
{
    if (!g_motor_initialized) {
        motor_init();
    }

    switch(speed_level) {
        case 1: g_current_duty = DUTY_LEVEL_1_NS; break;
        case 2: g_current_duty = DUTY_LEVEL_2_NS; break;
        case 3: g_current_duty = DUTY_LEVEL_3_NS; break;
        case 4: g_current_duty = DUTY_LEVEL_4_NS; break;
        case 5: g_current_duty = DUTY_LEVEL_5_NS; break;
        case 6: g_current_duty = DUTY_LEVEL_6_NS; break;
        case 7: g_current_duty = DUTY_LEVEL_7_NS; break;
        case 8: g_current_duty = DUTY_LEVEL_8_NS; break;
        case 9: g_current_duty = DUTY_LEVEL_9_NS; break;
        default: g_current_duty = DUTY_LEVEL_3_NS; break;
    }
    
    mcu_printf("[속도 변경] Level %d → %d%%\n", speed_level, speed_level * 10);
}

static void apply_motor_pwm(uint32 left_duty, uint32 right_duty)
{
    uint32 wait_cnt = 0;
    
    // 왼쪽 모터
    PDM_Disable(MOTOR_LEFT_ENA_CH, PMM_ON);
    wait_cnt = 0;
    while (PDM_GetChannelStatus(MOTOR_LEFT_ENA_CH) && ++wait_cnt < 1000);
    g_left_pwm_cfg.mcDutyNanoSec1 = left_duty;
    (void)PDM_SetConfig(MOTOR_LEFT_ENA_CH, &g_left_pwm_cfg);
    PDM_Enable(MOTOR_LEFT_ENA_CH, PMM_ON);

    // 오른쪽 모터
    PDM_Disable(MOTOR_RIGHT_ENB_CH, PMM_ON);
    wait_cnt = 0;
    while (PDM_GetChannelStatus(MOTOR_RIGHT_ENB_CH) && ++wait_cnt < 1000);
    g_right_pwm_cfg.mcDutyNanoSec1 = right_duty;
    (void)PDM_SetConfig(MOTOR_RIGHT_ENB_CH, &g_right_pwm_cfg);
    PDM_Enable(MOTOR_RIGHT_ENB_CH, PMM_ON);
}

void control_motor_drive(uint32 cmd, uint32 is_steer_cmd)
{
    if (!g_motor_initialized) {
        motor_init();
    }

    uint32 left_duty = DUTY_STOP_NS;
    uint32 right_duty = DUTY_STOP_NS;

    switch(cmd)
    {
        case CMD_FORWARD:  // W: 직진
            GPIO_Set(MOTOR_LEFT_IN1, 1);
            GPIO_Set(MOTOR_LEFT_IN2, 0);
            GPIO_Set(MOTOR_RIGHT_IN3, 1);
            GPIO_Set(MOTOR_RIGHT_IN4, 0);
            left_duty = g_current_duty;
            right_duty = g_current_duty;
            left_duty += is_steer_cmd == 1 ? g_steer_duty : 0;
            right_duty += is_steer_cmd == 1? g_steer_duty : 0;
            mcu_printf("[모터] 전진\n");
            break;

        case CMD_BACKWARD:  // S: 후진
            GPIO_Set(MOTOR_LEFT_IN1, 0);
            GPIO_Set(MOTOR_LEFT_IN2, 1);
            GPIO_Set(MOTOR_RIGHT_IN3, 0);
            GPIO_Set(MOTOR_RIGHT_IN4, 1);
            left_duty = g_current_duty;
            right_duty = g_current_duty;
            left_duty += is_steer_cmd == 1 ? g_steer_duty : 0;
            right_duty += is_steer_cmd == 1? g_steer_duty : 0;
            mcu_printf("[모터] 후진\n");
            break;

        case CMD_FORWARD_LEFT:  // W+A: 좌회전 전진
            GPIO_Set(MOTOR_LEFT_IN1, 1);
            GPIO_Set(MOTOR_LEFT_IN2, 0);
            GPIO_Set(MOTOR_RIGHT_IN3, 1);
            GPIO_Set(MOTOR_RIGHT_IN4, 0);
            left_duty = g_current_duty / 2;  // 왼쪽 느리게
            right_duty = g_current_duty;     // 오른쪽 빠르게
            left_duty += is_steer_cmd == 1 ? g_steer_duty : 0;
            right_duty += is_steer_cmd == 1? g_steer_duty : 0;
            mcu_printf("[모터] 좌회전 전진\n");
            break;

        case CMD_FORWARD_RIGHT:  // W+D: 우회전 전진
            GPIO_Set(MOTOR_LEFT_IN1, 1);
            GPIO_Set(MOTOR_LEFT_IN2, 0);
            GPIO_Set(MOTOR_RIGHT_IN3, 1);
            GPIO_Set(MOTOR_RIGHT_IN4, 0);
            left_duty = g_current_duty;      // 왼쪽 빠르게
            right_duty = g_current_duty / 2; // 오른쪽 느리게
            left_duty += is_steer_cmd == 1 ? g_steer_duty : 0;
            right_duty += is_steer_cmd == 1? g_steer_duty : 0;
            mcu_printf("[모터] 우회전 전진\n");
            break;

        case CMD_BACKWARD_LEFT:  // S+A: 좌회전 후진
            GPIO_Set(MOTOR_LEFT_IN1, 0);
            GPIO_Set(MOTOR_LEFT_IN2, 1);
            GPIO_Set(MOTOR_RIGHT_IN3, 0);
            GPIO_Set(MOTOR_RIGHT_IN4, 1);
            left_duty = g_current_duty / 2;
            right_duty = g_current_duty;
            left_duty += is_steer_cmd == 1 ? g_steer_duty : 0;
            right_duty += is_steer_cmd == 1? g_steer_duty : 0;
            mcu_printf("[모터] 좌회전 후진\n");
            break;

        case CMD_BACKWARD_RIGHT:  // S+D: 우회전 후진
            GPIO_Set(MOTOR_LEFT_IN1, 0);
            GPIO_Set(MOTOR_LEFT_IN2, 1);
            GPIO_Set(MOTOR_RIGHT_IN3, 0);
            GPIO_Set(MOTOR_RIGHT_IN4, 1);
            left_duty = g_current_duty;
            right_duty = g_current_duty / 2;
            left_duty += is_steer_cmd == 1 ? g_steer_duty : 0;
            right_duty += is_steer_cmd == 1? g_steer_duty : 0;
            mcu_printf("[모터] 우회전 후진\n");
            break;

        default:  // 정지
            GPIO_Set(MOTOR_LEFT_IN1, 0);
            GPIO_Set(MOTOR_LEFT_IN2, 0);
            GPIO_Set(MOTOR_RIGHT_IN3, 0);
            GPIO_Set(MOTOR_RIGHT_IN4, 0);
            left_duty = DUTY_STOP_NS;
            right_duty = DUTY_STOP_NS;
            mcu_printf("[모터] 정지\n");    
            break;
    }

    apply_motor_pwm(left_duty, right_duty);
}

// #include "../team2_header.h"
// #include "speed.h"
// #include <math.h>

// static PDMModeConfig_t g_left_pwm_cfg;
// static PDMModeConfig_t g_right_pwm_cfg;
// static uint8 g_motor_initialized = 0;

// static void motor_init(void)
// {
//     // GPIO 및 PWM 초기화 (기존 코드와 동일)
//     GPIO_Config(MOTOR_LEFT_IN1, (GPIO_FUNC(0) | GPIO_OUTPUT));
//     GPIO_Config(MOTOR_LEFT_IN2, (GPIO_FUNC(0) | GPIO_OUTPUT));
//     GPIO_Config(MOTOR_RIGHT_IN3, (GPIO_FUNC(0) | GPIO_OUTPUT));
//     GPIO_Config(MOTOR_RIGHT_IN4, (GPIO_FUNC(0) | GPIO_OUTPUT));

//     g_left_pwm_cfg.mcPortNumber = GPIO_PERICH_CH0;
//     g_left_pwm_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
//     g_left_pwm_cfg.mcPeriodNanoSec1 = MOTOR_PERIOD_NS; 

//     g_right_pwm_cfg.mcPortNumber = GPIO_PERICH_CH0;
//     g_right_pwm_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
//     g_right_pwm_cfg.mcPeriodNanoSec1 = MOTOR_PERIOD_NS;

//     g_motor_initialized = 1;
// }

// static void apply_motor_pwm(uint32 duty)
// {
//     // 왼쪽 모터 적용
//     PDM_Disable(MOTOR_LEFT_ENA_CH, PMM_ON);
//     g_left_pwm_cfg.mcDutyNanoSec1 = duty;
//     (void)PDM_SetConfig(MOTOR_LEFT_ENA_CH, &g_left_pwm_cfg);
//     PDM_Enable(MOTOR_LEFT_ENA_CH, PMM_ON);

//     // 오른쪽 모터 적용
//     PDM_Disable(MOTOR_RIGHT_ENB_CH, PMM_ON);
//     g_right_pwm_cfg.mcDutyNanoSec1 = duty;
//     (void)PDM_SetConfig(MOTOR_RIGHT_ENB_CH, &g_right_pwm_cfg);
//     PDM_Enable(MOTOR_RIGHT_ENB_CH, PMM_ON);
// }

// // [수정된 함수] 선속도를 받아 고정 PWM으로 구동
// void control_motor_with_ros(float linear_vel)
// {
//     if (!g_motor_initialized) motor_init();

//     uint32 left_duty = g_current_duty + g_steer_duty;
    
//     // 1. 데드존 처리 (아주 작은 속도 명령은 정지로 간주)
//     if (fabsf(linear_vel) < 0.05f) 
//     {
//         // 정지
//         GPIO_Set(MOTOR_LEFT_IN1, 0);
//         GPIO_Set(MOTOR_LEFT_IN2, 0);
//         GPIO_Set(MOTOR_RIGHT_IN3, 0);
//         GPIO_Set(MOTOR_RIGHT_IN4, 0);
//         apply_motor_pwm(0);
//         return;
//     }

//     // 2. 방향 제어
//     if (linear_vel > 0) 
//     {
//         // 전진
//         GPIO_Set(MOTOR_LEFT_IN1, 1);
//         GPIO_Set(MOTOR_LEFT_IN2, 0);
//         GPIO_Set(MOTOR_RIGHT_IN3, 1);
//         GPIO_Set(MOTOR_RIGHT_IN4, 0);
//     }
//     else 
//     {
//         // 후진
//         GPIO_Set(MOTOR_LEFT_IN1, 0);
//         GPIO_Set(MOTOR_LEFT_IN2, 1);
//         GPIO_Set(MOTOR_RIGHT_IN3, 0);
//         GPIO_Set(MOTOR_RIGHT_IN4, 1);
//     }

//     // 3. 고정 속도 적용 (사용자가 요청한 450,000)
//     apply_motor_pwm(left_duty);
//     // mcu_printf("Speed On: %d ns\n", MOTOR_FIXED_DUTY_NS);
// }