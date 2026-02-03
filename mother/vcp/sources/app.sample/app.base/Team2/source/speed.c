#include "speed.h"

static PDMModeConfig_t g_left_pwm_cfg;
static PDMModeConfig_t g_right_pwm_cfg;
static PDMModeConfig_t g_servo_cfg;
static uint8 g_motor_initialized = 0;

static uint32 s_current_speed = 0;
static uint32 s_current_steer = SERVO_CENTER_NS;
static uint8  s_current_dir = 0; // 0: 전진, 1: 후진

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

    // 4. 서보모터 설정 (조향)
    g_servo_cfg.mcPortNumber = GPIO_PERICH_CH0;  // GPA 포트
    g_servo_cfg.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    g_servo_cfg.mcInversedSignal = 0;
    g_servo_cfg.mcOutSignalInIdle = 0;
    g_servo_cfg.mcLoopCount = 0;
    g_servo_cfg.mcOutputCtrl = 0;
    g_servo_cfg.mcPeriodNanoSec1 = SERVO_PERIOD_NS;
    g_servo_cfg.mcDutyNanoSec1 = SERVO_CENTER_NS;  // 초기 중립
    g_servo_cfg.mcPeriodNanoSec2 = 0;

    // 서보 초기화
    PDM_SetConfig(SERVO_STEER_CH, &g_servo_cfg);
    PDM_Enable(SERVO_STEER_CH, PMM_ON);

    g_motor_initialized = 1;
}


void control_motor_drive(uint32 cmd)
{
    if (!g_motor_initialized) {
        motor_init(); // 기존 초기화 함수 호출
        g_motor_initialized = 1;
    }

    // 1. 명령 해석: 속도와 조향 변수를 각각 업데이트
    if (cmd == 0) { // W: 전진 가속
        s_current_dir = 0;
        s_current_speed += SPEED_STEP;
        if (s_current_speed > SPEED_MAX) s_current_speed = SPEED_MAX;
    } 
    else if (cmd == 2) { // S: 후진 가속
        s_current_dir = 1;
        s_current_speed += SPEED_STEP;
        if (s_current_speed > SPEED_MAX) s_current_speed = SPEED_MAX;
    }
    else if (cmd == 1) { // A: 왼쪽 조향
        if (s_current_steer > SERVO_LEFT_LIMIT) s_current_steer -= STEER_STEP;
    }
    else if (cmd == 3) { // D: 오른쪽 조향
        if (s_current_steer < SERVO_RIGHT_LIMIT) s_current_steer += STEER_STEP;
    }
    else { // 0xFF (타임아웃): 자동 복원
        if (s_current_speed >= (SPEED_STEP / 2)) s_current_speed -= (SPEED_STEP / 2);
        else s_current_speed = 0;

        if (s_current_steer > SERVO_CENTER_NS + 10000) s_current_steer -= 20000;
        else if (s_current_steer < SERVO_CENTER_NS - 10000) s_current_steer += 20000;
        else s_current_steer = SERVO_CENTER_NS;
    }

    // 2. 방향 GPIO 설정
    if (s_current_speed == 0) {
        GPIO_Set(MOTOR_LEFT_IN1, 0); GPIO_Set(MOTOR_LEFT_IN2, 0);
        GPIO_Set(MOTOR_RIGHT_IN3, 0); GPIO_Set(MOTOR_RIGHT_IN4, 0);
    } else if (s_current_dir == 0) {
        GPIO_Set(MOTOR_LEFT_IN1, 1); GPIO_Set(MOTOR_LEFT_IN2, 0);
        GPIO_Set(MOTOR_RIGHT_IN3, 1); GPIO_Set(MOTOR_RIGHT_IN4, 0);
    } else {
        GPIO_Set(MOTOR_LEFT_IN1, 0); GPIO_Set(MOTOR_LEFT_IN2, 1);
        GPIO_Set(MOTOR_RIGHT_IN3, 0); GPIO_Set(MOTOR_RIGHT_IN4, 1);
    }

    // 3. 하드웨어 PDM 업데이트 (Helper 함수 없이 직접 제어)
    uint32 wait_cnt;

    // --- [왼쪽 모터 PWM] ---
    PDM_Disable(MOTOR_LEFT_ENA_CH, PMM_ON);
    wait_cnt = 0;
    while (PDM_GetChannelStatus(MOTOR_LEFT_ENA_CH) && ++wait_cnt < 1000);
    g_left_pwm_cfg.mcDutyNanoSec1 = s_current_speed;
    (void)PDM_SetConfig(MOTOR_LEFT_ENA_CH, &g_left_pwm_cfg);
    PDM_Enable(MOTOR_LEFT_ENA_CH, PMM_ON);

    // --- [오른쪽 모터 PWM] ---
    PDM_Disable(MOTOR_RIGHT_ENB_CH, PMM_ON);
    wait_cnt = 0;
    while (PDM_GetChannelStatus(MOTOR_RIGHT_ENB_CH) && ++wait_cnt < 1000);
    g_right_pwm_cfg.mcDutyNanoSec1 = s_current_speed;
    (void)PDM_SetConfig(MOTOR_RIGHT_ENB_CH, &g_right_pwm_cfg);
    PDM_Enable(MOTOR_RIGHT_ENB_CH, PMM_ON);

    // --- [서보모터 조향 PWM] ---
    PDM_Disable(SERVO_STEER_CH, PMM_ON);
    wait_cnt = 0;
    while (PDM_GetChannelStatus(SERVO_STEER_CH) && ++wait_cnt < 1000);
    g_servo_cfg.mcDutyNanoSec1 = s_current_steer;
    (void)PDM_SetConfig(SERVO_STEER_CH, &g_servo_cfg);
    PDM_Enable(SERVO_STEER_CH, PMM_ON);
}

