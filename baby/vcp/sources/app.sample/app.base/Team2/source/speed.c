#include "speed.h"
#include "../team2_header.h"
#include "encoder.h"
#include <math.h>

static PDMModeConfig_t g_pwm_cfg;
static uint8 g_motor_initialized = 0;


#define SAFE_DISTANCE_CM  45.0f
#define ACC_DEAD_ZONE     2.0f

static void motor_init(void)
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
    mcu_printf("motor_init Finish\n");
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
            SAL_TaskSleep(1); //<- 이걸 지우세요!
            for (volatile int i = 0; i < 1000; i++)
                ; // 단순 루프로 아주 짧게 대기

            if (++wait_cnt > 100)
                break; // 카운트를 좀 더 늘려주세요
        }
    }

    g_pwm_cfg.mcDutyNanoSec1 = target_duty;

    if (PDM_SetConfig(MOTOR_PWM_CH, &g_pwm_cfg) == SAL_RET_SUCCESS)
    {
        PDM_Enable(MOTOR_PWM_CH, PMM_ON);
        mcu_printf("motor Enable\n");
    }
    else
    {
        mcu_printf("Speed failed in control_motor_drive\n");
    }
}

// void process_acc_system(float current_dist_cm)
// {
//     if(!g_motor_initialized) motor_init();

//     uint32 target_duty = DUTY_STOP_NS;
//     uint32 wait_cnt = 0;

//     // 1. 거리 기반 단계별 로직 판단
//     if (current_dist_cm <= 5.0f) // 센서 오류 또는 초근접 (비상 정지)
//     {
//         GPIO_Set(MOTOR_IN1, 0);
//         GPIO_Set(MOTOR_IN2, 0);
//         target_duty = DUTY_STOP_NS;
//     }
//     else if (current_dist_cm < (SAFE_DISTANCE_CM - ACC_DEAD_ZONE)) 
//     {
//         // [후진 구간] 목표 거리(50cm)보다 가까울 때
//         GPIO_Set(MOTOR_IN1, 0);
//         GPIO_Set(MOTOR_IN2, 1);
        
//         // 거리가 아주 가까우면 조금 더 빠르게 후진 (최대 50% 출력)
//         if(current_dist_cm < 20.0f) target_duty = (uint32)(PERIOD_NS * 0.50f);
//         else target_duty = (uint32)(PERIOD_NS * 0.30f);
//     }
//     else if (current_dist_cm > (SAFE_DISTANCE_CM + ACC_DEAD_ZONE))
//     {
//         // [전진 구간] 목표 거리(50cm)보다 멀 때
//         GPIO_Set(MOTOR_IN1, 1);
//         GPIO_Set(MOTOR_IN2, 0);

//         // 거리에 따른 속도 차등 부여
//         if (current_dist_cm > 150.0f) {
//             target_duty = (uint32)(PERIOD_NS * 0.70f); // 매우 멀면 70%
//         } else if (current_dist_cm > 80.0f) {
//             target_duty = (uint32)(PERIOD_NS * 0.50f); // 중간 거리 50%
//         } else {
//             target_duty = (uint32)(PERIOD_NS * 0.35f); // 근접 시 35% 서행
//         }
//     }
//     else
//     {
//         // [정지/유지 구간] 데드존(48cm ~ 52cm) 사이
//         GPIO_Set(MOTOR_IN1, 0);
//         GPIO_Set(MOTOR_IN2, 0);
//         target_duty = DUTY_STOP_NS;
//     }

//     // 2. 하드웨어 보호용 최대 출력 제한 (Safety Clamp)
//     if (target_duty > (uint32)(PERIOD_NS * 0.85f)) {
//         target_duty = (uint32)(PERIOD_NS * 0.85f);
//     }

//     // 3. PWM 업데이트 시퀀스 (Disable -> Config -> Enable)
//     PDM_Disable(MOTOR_PWM_CH, PMM_ON);
//     wait_cnt = 0; 
//     while (PDM_GetChannelStatus(MOTOR_PWM_CH)) {
//         for (volatile int i = 0; i < 1000; i++);
//         if (++wait_cnt > 1000) break;
//     }

//     g_pwm_cfg.mcDutyNanoSec1 = target_duty;
//     if (PDM_SetConfig(MOTOR_PWM_CH, &g_pwm_cfg) == SAL_RET_SUCCESS) {
//         PDM_Enable(MOTOR_PWM_CH, PMM_ON);
//     }
// }

/* speed.c의 process_acc_system 수정(테스트) */

void process_acc_system(float current_dist_cm)
{
    if(!g_motor_initialized) motor_init();

    uint32 target_duty = DUTY_STOP_NS;
    uint32 wait_cnt = 0;
    const char* status_msg = "STOP"; // 확인용 문구

    // 1. 거리 기반 제어 로직
    if (current_dist_cm <= 5.0f) {
        status_msg = "EMERGENCY STOP";
        target_duty = DUTY_STOP_NS;
        GPIO_Set(MOTOR_IN1, 0);
        GPIO_Set(MOTOR_IN2, 0);
    }
    else if (current_dist_cm < (SAFE_DISTANCE_CM - ACC_DEAD_ZONE)) {
        // 후진 구간 (목표보다 가까움)
        status_msg = "REVERSE";
        GPIO_Set(MOTOR_IN1, 0);
        GPIO_Set(MOTOR_IN2, 1);
        
        if(current_dist_cm < 20.0f) {
            target_duty = (uint32)(PERIOD_NS * 0.50f);  // 매우 가까우면 50% 후진
        } else {
            target_duty = (uint32)(PERIOD_NS * 0.30f);  // 30% 후진
        }
    }
    else if (current_dist_cm > (SAFE_DISTANCE_CM + ACC_DEAD_ZONE)) {
        // 전진 구간 (목표보다 멀음)
        status_msg = "FORWARD";
        GPIO_Set(MOTOR_IN1, 1);
        GPIO_Set(MOTOR_IN2, 0);
        
        if (current_dist_cm > 150.0f) {
            target_duty = (uint32)(PERIOD_NS * 0.70f);  // 매우 멀면 70%
        } else if (current_dist_cm > 80.0f) {
            target_duty = (uint32)(PERIOD_NS * 0.50f);  // 중간 거리 50%
        } else {
            target_duty = (uint32)(PERIOD_NS * 0.35f);  // 근접 시 35%
        }
    }
    else {
        // Dead-zone (48~52cm)
        status_msg = "HOLDING";
        target_duty = DUTY_STOP_NS;
        GPIO_Set(MOTOR_IN1, 0);
        GPIO_Set(MOTOR_IN2, 0);
    }

    // 2. 하드웨어 보호: 최대 출력 제한
    if (target_duty > (uint32)(PERIOD_NS * 0.85f)) {
        target_duty = (uint32)(PERIOD_NS * 0.85f);
    }

    // 3. PWM 업데이트 (Disable -> Config -> Enable)
    PDM_Disable(MOTOR_PWM_CH, PMM_ON);
    wait_cnt = 0; 
    while (PDM_GetChannelStatus(MOTOR_PWM_CH)) {
        for (volatile int i = 0; i < 1000; i++);
        if (++wait_cnt > 1000) break;
    }

    g_pwm_cfg.mcDutyNanoSec1 = target_duty;
    if (PDM_SetConfig(MOTOR_PWM_CH, &g_pwm_cfg) == SAL_RET_SUCCESS) {
        PDM_Enable(MOTOR_PWM_CH, PMM_ON);
    }

    // 4. 결과 출력 (테스트용 상세 로그)
    uint32 duty_percent = (target_duty * 100) / PERIOD_NS;
    mcu_printf("   >> ACC: Dist=%dcm | %s | Duty=%d%%\n", 
               (int)current_dist_cm, status_msg, duty_percent);
}

// 간단한 PI 제어기 (Proportional + Integral)
#define KP_SPEED  0.08f    // 속도 오차 1cm/s당 12% duty 변화 (부하 고려)
#define KI_SPEED  0.015f   // 적분 게인
#define MAX_INTEGRAL  50.0f  // 적분 포화 (부하 시 더 큰 적분 필요)

static float g_speed_integral = 0.0f;

void process_acc_with_encoder(float current_dist_cm)
{
    if(!g_motor_initialized) motor_init();

    uint32 wait_cnt = 0;
    const char* status_msg = "STOP";
    float target_speed_cms = 0.0f;  // 목표 속도 (cm/s)
    uint8 direction = 0;  // 0: 정지, 1: 전진, 2: 후진

    // 1. 거리 기반 목표 속도 설정
    if (current_dist_cm <= 5.0f) {
        status_msg = "EMERGENCY";
        target_speed_cms = 0.0f;
        direction = 0;
    }
    else if (current_dist_cm < (SAFE_DISTANCE_CM - ACC_DEAD_ZONE)) {
        // 후진 필요
        status_msg = "REVERSE";
        direction = 2;
        
        if(current_dist_cm < 20.0f) {
            target_speed_cms = 10.0f;  // 10cm/s 후진
        } else {
            target_speed_cms = 5.0f;   // 5cm/s 후진
        }
    }
    else if (current_dist_cm > (SAFE_DISTANCE_CM + ACC_DEAD_ZONE)) {
    // 전진 필요 (거리 기반 점진적 감속)
    status_msg = "FORWARD";
    direction = 1;
    
    if (current_dist_cm > 150.0f) {
        target_speed_cms = 10.0f;  // 매우 멀면: 고속 (20cm/s)
    } else if (current_dist_cm > 100.0f) {
        target_speed_cms = 7.0f;  // 멀면: 중속 (15cm/s)
    } else if (current_dist_cm > 70.0f) {
        target_speed_cms = 5.0f;  // 접근 중: 감속 시작 (12cm/s)
    } else if (current_dist_cm > 55.0f) {
        target_speed_cms = 3.0f;   // 목표 근처: 더 감속 (8cm/s)
    } else {
        // 52~55cm: 마지막 접근 구간 (매우 느리게)
        target_speed_cms = 2.0f;   // 저속 접근 (4cm/s)
    }
}
    else {
        // Dead-zone
        status_msg = "HOLDING";
        target_speed_cms = 0.0f;
        direction = 0;
    }

    // 2. 방향 설정
    if (direction == 1) {
        GPIO_Set(MOTOR_IN1, 1);
        GPIO_Set(MOTOR_IN2, 0);
    } else if (direction == 2) {
        GPIO_Set(MOTOR_IN1, 0);
        GPIO_Set(MOTOR_IN2, 1);
    } else {
        GPIO_Set(MOTOR_IN1, 0);
        GPIO_Set(MOTOR_IN2, 0);
        g_speed_integral = 0.0f;  // 정지 시 적분 초기화
    }

    // 3. 엔코더에서 실제 속도 읽기
    float actual_speed_cms = Encoder_GetSpeedCms();
    if (actual_speed_cms < 0) actual_speed_cms = -actual_speed_cms;  // 절댓값

    // 4. PI 제어기 (정지가 아닐 때만)
    uint32 target_duty = DUTY_STOP_NS;
    
    if (direction != 0 && target_speed_cms > 0.1f) {
        // 속도 오차 계산
        float speed_error = target_speed_cms - actual_speed_cms;
        
        // P term (정규화 0~1)
        float p_term = KP_SPEED * speed_error;
        
        // I term (누적)
        g_speed_integral += speed_error * 0.1f;  // dt ≈ 100ms
        if (g_speed_integral > MAX_INTEGRAL) g_speed_integral = MAX_INTEGRAL;
        if (g_speed_integral < -MAX_INTEGRAL) g_speed_integral = -MAX_INTEGRAL;
        float i_term = KI_SPEED * g_speed_integral;
        
        // 정규화된 duty 계산 (0~1)
        float normalized_duty = 0.10f + p_term + i_term;  // 10% 기본 (부하 고려)
        
        // 범위 제한 (10% ~ 85%)
        if (normalized_duty > 0.85f) normalized_duty = 0.85f;
        if (normalized_duty < 0.10f) normalized_duty = 0.10f;
        
        // ns 단위로 변환
        target_duty = (uint32)(PERIOD_NS * normalized_duty);
        
        // PI 상태 디버깅 (가끔씩만 출력)
        static int debug_cnt = 0;
        if (++debug_cnt >= 20) {  // 2초마다
            mcu_printf("   [PI] Err=%.1f P=%.2f I=%.2f Integ=%.1f → Duty=%.0f%%\n",
                       speed_error, p_term*100, i_term*100, g_speed_integral, normalized_duty*100);
            debug_cnt = 0;
        }
    } else {
        target_duty = DUTY_STOP_NS;
        g_speed_integral = 0.0f;
    }

    // 5. PWM 업데이트
    PDM_Disable(MOTOR_PWM_CH, PMM_ON);
    wait_cnt = 0; 
    while (PDM_GetChannelStatus(MOTOR_PWM_CH)) {
        for (volatile int i = 0; i < 1000; i++);
        if (++wait_cnt > 1000) break;
    }

    g_pwm_cfg.mcDutyNanoSec1 = target_duty;
    if (PDM_SetConfig(MOTOR_PWM_CH, &g_pwm_cfg) == SAL_RET_SUCCESS) {
        PDM_Enable(MOTOR_PWM_CH, PMM_ON);
    }

    // 6. 로그 출력
    int dist_int = (int)current_dist_cm;
    int target_speed_int = (int)(target_speed_cms * 10);
    int actual_speed_int = (int)(actual_speed_cms * 10);
    uint32 duty_percent = 0;
    if (PERIOD_NS > 0) {
        duty_percent = (uint32)((target_duty * 100UL) / PERIOD_NS);
    }
    
    float enc_dist_cm = Encoder_GetDistanceCm();
    int enc_dist_int = (int)(enc_dist_cm * 10);
    
    mcu_printf("   >> ACC: Dist=%dcm | %s | Target=%d.%dcm/s Actual=%d.%dcm/s | Duty=%d%% | TotalDist=%d.%dcm\n", 
               dist_int, status_msg, 
               target_speed_int/10, target_speed_int%10,
               actual_speed_int/10, actual_speed_int%10,
               (int)duty_percent,
               enc_dist_int/10, enc_dist_int%10);
}