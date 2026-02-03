#ifndef _MPU6050_DRIVER_H_
#define _MPU6050_DRIVER_H_

#include <sal_api.h>
#include <i2c.h>

/* --- 주소 및 레지스터 정의 --- */
#define MPU6050_ADDR_7BIT     (uint8)0x68u
#define MPU6050_WHO_AM_I      (uint8)0x75u
#define MPU6050_PWR_MGMT_1    (uint8)0x6Bu
#define MPU6050_SMPLRT_DIV    (uint8)0x19u
#define MPU6050_CONFIG        (uint8)0x1Au
#define MPU6050_GYRO_CONFIG   (uint8)0x1Bu
#define MPU6050_ACCEL_CONFIG  (uint8)0x1Cu
#define MPU6050_ACCEL_XOUT_H  (uint8)0x3Bu

/* --- 계산용 상수 --- */
#define ALPHA                 0.90f
#define RAD_TO_DEG            57.2957f

/* --- 데이터 구조체 --- */
typedef struct {
    float roll, pitch, yaw;
    int32 gz_bias_raw;
    int32 ax_bias, ay_bias, az_bias;
    uint32 last_tick;
} MPU6050_Filter_t;

/* --- 함수 원형 (외부 공개) --- */
int MPU6050_Init(uint8 ch, uint8 addr7);
int MPU6050_Read_Raw(uint8 ch, uint8 addr7, int16 *ax, int16 *ay, int16 *az, int16 *gx, int16 *gy, int16 *gz);
void MPU6050_Calibrate(uint8 ch, uint8 addr7, MPU6050_Filter_t *pMpu);
void MPU6050_Update_Filter(MPU6050_Filter_t *pMpu, int16 ax, int16 ay, int16 az, int16 gx, int16 gy, int16 gz, float dt);

#endif