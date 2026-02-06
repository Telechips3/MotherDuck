// SPDX-License-Identifier: Apache-2.0

#include <../team2_header.h>
#include "imu.h"
#include "mpu_driver.h"
#include <i2c.h>
#include <debug.h>
#include <app_cfg.h>

/* ===== I2C 설정 ===== */
#define IMU_I2C_CH     0
#define IMU_I2C_PORT   0
#define IMU_I2C_SPEED  100

/* ===== Internal State ===== */
static MPU6050_Filter_t s_mpu;
static IMU_Data_t s_imu;

static uint32 imuTaskID;
static uint32 imuTaskStk[IMU_TASK_STACK_SIZE];

/* ===== Task ===== */
static void IMUTask(void *pArg)
{
    (void)pArg;

    mcu_printf("[IMU] task entered. \n");
    /* I2C 초기화 */
    I2C_Init();
    I2C_Open(IMU_I2C_CH, IMU_I2C_PORT, IMU_I2C_SPEED, NULL, NULL);
    mcu_printf("[IMU] I2C opended. \n");
    if (MPU6050_Init(IMU_I2C_CH, MPU6050_ADDR_7BIT) != 0)
    {
        mcu_printf("[IMU] MPU6050 detection failed\n");
        while (1)
            SAL_TaskSleep(1000);
    }

    mcu_printf("[IMU] MPU6050 detected\n");

    /* 보정 */
    MPU6050_Calibrate(IMU_I2C_CH, MPU6050_ADDR_7BIT, &s_mpu);

    uint32 lastTick = 0;

    while (1)
    {
        int16 ax, ay, az, gx, gy, gz;
        uint32 now;

        SAL_GetTickCount(&now);
        float dt = (lastTick == 0) ? 0.02f : (float)(now - lastTick) / 1000.0f;
        lastTick = now;

        if (MPU6050_Read_Raw(IMU_I2C_CH, MPU6050_ADDR_7BIT,
                             &ax, &ay, &az, &gx, &gy, &gz) == 0)
        {
            MPU6050_Update_Filter(&s_mpu,
                                  ax, ay, az, gx, gy, gz, dt);

            s_imu.roll  = s_mpu.roll;
            s_imu.pitch = s_mpu.pitch;
            s_imu.yaw   = s_mpu.yaw;
        }

        SAL_TaskSleep(IMU_TASK_SLEEP_MS);
    }
}

/* ===== TaskCreate (main.c에서 호출) ===== */
SALRetCode_t IMUTaskCreate(void)
{
    SALRetCode_t err = SAL_TaskCreate(&imuTaskID,
                                      (const uint8 *)"IMU Task",
                                      IMUTask,
                                      imuTaskStk,
                                      IMU_TASK_STACK_SIZE,
                                      IMU_TASK_PRIORITY,
                                      NULL);

    mcu_printf("IMU task create: %d\n", (int)err);
    return err;
}

/* ===== Getter ===== */
int IMU_GetData(IMU_Data_t *out)
{
    if (!out)
        return -1;

    *out = s_imu;
    return 0;
}
