// SPDX-License-Identifier: Apache-2.0

#include "imu.h"
#include "mpu_driver.h"
#include <i2c.h>
#include <debug.h>
#include <app_cfg.h>

/* ===== I2C 설정 ===== */
#define IMU_I2C_CH     0
#define IMU_I2C_PORT   0
#define IMU_I2C_SPEED  100

#define IMU_TASK_PERIOD_MS  20

/* ===== Internal State ===== */
static MPU6050_Filter_t s_mpu;
static IMU_Data_t s_imu;
static uint32 s_lastTick = 0;
static uint8 s_imu_inited = 0;

static void IMU_ModuleInitOnce(void)
{
    I2C_Init();
    I2C_Open(IMU_I2C_CH, IMU_I2C_PORT, IMU_I2C_SPEED, NULL, NULL);

    if (MPU6050_Init(IMU_I2C_CH, MPU6050_ADDR_7BIT) != 0)
    {
        mcu_printf("[IMU] MPU6050 detection failed\n");
        //need fail-safe
    }

    mcu_printf("[IMU] MPU6050 detected\n");
    MPU6050_Calibrate(IMU_I2C_CH, MPU6050_ADDR_7BIT, &s_mpu);
    mcu_printf("[IMU] MPU6050 cal done\n");
    s_lastTick = 0;
    s_imu_inited = 1;
}

void IMU_ModuleInit(void)
{
    if (s_imu_inited == 0)
    {
        mcu_printf("[IMU] init start\n");
        IMU_ModuleInitOnce();
        mcu_printf("[IMU] init done\n");
    }
}

void IMU_ModuleUpdate(void)
{
    if (s_imu_inited == 0)
    {
        IMU_ModuleInitOnce();
    }

    int16 ax, ay, az, gx, gy, gz;
    uint32 now;

    SAL_GetTickCount(&now);
    float dt = (s_lastTick == 0) ? 0.02f : (float)(now - s_lastTick) / 1000.0f;
    s_lastTick = now;

    if (MPU6050_Read_Raw(IMU_I2C_CH, MPU6050_ADDR_7BIT,
                         &ax, &ay, &az, &gx, &gy, &gz) == 0)
    {
        MPU6050_Update_Filter(&s_mpu, ax, ay, az, gx, gy, gz, dt);
        s_imu.roll  = s_mpu.roll;
        s_imu.pitch = s_mpu.pitch;
        s_imu.yaw   = s_mpu.yaw;
    }
}

/* ===== Legacy Task Wrapper ===== */
static void IMUTask(void *pArg)
{
    (void)pArg;

    IMU_ModuleInit();

    while (1)
    {
        IMU_ModuleUpdate();
        SAL_TaskSleep(IMU_TASK_PERIOD_MS);
    }
}

SALRetCode_t IMUTaskCreate(void)
{
    static uint32 imuTaskID;
    static uint32 imuTaskStk[ACFG_TASK_NORMAL_STK_SIZE];

    SALRetCode_t err = SAL_TaskCreate(&imuTaskID,
                                      (const uint8 *)"IMU Task",
                                      IMUTask,
                                      imuTaskStk,
                                      ACFG_TASK_NORMAL_STK_SIZE,
                                      SAL_PRIO_APP_CFG,
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
