#include "mpu_driver.h"
#include <math.h>

static uint8 g_i2c_cmd;

/* --- 내부 I2C 도우미 함수 --- */
static SALRetCode_t read_reg(uint8 ch, uint8 devAddr7, uint8 reg, uint8 *buf, uint8 len) {
    I2CXfer_t x = {0};
    g_i2c_cmd = reg;
    x.xCmdLen = 1; x.xCmdBuf = &g_i2c_cmd;
    x.xInLen = len; x.xInBuf = buf;
    return I2C_XferCmd(ch, devAddr7 << 1, x, 0);
}

static SALRetCode_t write_reg(uint8 ch, uint8 devAddr7, uint8 reg, uint8 val) {
    I2CXfer_t x = {0};
    uint8 buf[2] = { reg, val };
    x.xCmdLen = 1; x.xCmdBuf = &buf[0];
    x.xOutLen = 1; x.xOutBuf = &buf[1];
    return I2C_XferCmd(ch, devAddr7 << 1, x, 0);
}

/* --- 초기화 및 제어 함수 구현 --- */
int MPU6050_Init(uint8 ch, uint8 addr7) {
    uint8 id = 0;
    if (read_reg(ch, addr7, MPU6050_WHO_AM_I, &id, 1) != SAL_RET_SUCCESS || id != 0x68) return -1;
    write_reg(ch, addr7, MPU6050_PWR_MGMT_1, 0x00);
    write_reg(ch, addr7, MPU6050_SMPLRT_DIV, 0x07);
    write_reg(ch, addr7, MPU6050_CONFIG, 0x03);
    write_reg(ch, addr7, MPU6050_GYRO_CONFIG, 0x00);
    write_reg(ch, addr7, MPU6050_ACCEL_CONFIG, 0x00);
    return 0;
}

void MPU6050_Calibrate(uint8 ch, uint8 addr7, MPU6050_Filter_t *pMpu) {
    int32 s_ax = 0, s_ay = 0, s_az = 0, s_gz = 0;
    int16 ax, ay, az, gx, gy, gz;
    const int n = 200;
    //mcu_printf("[MPU_DRIVER] start =========== \n");
    for (int i = 0; i < n; i++) {
        if (MPU6050_Read_Raw(ch, addr7, &ax, &ay, &az, &gx, &gy, &gz) == 0) {
            s_ax += ax; s_ay += ay; s_az += az; s_gz += gz;
        }
        // mcu_printf("[MPU_DRIVER] loop %d=========== \n",i);
        SAL_TaskSleep(5);
    }
    pMpu->ax_bias = s_ax / n;
    pMpu->ay_bias = s_ay / n;
    pMpu->az_bias = (s_az / n) - 16384;
    pMpu->gz_bias_raw = s_gz / n;
}

int MPU6050_Read_Raw(uint8 ch, uint8 addr7, int16 *ax, int16 *ay, int16 *az, int16 *gx, int16 *gy, int16 *gz) {
    uint8 buf[14] = {0};
    if (read_reg(ch, addr7, MPU6050_ACCEL_XOUT_H, buf, 14) != SAL_RET_SUCCESS) return -1;
    *ax = (int16)((buf[0] << 8) | buf[1]);
    *ay = (int16)((buf[2] << 8) | buf[3]);
    *az = (int16)((buf[4] << 8) | buf[5]);
    *gx = (int16)((buf[8] << 8) | buf[9]);
    *gy = (int16)((buf[10] << 8) | buf[11]);
    *gz = (int16)((buf[12] << 8) | buf[13]);
    return 0;
}

void MPU6050_Update_Filter(MPU6050_Filter_t *pMpu, int16 ax, int16 ay, int16 az, int16 gx, int16 gy, int16 gz, float dt) {
    float c_ax = (float)(ax - pMpu->ax_bias);
    float c_ay = (float)(ay - pMpu->ay_bias);
    float c_az = (float)(az - pMpu->az_bias);
    float gyroX = (float)gx / 131.0f;
    float gyroY = (float)gy / 131.0f;
    float gyroZ = (float)(gz - pMpu->gz_bias_raw) / 131.0f;

    float accPitch = atan2f(c_ay, sqrtf(c_ax * c_ax + c_az * c_az)) * RAD_TO_DEG;
    float accRoll  = atan2f(-c_ax, c_az) * RAD_TO_DEG;

    pMpu->pitch = ALPHA * (pMpu->pitch + gyroY * dt) + (1.0f - ALPHA) * accPitch;
    pMpu->roll  = ALPHA * (pMpu->roll + gyroX * dt) + (1.0f - ALPHA) * accRoll;
    pMpu->yaw  += gyroZ * dt;
}