#ifndef IMU_H
#define IMU_H

#include <sal_api.h>

/* 외부에서 보는 IMU 데이터 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} IMU_Data_t;

/* main.c에서 호출 */
SALRetCode_t IMUTaskCreate(void);

/* SensorTask에서 사용 */
void IMU_ModuleInit(void);
void IMU_ModuleUpdate(void);

/* 다른 Task에서 사용 */
int IMU_GetData(IMU_Data_t *out);

#endif
