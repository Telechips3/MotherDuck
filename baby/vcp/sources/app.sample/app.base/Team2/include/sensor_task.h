#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <sal_api.h>
#include "pose.h"

SALRetCode_t SensorTaskCreate(void);
int SensorTask_GetPose(Pose *out);

#endif
